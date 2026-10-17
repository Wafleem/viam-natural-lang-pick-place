import * as VIAM from '@viamrobotics/sdk';
// SDK loading never connects. Only the explicit Connect action dials a machine.
export function validateHost(input) {
  const host = typeof input === 'string' ? input.trim().toLowerCase() : '';
  if (host.length > 253 || !/^(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+viam\.cloud$/.test(host)) {
    throw new Error('Enter the public machine hostname ending in .viam.cloud, without a URL scheme or path.');
  }
  return host;
}

export function getHostedConnection(pathname, cookieText) {
  const match = /^\/machine\/([^/]+)(?:\/|$)/.exec(pathname);
  if (!match) throw new Error('Open this app through a Viam single-machine application session.');
  let machineId, data;
  try {
    machineId = decodeURIComponent(match[1]);
    const cookie = cookieText.split(';').map(value => value.trim())
      .find(value => value.slice(0, value.indexOf('=')) === machineId);
    if (!cookie) throw new Error();
    data = JSON.parse(decodeURIComponent(cookie.slice(cookie.indexOf('=') + 1)));
  } catch {
    throw new Error('The selected machine session is missing or invalid. Reopen this app through Viam login.');
  }
  if (data?.machineId !== machineId || data.credentials?.type !== 'api-key') {
    throw new Error('The selected machine session does not match this page.');
  }
  return validateCredentials({ host: data.hostname, keyId: data.credentials.authEntity, apiKey: data.credentials.payload });
}

function validateCredentials({ host, keyId, apiKey }) {
  host = validateHost(host);
  if (typeof keyId !== 'string' || !keyId.trim() || keyId.length > 200
    || typeof apiKey !== 'string' || !apiKey.trim() || apiKey.length > 10000) {
    throw new Error('Enter a machine API key ID and API key.');
  }
  return { host, keyId: keyId.trim(), apiKey };
}

const sortedKeys = value => value && typeof value === 'object' && !Array.isArray(value) ? Object.keys(value).sort().join(',') : null;
const validPlace = place => {
  const keys = sortedKeys(place);
  return (keys === 'type' && ['ring', 'box'].includes(place.type))
    || (keys === 'color' && typeof place.color === 'string' && /^[a-z]{3,12}$/.test(place.color));
};

function validateCommand(command) {
  const keys = sortedKeys(command);
  if (keys === 'action' && command.action === 'status') return;
  const reviewedId = typeof command?.request_id === 'string'
    && /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(command.request_id);
  if (keys === 'action,pick,place,request_id,workcell_ready' && command.action === 'pick_place'
    && command.workcell_ready === true && reviewedId
    && sortedKeys(command.pick) === 'object,select' && command.pick.object === 'cube'
    && ['only', 'left', 'right'].includes(command.pick.select) && validPlace(command.place)) return;
  throw new Error('Only status or one reviewed pick_place command can use this connection.');
}

export async function createViamConnection(options, {
  loadSdk = async () => VIAM,
} = {}) {
  const { host, keyId, apiKey } = validateCredentials(options);
  const onDisconnect = options.onDisconnect ?? (() => {});
  let machine;
  try {
    const sdk = await loadSdk();
    machine = await sdk.createRobotClient({
      host, credentials: { type: 'api-key', authEntity: keyId, payload: apiKey },
      signalingAddress: 'https://app.viam.com:443', noReconnect: true, dialTimeoutMs: 15000,
    });
    const service = new sdk.GenericServiceClient(machine, 'code-2');
    let active = true;
    const changed = ({ eventType }) => {
      if (active && ['reconnecting', 'reconnection_failed', 'disconnecting', 'disconnected'].includes(eventType)) {
        active = false; onDisconnect();
      }
    };
    machine.on('connectionstatechange', changed);
    return {
      host,
      async command(payload) {
        validateCommand(payload);
        if (!active) throw new Error('Reconnect and read current status before sending.');
        // Exactly one SDK invocation. RPC failures never trigger a retry.
        return service.doCommand(payload, { timeoutMs: 15000 });
      },
      async disconnect() {
        active = false; machine.off('connectionstatechange', changed);
        // Closing a connection is not a robot stop command.
        await machine.disconnect();
      },
    };
  } catch {
    if (machine) { try { await machine.disconnect(); } catch {} }
    // Do not echo SDK errors, credentials, or credential-bearing dial configuration.
    throw new Error('Could not connect to Viam. Check the machine address, API key and network access.');
  }
}
