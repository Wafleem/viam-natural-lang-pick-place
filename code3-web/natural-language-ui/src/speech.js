// Speech is only an input adapter. It cannot propose, execute, or stop a task.
export function setupSpeech(button, input, message, changed) {
  const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!Recognition) {
    button.disabled = true; message.textContent = 'Speech unavailable here. Type your instruction.'; return;
  }
  let recognition = null, active = false, stopping = false, finalText = new Map();
  function start() {
    if (active) return;
    active = true; stopping = false; finalText = new Map(); recognition = new Recognition();
    recognition.lang = 'en-US'; recognition.continuous = true; recognition.interimResults = false;
    recognition.onresult = event => {
      for (let i = event.resultIndex; i < event.results.length; i++) {
        if (event.results[i].isFinal) finalText.set(i, event.results[i][0].transcript);
      }
    };
    recognition.onerror = () => { message.textContent = 'Speech could not finish. Try again or type the instruction.'; };
    recognition.onend = () => {
      const transcript = [...finalText.entries()].sort(([a], [b]) => a - b).map(([, text]) => text).join(' ').trim();
      active = false; button.classList.remove('listening'); button.textContent = 'Hold to speak';
      if (transcript) {
        input.value = transcript; changed();
        message.textContent = transcript.length > 300 ? 'Transcript exceeds 300 characters. Review and shorten it before translating.' : 'Final transcript ready. Review it, then translate.';
      } else if (message.textContent === 'Listening…') message.textContent = 'No final transcript. Try again or type.';
    };
    try {
      recognition.start(); button.classList.add('listening'); button.textContent = 'Release to finish';
      message.textContent = 'Listening…';
    } catch {
      active = false; message.textContent = 'Microphone unavailable. Type your instruction.';
    }
  }
  function stop() { if (active && !stopping) { stopping = true; recognition?.stop(); } }
  button.addEventListener('pointerdown', event => { if (event.button === 0) { button.setPointerCapture(event.pointerId); start(); } });
  button.addEventListener('pointerup', stop); button.addEventListener('pointercancel', stop);
  button.addEventListener('lostpointercapture', stop);
  button.addEventListener('keydown', event => {
    if (event.code === 'Space' || event.code === 'Enter') { event.preventDefault(); if (!event.repeat) start(); }
  });
  button.addEventListener('keyup', event => {
    if (event.code === 'Space' || event.code === 'Enter') { event.preventDefault(); stop(); }
  });
  button.addEventListener('blur', stop); window.addEventListener('blur', stop);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
}
