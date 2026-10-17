#!/bin/sh

cd `dirname $0`

VENV_NAME="venv"
PYTHON="$VENV_NAME/bin/python"
ENV_ERROR="This module requires Python >=3.8, pip, and virtualenv to be installed."

# The venv persists across cloud builds via a cache mount; create only when missing.
if [ -x "$PYTHON" ]; then
    echo "Existing venv found — skipping creation."
else
    if ! python3 -m venv "$VENV_NAME" >/dev/null 2>&1; then
        echo "Failed to create virtualenv."
        if command -v apt-get >/dev/null; then
            echo "Detected Debian/Ubuntu, attempting to install python3-venv automatically."
            SUDO="sudo"
            if ! command -v $SUDO >/dev/null; then SUDO=""; fi
            if ! apt info python3-venv >/dev/null 2>&1; then
                $SUDO apt -qq update >/dev/null
            fi
            $SUDO apt install -qqy python3-venv >/dev/null 2>&1
            if ! python3 -m venv "$VENV_NAME" >/dev/null 2>&1; then
                echo $ENV_ERROR >&2; exit 1
            fi
        else
            echo $ENV_ERROR >&2; exit 1
        fi
    fi
fi

# Reinstall only when requirements.txt changes; the hash marker lives inside
# venv/ so it travels with the cached venv. No -U: it hits PyPI every build.
REQ_HASH=$( (sha256sum requirements.txt 2>/dev/null || shasum -a 256 requirements.txt) | awk '{print $1}')
# Empty hash (no sha tool) must install, not read as a cache hit.
if [ -z "$REQ_HASH" ] || [ "$(cat "$VENV_NAME/.installed" 2>/dev/null)" != "$REQ_HASH" ]; then
    echo "Installing Python packages..."
    # --no-index resolves from the image's PIP_FIND_LINKS wheelhouse (viam-cloud-build
    # builder/Dockerfile.golang) — keep its wheels in sync with these pins.
    if "$PYTHON" -m pip install --no-index -r requirements.txt -q 2>/dev/null; then
        echo "Installed from local wheelhouse."
    else
        echo "Wheelhouse unavailable or incomplete — installing from PyPI..."
        "$PYTHON" -m pip install -r requirements.txt -q || exit 1
    fi
    echo "$REQ_HASH" > "$VENV_NAME/.installed"
else
    echo "Requirements unchanged — reusing cached venv."
fi
