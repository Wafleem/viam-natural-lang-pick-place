#!/bin/sh

cd `dirname $0`

VENV_NAME="venv"
PYTHON="$VENV_NAME/bin/python"

# pyinstaller persists in the cached venv; wheelhouse first, PyPI fallback (see setup.sh).
if ! "$PYTHON" -c "import PyInstaller" 2>/dev/null; then
    echo "Installing pyinstaller..."
    "$PYTHON" -m pip install --no-index pyinstaller -q 2>/dev/null         || "$PYTHON" -m pip install pyinstaller -q || exit 1
else
    echo "pyinstaller already present — skipping install."
fi

# onedir skips onefile's zlib-9 PKG archive (~16s in cloud builds); tar -czf below is the sole compressor.
"$PYTHON" -m PyInstaller --onedir --noconfirm --collect-all viam --hidden-import="googleapiclient" src/main.py
echo "PyInstaller output size: $(du -sh dist/main | cut -f1)"

tar -czf dist/archive.tar.gz meta.json ./dist/main
echo "Archive size: $(du -h dist/archive.tar.gz | cut -f1)"
