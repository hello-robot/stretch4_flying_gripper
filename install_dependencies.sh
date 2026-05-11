#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Get the directory where the script is located
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REQ_FILE="$DIR/requirements.txt"

if [ ! -f "$REQ_FILE" ]; then
    echo "Error: $REQ_FILE not found."
    exit 1
fi

echo "Installing unmet Python dependencies from requirements.txt..."

# Use the current Python environment's pip to install dependencies.
# The 'install' command automatically skips packages that are already met.
if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)"; then
    python3 -m pip install -r "$REQ_FILE" --break-system-packages
else
    python3 -m pip install -r "$REQ_FILE"
fi

echo "Dependencies successfully checked and installed."
