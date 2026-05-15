#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Get the directory where the script is located
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "Installing unmet Python dependencies from pyproject.toml..."

# Use the current Python environment's pip to install dependencies.
# The 'install' command automatically skips packages that are already met.
if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)"; then
    python3 -m pip install . --break-system-packages
else
    python3 -m pip install .
fi

echo "Dependencies successfully checked and installed."
