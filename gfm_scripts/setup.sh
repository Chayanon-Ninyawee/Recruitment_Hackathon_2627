#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Setting up environment..."

# tmux
ln -sf "${SCRIPT_DIR}/.tmux.conf" "${HOME}/.tmux.conf"
echo "  tmux config installed"

echo
echo "Setup complete."
