#!/usr/bin/env bash
set -euo pipefail

profile="${1:-}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$profile" in
  mac | pi5)
    ;;
  *)
    echo "Usage: $0 mac|pi5" >&2
    exit 2
    ;;
esac

mkdir -p "$root/.vscode"
cp "$root/.vscode/settings.${profile}.example.json" "$root/.vscode/settings.json"
echo "Activated IDE profile: ${profile}"
