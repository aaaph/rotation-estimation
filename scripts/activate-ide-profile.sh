#!/usr/bin/env bash
set -euo pipefail

profile="${1:-}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$profile" in
  dev-runtime | edge-runtime)
    ;;
  *)
    echo "Usage: $0 dev-runtime|edge-runtime" >&2
    exit 2
    ;;
esac

mkdir -p "$root/.vscode"
cp "$root/.vscode/settings.${profile}.example.json" "$root/.vscode/settings.json"
echo "Activated IDE profile: ${profile}"
