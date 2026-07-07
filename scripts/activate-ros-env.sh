#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file="${ROTATION_ESTIMATION_ENV_FILE:-$root/.env}"

if [[ -f "$env_file" ]]; then
  set -a
  # This file is local developer configuration and is intentionally shell-style.
  # shellcheck disable=SC1090
  source "$env_file"
  set +a
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if [[ -n "${ZENOH_CONFIG_OVERRIDE:-}" ]]; then
  export ZENOH_CONFIG_OVERRIDE
elif [[ -n "${ROS_ZENOH_ENDPOINT:-}" ]]; then
  case "${ROS_ZENOH_MODE:-router}" in
    client)
      export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ROS_ZENOH_ENDPOINT}\"]"
      ;;
    router | "")
      export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"${ROS_ZENOH_ENDPOINT}\"]"
      ;;
    *)
      echo "Unsupported ROS_ZENOH_MODE='${ROS_ZENOH_MODE}'. Use 'router' or 'client'." >&2
      exit 2
      ;;
  esac
fi
