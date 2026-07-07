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

if [[ -n "${ZENOH_ROUTER_CONFIG_OVERRIDE:-}" ]]; then
  export ZENOH_CONFIG_OVERRIDE="${ZENOH_ROUTER_CONFIG_OVERRIDE}"
else
  override_parts=()

  if [[ -n "${ROS_ZENOH_ROUTER_LISTEN_ENDPOINT:-}" ]]; then
    override_parts+=("listen/endpoints=[\"${ROS_ZENOH_ROUTER_LISTEN_ENDPOINT}\"]")
  fi

  router_connect_endpoint="${ROS_ZENOH_ROUTER_CONNECT_ENDPOINT:-}"

  # Backward compatibility for older local .env files.
  if [[ -z "$router_connect_endpoint" && "${ROS_ZENOH_MODE:-}" == "router" ]]; then
    router_connect_endpoint="${ROS_ZENOH_ENDPOINT:-}"
  fi

  if [[ -n "$router_connect_endpoint" ]]; then
    override_parts+=("connect/endpoints=[\"${router_connect_endpoint}\"]")
  fi

  if ((${#override_parts[@]} > 0)); then
    IFS=";"
    export ZENOH_CONFIG_OVERRIDE="${override_parts[*]}"
    unset IFS
  fi
fi

exec ros2 run rmw_zenoh_cpp rmw_zenohd
