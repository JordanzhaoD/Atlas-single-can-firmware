#!/usr/bin/env bash
# pio-build.sh — hermetic ESP build wrapper for waveshare-single-can-firmware.
#
# Why this exists (2026-11-19): three recurring build blockers, each of which
# cost a session hours before this wrapper captured them:
#   1. Shared ~/.platformio package collisions. The dual-CAN firmware session
#      replaced the shared toolchain-xtensa-esp-elf with its esp-14.2.0_20260121
#      build while leaving a hand-edited .piopm claiming 15.2.0+20251204, so
#      our espressif32@7.0.0 pin resolved to binaries our ESP-IDF rejects
#      (tool_version_check demands esp-15.2.0_20251204). FIXED STRUCTURALLY by
#      core_dir = .pio-core in platformio.ini — this repo owns its packages.
#   2. Homebrew Python has no default CA trust store, so pio's embedded pip
#      cannot verify pypi.org while installing the ESP-IDF python env
#      ("unable to get local issuer certificate" → idf_component_manager
#      ModuleNotFoundError). Fixed by exporting SSL_CERT_FILE=/etc/ssl/cert.pem.
#   3. platformio.org registry access stalls without a proxy on this network.
#      Fixed by routing through the NAS Clash exit (192.168.1.5:7891) when it
#      is reachable; builds that need no downloads work fine without it.
#
# Usage: scripts/pio-build.sh [-e ENV] [extra pio run args...]
#   Default env: waveshare_single_can_standalone
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ENV="waveshare_single_can_standalone"
if [[ "${1:-}" == "-e" ]]; then ENV="$2"; shift 2; fi

# (1) CA trust for the embedded pip / ESP-IDF python env (macOS system bundle).
if [[ -z "${SSL_CERT_FILE:-}" && -f /etc/ssl/cert.pem ]]; then
  export SSL_CERT_FILE=/etc/ssl/cert.pem
fi

# (2) Proxy for pio registry/pypi traffic: NAS Clash exit when reachable.
#      Override with PIO_BUILD_PROXY=host:port, or PIO_BUILD_PROXY=none to
#      force a direct connection.
if [[ -z "${PIO_BUILD_PROXY+x}" ]]; then
  if [[ -n "${https_proxy:-}" ]]; then
    PIO_BUILD_PROXY="(inherited)"
  elif nc -z -w 1 192.168.1.5 7891 2>/dev/null; then
    export https_proxy=http://192.168.1.5:7891 http_proxy=http://192.168.1.5:7891
    PIO_BUILD_PROXY="192.168.1.5:7891"
  else
    PIO_BUILD_PROXY="none (direct)"
  fi
elif [[ "${PIO_BUILD_PROXY}" != "none" ]]; then
  export https_proxy="http://${PIO_BUILD_PROXY}" http_proxy="http://${PIO_BUILD_PROXY}"
fi

# (3) Preflight: the hermetic core must hold the honest xtensa toolchain
#      (gcc must self-report esp-15.2.0_20251204; a mismatch here means the
#      .pio-core seed was disturbed — see the core_dir comment in
#      platformio.ini before touching anything).
TC_GCC=".pio-core/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc"
if [[ "$ENV" == waveshare_single_can_standalone ]]; then
  if [[ ! -x "$TC_GCC" ]]; then
    echo "pio-build.sh: hermetic toolchain missing ($TC_GCC)" >&2
    echo "  first run on this machine: reseed .pio-core (see platformio.ini comment)" >&2
    exit 1
  fi
  if ! "$TC_GCC" --version | head -1 | grep -q "esp-15.2.0_20251204"; then
    echo "pio-build.sh: hermetic toolchain version drift:" >&2
    "$TC_GCC" --version | head -1 >&2
    echo "  expected esp-15.2.0_20251204 (espressif32@7.0.0 / framework-espidf 4.60001.0 pin)" >&2
    exit 1
  fi
fi

echo "pio-build.sh: env=$ENV proxy=$PIO_BUILD_PROXY ssl_cert=${SSL_CERT_FILE:-none}"
exec pio run -e "$ENV" "$@"
