#!/usr/bin/env bash
# =============================================================================
#  build_release.sh — compile the AGV controller and install it as a locked,
#  source-free release on THIS Ubuntu machine.
# =============================================================================
#
#  What it does
#    1. Installs the build tools (gcc, patchelf, ccache, python3-dev) and Nuitka.
#    2. Compiles main.py and everything it imports into a native binary
#       (build/main.dist/main.bin). No .py source goes into the release.
#    3. Installs the release to /opt/agv2 (owned by root, readable only by the
#       service account), with writable logs/ and analysis_plot/ folders.
#    4. Creates the system user "agv" (no login shell) that runs the controller,
#       adds it to "dialout" for the CANable2 USB adapter.
#    5. Writes /etc/systemd/system/agv-controller.service (the old one, if any,
#       is backed up first) and the sudoers rule the HMI restart button needs.
#    6. Starts the service and checks the web API answers.
#    7. Asks for the web engineer password on the first install (kept on rebuilds).
#
#  It does NOT delete your source code. Delete it yourself only after the
#  release has been tested on the AGV — the checklist printed at the end shows how.
#
#  Usage (run from the project folder, on the AGV's own Ubuntu PC):
#      chmod +x build_release.sh
#      ./build_release.sh                 # build + install + start
#      ./build_release.sh --build-only    # just compile into build/, install nothing
#
#  Settings (override with environment variables if needed):
#      AGV_ID=agv2_kim  RELEASE_DIR=/opt/agv2  SERVICE_USER=agv  ./build_release.sh
# =============================================================================
set -euo pipefail

AGV_ID="${AGV_ID:-agv2_kim}"
RELEASE_DIR="${RELEASE_DIR:-/opt/agv2}"
SERVICE_USER="${SERVICE_USER:-agv}"
SERVICE_NAME="agv-controller.service"     # must match _RESTART_CMD in app/app.py
UNIT_FILE="/etc/systemd/system/${SERVICE_NAME}"
SUDOERS_FILE="/etc/sudoers.d/agv-controller"
WEB_PORT=5000

BUILD_ONLY=0
[[ "${1:-}" == "--build-only" ]] && BUILD_ONLY=1

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SRC_DIR}/build"
DIST_DIR="${BUILD_DIR}/main.dist"

say()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[32m✔\033[0m %s\n' "$*"; }
die()  { printf '\n\033[1;31m✘ %s\033[0m\n' "$*" >&2; exit 1; }

# ── 0. Sanity checks ─────────────────────────────────────────────────────────
[[ -f "${SRC_DIR}/main.py" ]] || die "Run this from the AGV project folder (main.py not found)."
[[ -f "${SRC_DIR}/profiles/${AGV_ID}.json" ]] || die "Profile profiles/${AGV_ID}.json not found."
[[ "$(uname -s)" == "Linux" ]] || die "Build on the AGV's Ubuntu PC (or an identical Ubuntu + CPU)."
if [[ $EUID -eq 0 ]]; then
  die "Run as your normal user, not with sudo — the script asks for sudo only where needed."
fi

# Use the project's virtualenv if there is one, so the build sees the same
# packages (pymodbus, python-can, flask, …) the controller runs with.
if [[ -x "${SRC_DIR}/.venv/bin/python" ]]; then
  PY="${SRC_DIR}/.venv/bin/python"
else
  PY="$(command -v python3)"
fi
ok "Python: ${PY} ($("${PY}" --version 2>&1))"

# The profile must load before we spend minutes compiling.
( cd "${SRC_DIR}" && AGV_ID="${AGV_ID}" "${PY}" -c "import config" >/dev/null ) \
  || die "profiles/${AGV_ID}.json does not load — fix it first (run: AGV_ID=${AGV_ID} ${PY} -c 'import config')."
ok "Profile ${AGV_ID} loads"

# ── 1. Build tools ───────────────────────────────────────────────────────────
say "Installing build tools (sudo)"
# Internet is needed for apt and pip — fail fast instead of hanging.
if ! timeout 8 bash -c 'exec 3<>/dev/tcp/archive.ubuntu.com/80' 2>/dev/null; then
  die "No internet (cannot reach archive.ubuntu.com). Connect the WiFi to a network with internet and run again."
fi
ok "Internet OK"
# Ubuntu's automatic updates hold the apt lock for a while after boot.
if pgrep -x apt-get >/dev/null || pgrep -x apt >/dev/null || pgrep -x dpkg >/dev/null || pgrep -f unattended-upgrade >/dev/null; then
  echo "    Ubuntu is installing updates in the background — waiting for it to finish (can take a few minutes)…"
fi
sudo apt-get -o DPkg::Lock::Timeout=900 update
sudo apt-get -o DPkg::Lock::Timeout=900 install -y build-essential patchelf ccache python3-dev
echo "    Installing Nuitka and the Python packages (pip)…"
"${PY}" -m pip install --upgrade nuitka ordered-set zstandard
"${PY}" -m pip install -r "${SRC_DIR}/requirements.txt"
ok "gcc, patchelf, ccache, Nuitka $("${PY}" -m nuitka --version 2>/dev/null | head -1)"

# ── 2. Compile ───────────────────────────────────────────────────────────────
say "Compiling with Nuitka (this takes several minutes the first time)"
rm -rf "${BUILD_DIR}"
cd "${SRC_DIR}"
"${PY}" -m nuitka \
  --standalone \
  --assume-yes-for-downloads \
  --include-data-dir=app/templates=app/templates \
  --include-data-dir=profiles=profiles \
  --include-module=can.interfaces.slcan \
  --include-package=serial \
  --nofollow-import-to=tkinter \
  --output-dir="${BUILD_DIR}" \
  --remove-output \
  --jobs="$(nproc)" \
  main.py
[[ -x "${DIST_DIR}/main.bin" ]] || die "Build failed: ${DIST_DIR}/main.bin not created."

# No source may leak into the release.
find "${DIST_DIR}" \( -name '*.py' -o -name '*.pyc' -o -name '__pycache__' \) -prune -exec rm -rf {} + 2>/dev/null || true
ok "Built ${DIST_DIR}/main.bin ($(du -sh "${DIST_DIR}" | cut -f1))"

if [[ ${BUILD_ONLY} -eq 1 ]]; then
  say "Build only — nothing installed."
  echo "    Test it by hand:  cd ${DIST_DIR} && AGV_ID=${AGV_ID} ./main.bin"
  exit 0
fi

# ── 3. Service account ───────────────────────────────────────────────────────
say "Service account '${SERVICE_USER}' (sudo)"
if ! id "${SERVICE_USER}" &>/dev/null; then
  sudo useradd --system --no-create-home --shell /usr/sbin/nologin "${SERVICE_USER}"
  ok "Created system user ${SERVICE_USER} (no login)"
else
  ok "User ${SERVICE_USER} already exists"
fi
sudo usermod -aG dialout "${SERVICE_USER}"     # CANable2 USB serial adapter
ok "${SERVICE_USER} is in group dialout"

# ── 4. Install to RELEASE_DIR ────────────────────────────────────────────────
say "Installing to ${RELEASE_DIR} (sudo)"
if systemctl list-unit-files "${SERVICE_NAME}" &>/dev/null && systemctl is-active --quiet "${SERVICE_NAME}"; then
  sudo systemctl stop "${SERVICE_NAME}"
  ok "Stopped running ${SERVICE_NAME}"
fi
BACKUP=""
if [[ -d "${RELEASE_DIR}" ]]; then
  BACKUP="${RELEASE_DIR}.old-$(date +%Y%m%d-%H%M%S)"
  sudo mv "${RELEASE_DIR}" "${BACKUP}"
  ok "Previous release moved to ${BACKUP}"
fi
sudo mkdir -p "${RELEASE_DIR}"
sudo cp -a "${DIST_DIR}/." "${RELEASE_DIR}/"
sudo mkdir -p "${RELEASE_DIR}/logs" "${RELEASE_DIR}/analysis_plot"
# Keep the engineer password from the previous release.
if [[ -n "${BACKUP}" ]] && sudo test -f "${BACKUP}/engineer_auth.json"; then
  sudo cp -a "${BACKUP}/engineer_auth.json" "${RELEASE_DIR}/engineer_auth.json"
  ok "Engineer password carried over from the previous release"
fi

# root owns everything; the service group may read/execute; nobody else sees it.
sudo chown -R root:"${SERVICE_USER}" "${RELEASE_DIR}"
sudo chmod -R u=rwX,g=rX,o= "${RELEASE_DIR}"
sudo chmod 0750 "${RELEASE_DIR}/main.bin"
# Writable runtime folders for the controller only.
sudo chown -R "${SERVICE_USER}:${SERVICE_USER}" "${RELEASE_DIR}/logs" "${RELEASE_DIR}/analysis_plot"
sudo chmod 0750 "${RELEASE_DIR}/logs" "${RELEASE_DIR}/analysis_plot"
ok "Installed; profile is at ${RELEASE_DIR}/profiles/${AGV_ID}.json (edit with sudo)"

# ── 5. sudoers rule for the HMI restart button ───────────────────────────────
say "Restart-button permission (sudo)"
TMP_SUDO="$(mktemp)"
cat > "${TMP_SUDO}" <<EOF
# Allow the AGV controller (HMI /errors page) to restart only its own service.
${SERVICE_USER} ALL=(root) NOPASSWD: /usr/bin/systemctl restart ${SERVICE_NAME}
EOF
sudo visudo -cf "${TMP_SUDO}" >/dev/null || die "sudoers rule failed validation"
sudo install -m 0440 -o root -g root "${TMP_SUDO}" "${SUDOERS_FILE}"
rm -f "${TMP_SUDO}"
ok "${SUDOERS_FILE}"

# ── 6. systemd unit ──────────────────────────────────────────────────────────
say "systemd service ${SERVICE_NAME} (sudo)"
if [[ -f "${UNIT_FILE}" ]]; then
  sudo cp "${UNIT_FILE}" "${UNIT_FILE}.bak-$(date +%Y%m%d-%H%M%S)"
  ok "Old unit backed up"
fi
TMP_UNIT="$(mktemp)"
cat > "${TMP_UNIT}" <<EOF
[Unit]
Description=AGV2 Kayaba controller (compiled release)
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=${SERVICE_USER}
Group=${SERVICE_USER}
SupplementaryGroups=dialout
WorkingDirectory=${RELEASE_DIR}
Environment=AGV_ID=${AGV_ID}
Environment=MPLBACKEND=Agg
Environment=PYTHONUNBUFFERED=1
ExecStart=${RELEASE_DIR}/main.bin
Restart=always
RestartSec=3
# Hardening: the controller cannot write anywhere except its own folders.
ProtectSystem=full
ProtectHome=true
PrivateTmp=true
ReadWritePaths=${RELEASE_DIR}/logs ${RELEASE_DIR}/analysis_plot

[Install]
WantedBy=multi-user.target
EOF
sudo install -m 0644 -o root -g root "${TMP_UNIT}" "${UNIT_FILE}"
rm -f "${TMP_UNIT}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}" >/dev/null
sudo systemctl restart "${SERVICE_NAME}"
ok "${UNIT_FILE} enabled and started"

# ── 7. Smoke test ────────────────────────────────────────────────────────────
say "Checking the controller"
sleep 6
if systemctl is-active --quiet "${SERVICE_NAME}"; then
  ok "Service is running"
else
  sudo journalctl -u "${SERVICE_NAME}" -n 40 --no-pager || true
  die "Service is not running — see the log above."
fi
if command -v curl &>/dev/null && curl -fsS "http://127.0.0.1:${WEB_PORT}/api/state" >/dev/null; then
  ok "Web HMI answers on port ${WEB_PORT}"
else
  echo "    (could not reach http://127.0.0.1:${WEB_PORT}/api/state yet — check in a browser)"
fi

# ── 8. Engineer password (web HMI lock) ─────────────────────────────────────
if ! sudo test -f "${RELEASE_DIR}/engineer_auth.json"; then
  say "Set the engineer password for the web dashboard (PARAMS, IO TEST, bypass, calibration)"
  sudo AGV_ID="${AGV_ID}" "${RELEASE_DIR}/main.bin" --set-engineer-password \
    || echo "    (not set — run later: sudo AGV_ID=${AGV_ID} ${RELEASE_DIR}/main.bin --set-engineer-password)"
else
  ok "Engineer password is set"
fi

# ── 9. Checklist ─────────────────────────────────────────────────────────────
cat <<EOF

$(printf '\033[1;32m')Release installed.$(printf '\033[0m')

  Binary   : ${RELEASE_DIR}/main.bin   (no source code inside)
  Profile  : ${RELEASE_DIR}/profiles/${AGV_ID}.json
  Logs     : ${RELEASE_DIR}/logs        live: sudo journalctl -u ${SERVICE_NAME} -f
  Control  : sudo systemctl {status|restart|stop} ${SERVICE_NAME}
  Engineer : change the web password with
             sudo AGV_ID=${AGV_ID} ${RELEASE_DIR}/main.bin --set-engineer-password

  BEFORE HANDING OVER — test AUTO, reverse + CC hitch, pusher, lidar and every
  web page on the real AGV. Then remove everything readable:

    rm -rf ${SRC_DIR}                      # the source + .git (keep your copy on GitHub!)
    gh auth logout 2>/dev/null; rm -f ~/.git-credentials
    ls ~/.ssh                              # remove keys that can reach your repo
    sudo deluser <buyer-user> sudo         # if the buyer gets a login, never give it sudo
    sudo systemctl disable --now ssh       # unless you need remote access

  Also: set a BIOS/UEFI password, disable USB boot, and consider LUKS disk
  encryption with TPM unlock so the SSD can't be read in another computer.

  To update later: get the new source on the AGV, run ./build_release.sh again,
  then delete the source again.
EOF
