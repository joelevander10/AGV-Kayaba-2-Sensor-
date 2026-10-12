#!/usr/bin/env bash
# =============================================================================
#  install_kiosk.sh — show the AGV dashboard automatically when the AGV boots.
#  Run ONCE on the AGV PC, as the desktop user (NOT with sudo):
#      cd ~/agv2 && ./kiosk/install_kiosk.sh
#  What it does
#    1. Installs /usr/local/bin/agv-dashboard (outside the project folder, so it
#       keeps working after the source is deleted for the release).
#    2. Adds ~/.config/autostart/agv-dashboard.desktop → opens the dashboard
#       full-screen at every desktop login.
#    3. Turns on automatic login for this user (GDM), so no password screen.
#    4. Stops the screen from blanking / locking.
#  Undo: ./kiosk/install_kiosk.sh --remove
# =============================================================================
set -euo pipefail
ME="$(id -un)"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSTART="$HOME/.config/autostart/agv-dashboard.desktop"
GDM_CONF=/etc/gdm3/custom.conf

[[ $EUID -ne 0 ]] || { echo "Run as your normal desktop user, not with sudo."; exit 1; }

if [[ "${1:-}" == "--remove" ]]; then
  rm -f "$AUTOSTART"
  sudo rm -f /usr/local/bin/agv-dashboard
  if [[ -f "$GDM_CONF" ]]; then
    sudo sed -i 's/^AutomaticLoginEnable *=.*/AutomaticLoginEnable=false/' "$GDM_CONF"
  fi
  gsettings set org.gnome.desktop.session idle-delay 300 2>/dev/null || true
  gsettings set org.gnome.desktop.screensaver lock-enabled true 2>/dev/null || true
  echo "Kiosk autostart removed (auto-login off, screen lock back on)."
  exit 0
fi

# 1. Launcher
sudo install -m 0755 -o root -g root "$SRC_DIR/agv-dashboard" /usr/local/bin/agv-dashboard
echo "✔ /usr/local/bin/agv-dashboard"

# 2. Autostart entry
mkdir -p "$(dirname "$AUTOSTART")"
cat > "$AUTOSTART" <<DESK
[Desktop Entry]
Type=Application
Name=AGV Dashboard
Comment=Open the AGV web dashboard full-screen
Exec=/usr/local/bin/agv-dashboard
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=5
NoDisplay=false
DESK
echo "✔ $AUTOSTART"

# 3. Automatic login (GDM / Ubuntu desktop)
if [[ -f "$GDM_CONF" ]]; then
  sudo cp -n "$GDM_CONF" "$GDM_CONF.bak-agv" 2>/dev/null || true
  sudo python3 - "$GDM_CONF" "$ME" <<'PY'
import re, sys
path, user = sys.argv[1], sys.argv[2]
s = open(path).read()
if "[daemon]" not in s:
    s = "[daemon]\n" + s
def put(s, key, val):
    pat = re.compile(rf"^[# \t]*{key}[ \t]*=.*$", re.M)
    if pat.search(s):
        return pat.sub(f"{key}={val}", s, count=1)
    return s.replace("[daemon]", f"[daemon]\n{key}={val}", 1)
s = put(s, "AutomaticLoginEnable", "true")
s = put(s, "AutomaticLogin", user)
open(path, "w").write(s)
PY
  echo "✔ automatic login for '$ME' ($GDM_CONF)"
else
  echo "! $GDM_CONF not found — turn on automatic login in Settings → Users."
fi

# 4. Never blank or lock the screen
gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null || true
gsettings set org.gnome.desktop.screensaver lock-enabled false 2>/dev/null || true
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null || true
echo "✔ screen blanking / lock off"

command -v chromium >/dev/null 2>&1 || command -v google-chrome >/dev/null 2>&1 || \
  echo "Tip: Chromium gives the cleanest kiosk (no restore prompts): sudo snap install chromium"
echo
echo "Done. Reboot to test:  sudo reboot"
echo "Leave the full-screen dashboard with Alt+F4. Open it by hand: agv-dashboard"
