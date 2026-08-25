#!/usr/bin/env bash
#
# Install the Click'n Fly desktop icon. Run once:
#     ./install_launcher.sh
#
# Writes ~/.local/share/applications/clicknfly.desktop with absolute paths
# resolved from this repo's location, so the app appears in the applications
# menu and can be launched (or pinned) without a terminal.
#
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
desktop="$apps/clicknfly.desktop"

chmod +x "$here/run_clicknfly.sh"
mkdir -p "$apps"
cat > "$desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=Click'n Fly
GenericName=Drone show operator interface
Comment=Quad4D: fly choreographed quadrotor shows in the cage
Exec=$here/run_clicknfly.sh
Icon=$here/src/qt_gui/media/clicknfly.svg
Path=$here
Terminal=false
StartupNotify=true
Categories=Science;Education;
Keywords=drone;quadrotor;show;paparazzi;quad4d;
EOF
chmod +x "$desktop"

# refresh the menu cache when the desktop environment provides the tool
command -v update-desktop-database >/dev/null 2>&1 && \
    update-desktop-database "$apps" >/dev/null 2>&1 || true

echo "Installed: $desktop"
echo "  Exec: $here/run_clicknfly.sh"
echo "  Icon: $here/src/qt_gui/media/clicknfly.svg"
echo
echo "Click'n Fly should now be in the applications menu (search 'Click')."
echo "If the app needs more than the virtualenv (PYTHONPATH, PAPARAZZI_HOME...),"
echo "put those exports in ~/.config/clicknfly.env -- a desktop launch does not"
echo "read your ~/.bashrc. Startup log: ~/.cache/clicknfly.log"
