#!/usr/bin/env bash
#
# Launch the Click'n Fly operator interface without a terminal (desktop icon).
#
# Reproduces the manual procedure:
#     source ~/venv_quad4d/bin/activate
#     cd <repo>/src/qt_gui && ./click_n_fly.py
#
# The repo path is resolved from this script's own location, so the launcher
# keeps working if the repo moves. Overridable:
#   QUAD4D_VENV          virtualenv to activate   (default: ~/venv_quad4d)
#   ~/.config/clicknfly.env   sourced if present, for anything else the app
#                             needs (PYTHONPATH, PAPARAZZI_HOME, ...) -- a
#                             desktop launch does NOT read your ~/.bashrc.
#
set -u
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
log="${XDG_CACHE_HOME:-$HOME/.cache}/clicknfly.log"
mkdir -p "$(dirname "$log")"

{
    echo "=== $(date '+%F %T')  starting Click'n Fly from $here ==="

    venv="${QUAD4D_VENV:-$HOME/venv_quad4d}"
    if [ -f "$venv/bin/activate" ]; then
        # shellcheck disable=SC1091
        source "$venv/bin/activate"
        echo "venv: $venv"
    else
        echo "venv: none found at $venv (using the system python)"
    fi

    # A desktop session has no PYTHONPATH at all, so the usual .bashrc idiom
    # (PYTHONPATH=$PYTHONPATH:/some/path) would trip `set -u` in the env file.
    # Define it, empty, so appending to it always works.
    export PYTHONPATH="${PYTHONPATH:-}"

    env_file="${XDG_CONFIG_HOME:-$HOME/.config}/clicknfly.env"
    if [ -f "$env_file" ]; then
        # shellcheck disable=SC1090
        source "$env_file"
        echo "env file: $env_file"
    fi

    cd "$here/src/qt_gui" || exit 1
    echo "running: ./click_n_fly.py $*"
    ./click_n_fly.py "$@"
} >>"$log" 2>&1
rc=$?


if [ $rc -ne 0 ]; then
    msg="Click'n Fly stopped with error $rc.

Last lines of $log:

$(tail -n 15 "$log")"
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --width=700 --title="Click'n Fly" --text="$msg"
    elif command -v kdialog >/dev/null 2>&1; then
        kdialog --error "$msg"
    else
        echo "$msg" >&2
    fi
fi
exit $rc
