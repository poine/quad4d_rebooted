# quad4d_rebooted — Click'n Fly

Click'n Fly is a trajectory editor/generator and a flight director for quadrotor
drones. It is intended as a tool for drone demonstrations in ENAC's indoor
flight arena, but might fit other applications. It is written in Python, with
the graphical user interface leveraging Qt.

It generates choreographed trajectories for several quadrotors, deconflicts them
before the flight, and flies them from a single interface: preparing, launching,
following and stopping a show without touching a command line.

It builds on the [Paparazzi](https://github.com/paparazzi/paparazzi) autopilot
for the flight itself and on the [pat](https://github.com/poine/pat) library for
the trajectories. Drone positions come from the arena's motion capture system,
not from GPS.

Project documentation is [here](https://poine.github.io/quad4d_rebooted).

## Requirements

Three things, installed separately — this repository holds the application only:

| | where to get it |
|---|---|
| Python 3.10 or later | your system package manager |
| `pat`, providing the `pat3` module | <https://github.com/poine/pat> |
| Paparazzi, with `sw/lib/python` | <https://github.com/paparazzi/paparazzi> |

You also need an arena equipped with OptiTrack, broadcasting `EXTERNAL_POSE`
messages.

## Installation

**1. Python.** Check what you have first:

```bash
python3 --version
```

If it is missing, or older than 3.10, on Debian or Ubuntu:

```bash
sudo apt install python3 python3-venv python3-pip
```

**2. The pat library.** It is not on PyPI, so clone it; its path is declared in
step 4. The location is up to you:

```bash
mkdir -p ~/Projects && git clone https://github.com/poine/pat.git ~/Projects/pat
```

Two things to watch here:

- the repository is named `pat`, the Python module `pat3`;
- **the module lives under `src/`, not at the root.** It is therefore `pat/src`
  that goes into `PYTHONPATH`; point at the clone root and the import fails on a
  perfectly good checkout.

It is also not the same repository as this application, though both are Antoine
Drouin's.

**3. The Python environment.** The launcher looks for `~/venv_quad4d` by
default:

```bash
python3 -m venv ~/venv_quad4d
source ~/venv_quad4d/bin/activate
pip install pyyaml numpy scipy matplotlib pyside6 numpy-stl pyqtgraph pyopengl ivy-python lxml
```

The same list sits in `src/qt_gui/requirements.txt`, so this does the same
thing if you prefer it:

```bash
pip install -r src/qt_gui/requirements.txt
```

Neither pulls in pat or Paparazzi: neither is on PyPI, and step 4 reaches them
through `PYTHONPATH` instead.


**4. The paths to pat and Paparazzi.** A launch from the desktop icon does not
read your `~/.bashrc`, so `PYTHONPATH` has to be declared in a file of its own,
`~/.config/clicknfly.env`, which the launcher loads on every start.

```bash
mkdir -p ~/.config
cat > ~/.config/clicknfly.env <<'EOF'
export PYTHONPATH="$PYTHONPATH:$HOME/Projects/pat/src"
export PYTHONPATH="$PYTHONPATH:/path/to/paparazzi/sw/lib/python"
export PAPARAZZI_HOME="/path/to/paparazzi"
EOF
```

The first line assumes the clone from step 2; adapt it if you put it elsewhere,
and replace the Paparazzi path with your own. This is the most common first-run
mistake: without these paths the application stops on
`ModuleNotFoundError: pat3`.

To check, without leaving the venv:

```bash
source ~/.config/clicknfly.env && python3 -c "import pat3; print(pat3.__file__)"
```

A word on `cat > ... <<'EOF'` blocks: pasted in one go into a terminal, they
sometimes collapse onto a single line and produce an unusable file, or a
`cat: export: No such file or directory`. If that happens, write the file with
an editor (`nano ~/.config/clicknfly.env`) rather than fighting the clipboard.

**5. The desktop icon.** One command, from the root of the repository:

```bash
./install_launcher.sh
```

It writes `~/.local/share/applications/clicknfly.desktop` with absolute paths
resolved from the repository's own location. "Click'n Fly" then appears in the
applications menu, and can be pinned.

The script resolves the repository path by itself: run it from another clone and
the icon switches to that one. There is only ever one desktop entry, and the
previous one is replaced.

**6. Verify.** Close the terminal, open a fresh one, and launch from the icon.
It has to start in a terminal that prepared nothing — that is the only test that
proves the installation stands on its own.

## Running

**From the icon**, which is the operating procedure. The launcher activates the
venv and loads `clicknfly.env` itself, on every start: nothing to type, nothing
to prepare.

**From a console**, you have to provide yourself what the launcher does on its
own:

```bash
source ~/venv_quad4d/bin/activate
cd path/to/quad4d_rebooted/src/qt_gui && ./click_n_fly.py
```

The venv and `PYTHONPATH` belong to **the terminal**, not to the machine: they
vanish when it closes, and a fresh terminal knows nothing of them. That is why
the icon works while a `./click_n_fly.py` typed in a new window fails on
`ModuleNotFoundError: pat3` — the two launch paths do not prepare the same
environment.

To stop thinking about it in a console, have every new terminal load the paths,
once and for all:

```bash
echo '[ -f "$HOME/.config/clicknfly.env" ] && . "$HOME/.config/clicknfly.env"' >> ~/.bashrc
```

The venv stays a manual step on purpose: it has no business imposing itself on
every shell of the machine.

Useful options:

| option | effect |
|---|---|
| `-v`, `--verbose` | developer detail: transit mode chosen, layering, scheduling |
| `--scen NAME` | start directly on a scenario |

Without `-v` the log stays to the point: warnings and the few key events.

**A launch from the icon has no terminal to write to.** On failure an error
window appears, and the full log is in:

```bash
tail -30 ~/.cache/clicknfly.log
```

## Before flying in the arena

Three things decide whether a demonstration flies at all, and the application
detects none of them:

- **Telemetry has to be the lightened configuration.** With the default one, the
  volume of messages the drones emit saturates the link at the expense of the
  motion capture positions, and commands stop getting through reliably.
- **Each drone must be paired with its own radio transmitter.** It is a safety
  requirement: without it, the drone will not fly.
- **Each drone must carry firmware that accepts the guided mode**, otherwise it
  stays in NAV. Reprogram the autopilot if needed.

Battery thresholds are not in the code: they are read from the `BAT` section of
each drone's `airframe` file, the very one the autopilot flies with. Changing a
threshold therefore needs no software change.

## Where to find what

| path | contents |
|---|---|
| `src/qt_gui/click_n_fly.py` | the application |
| `src/qt_gui/traj_factory.py` | the figures |
| `src/qt_gui/scenarios.py` | the predefined scenarios |
| `src/qt_gui/spatial_deconfliction.py` | deconfliction by path scheduling |
| `src/qt_gui/data/` | operator-composed scenarios, local to each machine |
| `docs/concept_operationnel.md` | the operations concept |
| `docs/trajectories.md` | the trajectories |


The files under `src/qt_gui/data/` are excluded from version control: they are
local to each installation. A fresh clone therefore starts without the previous
one's custom scenarios.
