# Download & Install

Prebuilt portable bundles for Windows and Linux (x86-64 and ARM64) are attached
to every GitHub Release. No Python installation is required — download, extract,
and run.

[Download for Windows](https://github.com/jorgieo/ins_sim/releases/latest){ .md-button .md-button--primary }
[Download for Linux](https://github.com/jorgieo/ins_sim/releases/latest){ .md-button .md-button--primary }

On the release page, grab the asset for your platform:

| Platform | Asset | Run |
| -------- | ----- | --- |
| Windows (64-bit) | `ins_sim-vX.Y.Z-windows-x86_64.zip` | extract, run `ins_sim.exe` |
| Linux (x86-64) | `ins_sim-vX.Y.Z-linux-x86_64.tar.gz` | extract, run `ins_sim` |
| Linux (ARM64) | `ins_sim-vX.Y.Z-linux-aarch64.tar.gz` | extract, run `ins_sim` |

!!! tip "Which Linux asset?"
    Run `uname -m` on the target machine: `x86_64` → use the **x86-64** asset;
    `aarch64` → use the **ARM64** asset. The ARM64 bundle covers
    **Raspberry Pi 4/5 running the 64-bit Raspberry Pi OS** and other
    ARM64/aarch64 Linux. 32-bit Raspberry Pi OS (`armv7l`) is not covered by a
    prebuilt bundle — see [Raspberry Pi (32-bit)](#raspberry-pi-32-bit) below.

## Windows

1. Download the `.zip` asset and extract it anywhere (no installer).
2. Open the extracted `ins_sim` folder and run `ins_sim.exe`.

!!! note "Windows SmartScreen"
    The binaries are not code-signed, so the first launch may show
    *"Windows protected your PC"*. Click **More info → Run anyway**.
    Keep `ins_sim.exe` inside its folder — it needs the bundled libraries
    beside it.

## Linux

Pick the asset matching your CPU (`uname -m`) — `x86_64` or `aarch64`:

```bash
# Intel/AMD 64-bit
tar xzf ins_sim-vX.Y.Z-linux-x86_64.tar.gz
./ins_sim/ins_sim

# ARM64 (aarch64)
tar xzf ins_sim-vX.Y.Z-linux-aarch64.tar.gz
./ins_sim/ins_sim
```

!!! note "Runtime requirements"
    Bundles are built on Ubuntu 24.04, so a similarly recent glibc is
    required. Minimal installations may also need the Qt WebEngine system
    libraries:

```bash
sudo apt-get install libegl1 libgl1 libxkbcommon0 libnss3 libasound2t64
```

### Raspberry Pi

The **ARM64** bundle (`ins_sim-vX.Y.Z-linux-aarch64.tar.gz`) is the one for a
Raspberry Pi 4 or 5 — but only when it is running the **64-bit Raspberry Pi
OS**. Confirm first:

```bash
uname -m        # aarch64  → use the ARM64 bundle
                # armv7l   → 32-bit OS, see below
```

#### Raspberry Pi (32-bit)

There is **no prebuilt bundle for 32-bit Raspberry Pi OS** (`armv7l`/armhf), and
the desktop GUI cannot be installed there either — PySide6 publishes no 32-bit
ARM wheels, so `pip install ".[gui]"` will not resolve. Two options, best
first:

1. **Reflash to the 64-bit Raspberry Pi OS** (the Pi 4/5 support it) and use the
   `aarch64` bundle above, or run the GUI [from source](#run-from-source).
2. **Stay on 32-bit and run the headless CLI from source** — it needs only
   numpy/scipy/pyyaml/matplotlib/plotly, no Qt:

    ```bash
    git clone https://github.com/jorgieo/ins_sim.git
    cd ins_sim
    pip install -e .          # base install, NOT ".[gui]"
    python main.py            # writes an interactive maps/trajectory_map.html
    ```

    This produces the matplotlib summary figure and a standalone interactive
    ground-track map (open `maps/trajectory_map.html` in a browser) without the
    desktop front end. See the CLI options with `python main.py --help`.

## Internet access

The application runs fully offline with one exception: the **Map** tab loads
OpenStreetMap basemap tiles from the network. Every other visualization works
without a connection.

## Run from source

Prefer a Python environment? Clone the repository and launch the GUI
directly:

```bash
git clone https://github.com/jorgieo/ins_sim.git
cd ins_sim
pip install ".[gui]"
python -m ins_sim.gui
```
