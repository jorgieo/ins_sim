# Download & Install

Prebuilt portable bundles for Windows and Linux are attached to every GitHub
Release. No Python installation is required — download, extract, and run.

[Download for Windows](https://github.com/jorgieo/ins_sim/releases/latest){ .md-button .md-button--primary }
[Download for Linux](https://github.com/jorgieo/ins_sim/releases/latest){ .md-button .md-button--primary }

On the release page, grab the asset for your platform:

| Platform | Asset | Run |
| -------- | ----- | --- |
| Windows (64-bit) | `ins_sim-vX.Y.Z-windows-x86_64.zip` | extract, run `ins_sim.exe` |
| Linux (x86-64) | `ins_sim-vX.Y.Z-linux-x86_64.tar.gz` | extract, run `ins_sim` |

## Windows

1. Download the `.zip` asset and extract it anywhere (no installer).
2. Open the extracted `ins_sim` folder and run `ins_sim.exe`.

!!! note "Windows SmartScreen"
    The binaries are not code-signed, so the first launch may show
    *"Windows protected your PC"*. Click **More info → Run anyway**.
    Keep `ins_sim.exe` inside its folder — it needs the bundled libraries
    beside it.

## Linux

```bash
tar xzf ins_sim-vX.Y.Z-linux-x86_64.tar.gz
./ins_sim/ins_sim
```

!!! note "Runtime requirements"
    Bundles are built on Ubuntu 24.04, so a similarly recent glibc is
    required. Minimal installations may also need the Qt WebEngine system
    libraries:

    ```bash
    sudo apt-get install libegl1 libgl1 libxkbcommon0 libnss3 libasound2t64
    ```

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
