# Command-line runs

`main.py`, at the top of the source repository, runs the same simulation as
the desktop app without its window. It suits batch runs, and computers
where the app cannot be installed. It comes with the source code, not with
the downloadable app: install from source first, as in
[Your first script](../get-started/first-script.md#install-the-package).

## Running it

From the repository folder:

```bash
python main.py                      # packaged mission and IMU, 20 trials
python main.py --trials 100 --dt 0.1 --imu-spec my_imu.yaml
```

| Option | Default | Meaning |
| ------ | ------- | ------- |
| `--config` | the packaged mission | Mission YAML file |
| `--imu-spec` | the packaged navigation-grade spec | IMU spec YAML file |
| `--trials` | 20 | Number of Monte Carlo trials |
| `--dt` | 0.1 | Time step [s] |
| `--seed` | 42 | Random seed; the app always uses 42 |

Baro altitude aiding is always on. `python main.py --help` lists the
options.

## What it prints

With `--trials 10 --dt 1.0` it prints:

```text
Total path  : 440.2 nm  (815 km)
Duration    : 79.8 min  (4790 s)
Samples     : 4,791  (dt=1.0 s)
Start g     : 9.78530 m/s²
Zero-noise error  start: 0.000 m  max: 0.0 m  end: 0.0 m
  t = 8.0 min (10%): err = 0.0 m
  t = 19.9 min (25%): err = 0.0 m
  t = 39.9 min (50%): err = 0.0 m
  t = 59.9 min (75%): err = 0.0 m
  t = 79.8 min (100%): err = 0.0 m
Max |dv_eb_n/dt|: 5.65 m/s²
Max |f_ib_b|: 11.36 m/s²
95th-pct error  start:  0.000 m   end: 6687.6 m
Monte Carlo simulation time: 0h 0m 17s
Interactive map saved: .../ins_sim/maps/trajectory_map.html
```

- **The mission:** its length in nautical miles ("nm") and kilometres, its
  duration, the number of samples, and gravity at the start.
- **The zero-noise check:** the navigator, fed the outputs of a perfect
  IMU, must follow the truth exactly; the error is printed at five points
  of the flight. It is 0.0 m for any mission, because the perfect outputs
  are built to match the navigator, so it checks the software rather than
  your mission; see
  [Validation](../validation/index.md#zero-noise-round-trip).
- **The largest acceleration and specific force** along the mission, a
  check that the flight is physically reasonable. 11.36 m/s² is about
  1.16 g, in the holding patterns' 30° turns.
- **The result:** the 95th percentile of the 3D position error at the start
  and the end, then the run time.

## What it shows

When the run finishes, a window opens with a summary figure of six panels:
the 3D track with its 95th-percentile error tube, the 3D position error of
every trial, the CEP with its linear fit, and the North tilt, East tilt
and heading errors. Close the window to end the program.

It also saves the ground-track map, the app's **Map** tab, as
`maps/trajectory_map.html` next to `main.py`. Open it in a web browser; the
map tiles need an internet connection.

## Without a display

On a computer without a display, such as a server you log in to remotely,
set the `MPLBACKEND` environment variable to `Agg`. The program then skips the
summary window, with a warning that it cannot show it, and still saves the
map:

```bash
MPLBACKEND=Agg python main.py --trials 100
```

For anything more, such as saving the numbers or looping over IMU specs,
write a short script instead: see [Your first script](../get-started/first-script.md)
and [Recipes](recipes.md).
