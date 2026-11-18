# Contributing to ins_sim

Thanks for helping improve ins_sim. This guide covers the development setup
and the conventions that keep the code and the documentation consistent
with each other.

## Setup

```bash
pip install -e ".[dev,gui,docs]"   # editable install with every extra
python -m pytest                   # unit tests, doctests and the docs notation lint
mkdocs serve                       # documentation preview at http://127.0.0.1:8000
```

## Notation: one symbol system for code and docs

ins_sim follows the notation of P. D. Groves, *Principles of GNSS,
Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. (Artech
House, 2013). The reference for every symbol is the
[Notation & conventions](docs/theory/notation.md) page; read it before
writing documentation or physics code.

The essentials:

- A kinematic vector x_βα^γ describes the motion of frame α **relative
  to** frame β, **resolved in** the axes of frame γ. For example, ω_ib^b is
  the body's angular rate relative to inertial space, in body axes.
- C_β^α transforms a vector's components from β axes to α axes.
- A tilde marks a measurement, a hat marks an estimate, and errors are
  always **estimate minus truth**.
- R_N is the **meridian** radius of curvature and R_E the **transverse**
  one. Many other texts use R_N for the transverse radius; do not follow
  them.
- Everything inside the simulator is SI. Datasheet and aviation units
  (degrees, feet, knots, °/h, µg) appear only in YAML files, the CLI and
  GUI displays, and are converted at the boundary.

### Python identifiers

Physics quantities use ASCII versions of the Groves symbols, in the style
of Groves' companion MATLAB code. The names match the equations that
define them, so a reader can move between a docs page and the source
without a dictionary.

| Pattern | Example | Meaning |
| ------- | ------- | ------- |
| `<qty>_<ref><obj>_<axes>` | `v_eb_n` | Velocity of b relative to e, resolved in n |
| `C_<from>_<to>` | `C_b_n` | Transformation from body to NED axes (a scipy `Rotation`) |
| Curvilinear position | `L_b`, `lambda_b`, `h_b` | Latitude, longitude, height |
| Role prefixes | `true_`, `meas_`, `est_` | Truth, measurement, estimate, when the role would otherwise be ambiguous |
| Errors | `delta_psi_nb_n`, `delta_v_eb_n` | Estimate minus truth |

Capitalised names such as `L_b`, `R_N`, `C_b_n` and `M_a` are deliberate
exceptions to PEP 8: they match the printed symbols.

The convention covers the public API and the navigation-equation code
(Earth model, truth IMU outputs, IMU error model, strapdown mechanization,
Monte Carlo). Some names stay as they are:

- YAML keys, command-line flags and GUI labels keep their user-facing
  names (for example `dt` for the integration interval τ_i).
- The private mission-geometry helpers in `trajectory/kinematics.py`
  (`_ground_roll`, `_turn`, `_geodetic_bearing`, ...) keep descriptive
  names such as `hdg_deg` and `lat_deg`, because they work in the YAML's
  aviation units rather than in navigation-equation quantities.

## Documentation style guide

The documentation lives in `docs/` and is built with MkDocs Material.
Pages follow a small number of types:

| Type | Purpose | Lives in |
| ---- | ------- | -------- |
| Tutorial | A guided first experience (first GUI run, first script) | `docs/get-started/` |
| How-to guide | Accomplish a task (write a mission, model an IMU) | `docs/guide/` |
| Theory | Explain the physics and mathematics as implemented | `docs/theory/` |
| Validation | Show a model reproducing an analytic prediction | `docs/validation/` |
| Reference | Look something up (API, YAML schemas, glossary) | `docs/reference/` |

### Theory pages

Theory pages are written to be read by someone learning inertial
navigation, and they follow the same structure:

1. **Plain-language introduction**: what the topic is, why it drives
   navigation error, and where its effect shows up in the GUI. No equations.
2. **Key results**: an `!!! abstract "Key results"` box with two to four
   bullets for readers who skim.
3. **Physical picture**: a figure or diagram before any equation. Flows
   and loops are Mermaid diagrams; geometry is an inline SVG (see
   [Figures](#figures)).
4. **Continuous-time model**: every equation is introduced by a sentence
   and followed by a "where:" list that gives each new symbol's meaning and
   SI unit.
5. **As implemented**: the steps the code takes, each in
   `=== "Equation"` / `=== "Code"` content tabs (see below). Where the code
   matches or deliberately differs from Groves, say so in a
   `!!! note "Compared with Groves (2013)"` box.
6. **What it looks like**: a figure produced by the simulator, linked to
   the validation check that tests it.
7. **Assumptions & limitations**: an `!!! warning` box.
8. **In the code**: a table mapping each equation to the function that
   implements it, plus one tested example.
9. **References**: footnotes pointing to the [bibliography](docs/reference/bibliography.md).

Keep a page under about 2,500 words; split it if it grows beyond that.

### Writing mathematics

Inline math is written `\( ... \)` and display math `$$ ... $$`. Every
frame-indexed symbol goes through a macro defined in
`docs/javascripts/mathjax.js`, so the notation is typeset identically on
every page. `tests/test_docs_notation.py` fails if a page writes such a
symbol in raw LaTeX.

| Macro | Renders as | Meaning |
| ----- | ---------- | ------- |
| `\Cmat{b}{n}` | C_b^n | Transformation from b to n |
| `\wvec{ib}{b}` | ω_ib^b | Angular rate |
| `\Wmat{ie}{n}` | Ω_ie^n | Skew-symmetric form of an angular rate |
| `\fvec{ib}{b}` | f_ib^b | Specific force |
| `\vvec{eb}{n}` | v_eb^n | Velocity |
| `\rvec{eb}{e}` | r_eb^e | Position |
| `\gvec{b}{n}` | g_b^n | Acceleration due to gravity |
| `\qvec{b}{n}` | q_b^n | Attitude quaternion |
| `\avec{ib}{b}` | α_ib^b | Attitude increment over one IMU interval |
| `\uvec{ib}{b}` | υ_ib^b | Velocity increment over one IMU interval |
| `\psivec{nb}{n}` | ψ_nb^n | Attitude error vector (write `\delta\psivec{nb}{n}`) |
| `\skew{\mathbf{x}}` | [x∧] | Skew-symmetric (cross-product) matrix |
| `\Ithree` | I_3 | 3×3 identity |

Frame-indexed macros take an optional first argument that decorates the
symbol: `\wvec[\tilde]{ib}{b}` is the gyro measurement and
`\Cmat[\hat]{b}{n}` the computed attitude.

### Equations next to the code that implements them

Discrete-time steps show the equation and the source lines side by side.
The source lines are pulled from the code at build time, so they cannot
drift. Mark the region in the Python source:

```python
# --8<-- [start:attitude_update]
...
# --8<-- [end:attitude_update]
```

and include it from the page with a `pymdownx.snippets` line inside a
`=== "Code"` tab, using the form
`ins_sim/navigation/strapdown.py:attitude_update` as the snippet path.
The build fails if the file or the section does not exist.

### Citing Groves and other sources

- Cite Groves by chapter. Quote an equation or section number only after
  checking it against the **second edition (2013)**; the first edition
  numbers them differently.
- Use the book's notation, but write every explanation, derivation, figure
  and table in your own words. Do not reproduce its text, figures or
  tables.
- Add each new source to `docs/reference/bibliography.md` and cite it with
  a footnote.

### Figures

Figures that show simulator output are produced by
`tools/generate_doc_plots.py`, never edited by hand, so they can be
regenerated after a model change. `python tools/generate_doc_plots.py
--validation` regenerates the Validation page: its figures and the results
table in `includes/validation_results.md`, from the scenarios in
`tools/validation_scenarios.py`, which `tests/validation/` also asserts.
`python tools/generate_doc_plots.py --theory` regenerates the figures that
only the Theory pages show (`tools/theory_figures.py`).
`python tools/generate_doc_plots.py --guide` retakes the screenshots of the
app in `docs/assets/gui/` (`tools/gui_screenshots.py`, which opens the app
on screen and needs an internet connection for the map tiles) and the
figure of the "Your first script" tutorial.

Diagrams that are not simulator output, such as the frames on the Frames &
attitude page, are inline SVG files in `includes/diagrams/`. A page
includes one with a snippet line inside a `<figure class="ins-figure">`
element. The SVG takes its colours from `docs/stylesheets/extra.css`, which
maps them to the theme's colour variables, so the diagram follows the
light and dark palettes.

### Examples

Every code example in the documentation must run. Docstring examples are
doctests, collected by `python -m pytest`. Longer examples live as scripts
under `docs/examples/` and are included into pages with snippets, so the
tested file is the one readers see. To show a script's output on a page,
save it next to the script as a `.txt` file
(`python docs/examples/<name>.py > docs/examples/<name>.txt`) and include
that too: the test then checks that the script still prints exactly that
output.

The tests run each script in an empty temporary folder, so a script must
find its data files relative to itself (`Path(__file__).parent`), and any
file it saves stays out of the repository. Keep scripts quick, a few
seconds where possible: they run on every test run. Example IMU specs in
`docs/examples/imu/` and missions in `docs/examples/missions/` are tested
too: every spec must load and every mission must build.

### Glossary and tooltips

Abbreviations and terms are defined once, in `includes/abbreviations.md`.
That file supplies the hover tooltips on every page and the entries on the
Glossary page. Add new terms there, not to individual pages.

## Docstrings

Public functions and classes use Google-style docstrings. Each argument
states its frame, shape and SI unit, using the Groves-style names:

```python
def normal_gravity(L_b, h_b):
    """Computes the magnitude of the acceleration due to gravity.

    Somigliana normal gravity on the WGS-84 ellipsoid, with a free-air
    height correction. See the Earth & gravity theory page.

    Args:
        L_b: Geodetic latitude, scalar or shape (M,) [rad].
        h_b: Geodetic height above the ellipsoid, scalar or shape (M,) [m].

    Returns:
        Gravity magnitude g(L_b, h_b), same shape as the inputs [m/s²].

    Examples:
        >>> round(float(normal_gravity(0.0, 0.0)), 6)
        9.780325
    """
```

- Describe quantities in words first, then give the symbol: "velocity of
  the body relative to the Earth, resolved in NED".
- Give units in square brackets and array shapes explicitly.
- Keep examples short, deterministic and runnable; round floating-point
  output so the doctest is stable.
- Do not use the docs' MathJax macros in docstrings: `help()` shows
  docstrings as plain text.
