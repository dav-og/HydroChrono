---
layout: page
title: Getting Started
---

This guide walks you through downloading the HydroChrono package, exploring the included demo cases, and verifying your installation with the automated test suite.

## Prerequisites

- **Windows 10 or 11** (x64)
- **A Vulkan-capable GPU** for the real-time VSG visualization. Most modern discrete and integrated GPUs support Vulkan. If your GPU does not, you can still run simulations headless with `--nogui`.
- **Python 3.9+** (only required if you want to run the automated test suite)


## 1) Download and unzip

1. Go to the [Releases](https://github.com/NREL/HydroChrono/releases) page and download the latest `.zip` package (e.g. `HydroChrono-0.7.0-win64.zip`).
2. Extract the archive to a location of your choice.
3. Open **PowerShell** in the extracted folder. You should see the following top-level structure:

```
HydroChrono-0.7.0-win64/
  bin/
  data/
  demos/
  tests/
  include/
  lib/
  licenses/
```

## 2) What's in the package

| Folder | Contents |
|--------|----------|
| `bin/` | `run_hydrochrono.exe` and its runtime DLLs (Project Chrono, MoorDyn, HDF5, VSG, yaml-cpp, etc.) |
| `demos/` | Runnable YAML-based demo cases for several WEC models (see below) |
| `data/chrono/` | Chrono runtime assets (skybox textures for the 3D viewer) |
| `tests/` | Python scripts for the automated regression test suite |
| `include/`, `lib/` | C++ headers and link libraries for developers building against HydroChrono |
| `licenses/` | Third-party license files |

The only executable you need for running simulations is `bin\run_hydrochrono.exe`. Everything else supports it.


## 3) Demo models

The `demos/` folder contains ready-to-run cases for four wave energy converter (WEC) models. Each model has a shared `assets/` directory (geometry meshes and BEMIO hydrodynamic data) and one or more case sub-directories.

### IEA Sphere (`demos\iea_sphere\`)

A 5 m radius hemisphere, developed as a simplified WEC benchmark under the IEA OES Task 10 project. Single body constrained to heave.

| Case | Description | Command |
|------|-------------|---------|
| `decay` | Free decay in still water | `.\bin\run_hydrochrono.exe .\demos\iea_sphere\decay` |
| `decay_ss` | Free decay with state-space radiation approximation | `.\bin\run_hydrochrono.exe .\demos\iea_sphere\decay_ss` |
| `irregular_waves_ss` | Irregular JONSWAP waves with state-space radiation | `.\bin\run_hydrochrono.exe .\demos\iea_sphere\irregular_waves_ss` |

### RM3 (`demos\rm3\`)

The Reference Model 3 (RM3) two-body point absorber, consisting of a float and a heave plate connected by a PTO spring-damper. A widely used WEC-Sim benchmark geometry.

| Case | Description | Command |
|------|-------------|---------|
| `decay` | Free decay in still water (two bodies, heave only) | `.\bin\run_hydrochrono.exe .\demos\rm3\decay` |
| `mooring` | Irregular waves with MoorDyn catenary mooring (3 lines, 6 segments) | `.\bin\run_hydrochrono.exe .\demos\rm3\mooring` |

<p align="center">
  <img src="{{ site.baseurl }}/assets/img/rm3_mooring_gui.png" alt="RM3 mooring simulation showing the two-body point absorber with catenary mooring lines colored by tension and animated wave surface" width="80%" />
</p>

### OSWEC (`demos\oswec\`)

An Oscillating Surge Wave Energy Converter with a pitching flap and a fixed base, from the WEC-Sim tutorial cases.

| Case | Description | Command |
|------|-------------|---------|
| `decay` | Pitch free decay in still water | `.\bin\run_hydrochrono.exe .\demos\oswec\decay` |

### F3OF (`demos\f3of\`)

A Floating Three-Flap Oscillating Flap device (platform with three independently pitching flaps).

| Case | Description | Command |
|------|-------------|---------|
| `decay_dt1` | Surge decay (flaps locked) | `.\bin\run_hydrochrono.exe .\demos\f3of\decay_dt1` |
| `decay_dt2` | Pitch decay (flaps locked) | `.\bin\run_hydrochrono.exe .\demos\f3of\decay_dt2` |
| `decay_dt3` | Flap pitch decay (base locked) | `.\bin\run_hydrochrono.exe .\demos\f3of\decay_dt3` |


## 4) Understanding the YAML inputs

Every simulation is defined by four plain-text YAML files inside the case directory:

```
demos/rm3/decay/
  rm3_decay.setup.yaml        # entry point
  rm3_decay.model.yaml        # multibody system
  rm3_decay.simulation.yaml   # solver and runtime settings
  rm3_decay.hydro.yaml        # hydrodynamics and waves
```

| File | Purpose |
|------|---------|
| `*.setup.yaml` | References the other three files and sets the output directory. This is the file (or its parent directory) you pass to `run_hydrochrono.exe`. |
| `*.model.yaml` | Defines the Chrono multibody system: bodies (mass, inertia, position, visualization mesh), joints, constraints, and actuators. |
| `*.simulation.yaml` | Controls how the simulation runs: time step, duration, integrator type, solver settings, and whether the GUI is enabled. |
| `*.hydro.yaml` | Specifies hydrodynamic inputs: BEMIO `.h5` file paths, body-to-hydro mapping, wave type and parameters, excitation settings, and (optionally) MoorDyn mooring configuration. |

For example, the RM3 decay setup file is simply:

```yaml
model_file: rm3_decay.model.yaml
simulation_file: rm3_decay.simulation.yaml
hydro_file: rm3_decay.hydro.yaml

output_directory: outputs
```

And its hydro file defines a still-water condition:

```yaml
hydrodynamics:
  bodies:
    - name: body1
      h5_file: ../assets/hydroData/rm3.h5
    - name: body2
      h5_file: ../assets/hydroData/rm3.h5

  waves:
    type: still
```

For full YAML templates and guidance on creating your own models, see [Create a New Model]({{ site.baseurl }}/new_model/).


## 5) Running a demo (step-by-step)

From the package root directory, run:

```powershell
.\bin\run_hydrochrono.exe .\demos\rm3\decay
```

You will see:

1. **Banner** with version and status information.
2. **Input summary** confirming which YAML files were loaded, the number of bodies and constraints, simulation duration, and time step.
3. **Hydrodynamic data summary** showing the HDF5 file, body count, frequency count, and which force components are active.
4. **Wave model summary** (type, height, period).
5. **A 3D GUI window** opens showing the assembled system with animated free-surface rendering. You can orbit, pan, and zoom with the mouse.
6. **Progress updates** as the simulation advances through time.
7. **Completion summary** with wall time and the path to the exported HDF5 file.

<p align="center">
  <img src="{{ site.baseurl }}/assets/img/cli_example.png" alt="CLI output example" width="75%" />
</p>

<p align="center">
  <img src="{{ site.baseurl }}/assets/img/gui_example.png" alt="GUI visualization example" width="40%" />
</p>

### Useful CLI flags

| Flag | Effect |
|------|--------|
| `--nogui` | Run headless (no 3D window). Useful for batch runs and the test suite. |
| `--log` | Write a detailed, timestamped log file to `<case>/logs/`. |
| `--quiet` | Suppress all console output. Pair with `--log` to capture output silently. |
| `--debug` | Increase CLI verbosity to include debug-level messages. |
| `--trace` | Maximum verbosity (developer diagnostics). |

Example:

```powershell
.\bin\run_hydrochrono.exe .\demos\rm3\decay --nogui --log
```

See the full logging guide: [Logging and CLI Output]({{ site.baseurl }}/logging/).


## 6) Outputs

Each simulation writes its results to an `outputs/` subdirectory inside the case folder (e.g. `demos\rm3\decay\outputs\`).

The primary output is a portable **HDF5** file (e.g. `results.still.h5`) containing time-series data organized as:

| Dataset path | Contents |
|--------------|----------|
| `/results/time/time` | Time vector |
| `/results/model/bodies/<name>/position` | Body XYZ position (columns 0-2) |
| `/results/model/bodies/<name>/orientation_xyz` | Body roll, pitch, yaw (columns 0-2) |
| `/results/model/tsdas/<name>/...` | Translational spring-damper force, extension, speed |
| `/results/model/rsdas/<name>/...` | Rotational spring-damper torque, angle, angular velocity |

You can inspect HDF5 files with:

- [HDFView](https://www.hdfgroup.org/downloads/hdfview/) (free GUI browser)
- VS Code with an HDF5 extension
- Python: `import h5py; f = h5py.File("results.still.h5", "r")`


## 7) Running the automated test suite

The `tests/` folder contains a Python-based regression test suite that runs the demo cases headless, compares the output against expected baselines, and generates comparison plots. This is the best way to verify that your package is working correctly.

### Option A: one-command setup (recommended)

This creates a local Python virtual environment and installs all dependencies automatically:

```powershell
cd tests
.\RUN-TESTS.ps1
```

The script will:

1. Detect `bin\run_hydrochrono.exe` in the package.
2. Prompt you to create a `.venv` and install required Python packages (`numpy`, `h5py`, `PyYAML`, `matplotlib`) from PyPI.
3. Run every demo case headless (`--nogui`).
4. Compare simulation output against reference baselines.
5. Print a **PASS/FAIL** summary to the console.
6. Write comparison plots to `demos\<model>\<case>\outputs\plots\`.

### Option B: use your existing Python environment

If you already have a Python environment with `numpy`, `h5py`, `PyYAML`, and `matplotlib`:

```powershell
cd tests
python .\run_tests.py --all --exe ..\bin\run_hydrochrono.exe --demos-dir ..\demos
```

### Run a single test

To run just one case (e.g. the IEA sphere decay):

```powershell
cd tests
python .\run_tests.py --sphere-decay --exe ..\bin\run_hydrochrono.exe --demos-dir ..\demos
```

### What to expect

A successful run produces output like:

```
PASS | N=1000 | RMSrel=0.001234 | tol=0.020000
Plot saved: ...\demos\iea_sphere\decay\outputs\plots\iea_sphere_decay_test___heave_comparison.png
```

If any test reports `FAIL`, the comparison plot will help diagnose where the simulation diverges from the reference.

<p align="center">
  <img src="{{ site.baseurl }}/assets/img/oswec_decay_test_comparison.png" alt="Example comparison plot from the OSWEC decay test" width="66%" />
</p>


## 8) Next steps

- **Create your own model** &mdash; follow the minimal-inputs guide: [Create a New Model]({{ site.baseurl }}/new_model/)
- **Verification details** &mdash; see the detailed verification results for each model: [Verification]({{ site.baseurl }}/verification/)
- **Report issues or request features** &mdash; open an issue on [GitHub](https://github.com/NREL/HydroChrono/issues)

<p align="center">
  <img src="{{ site.baseurl }}/assets/img/wave_animation2.gif" alt="Wave animation" width="80%" />
</p>
