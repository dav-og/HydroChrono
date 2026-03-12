# HydroChrono Demo Cases

This folder contains runnable YAML-based demo cases for HydroChrono.
Each model has its own directory with shared assets (geometry, hydro data)
and one or more case configurations.

## Running a demo

From the package root:

```
bin\run_hydrochrono.exe demos\rm3\decay\rm3_decay.setup.yaml --nogui
```

Output files are written to an `outputs/` subdirectory next to the setup YAML.

## Available demos

| Model | Case | Description | Setup YAML |
|-------|------|-------------|------------|
| rm3 | decay | RM3 two-body WEC, free decay in still water | `rm3/decay/rm3_decay.setup.yaml` |
| rm3 | mooring | RM3 WEC with MoorDyn mooring lines, irregular waves | `rm3/mooring/rm3_mooring.setup.yaml` |
| iea_sphere | decay | IEA sphere, free decay in still water | `iea_sphere/decay/iea_sphere_decay.setup.yaml` |
| iea_sphere | decay_ss | IEA sphere, free decay with state-space radiation | `iea_sphere/decay_ss/iea_sphere_decay_ss.setup.yaml` |
| iea_sphere | irregular_waves_ss | IEA sphere, irregular waves with state-space | `iea_sphere/irregular_waves_ss/iea_sphere_irregular_waves_ss.setup.yaml` |
| oswec | decay | OSWEC flap device, free decay in still water | `oswec/decay/oswec_decay.setup.yaml` |
| f3of | decay_dt1 | F3OF platform, DT1 surge decay (flaps locked) | `f3of/decay_dt1/f3of_decay_dt1.setup.yaml` |
| f3of | decay_dt2 | F3OF platform, DT2 pitch decay (flaps locked) | `f3of/decay_dt2/f3of_decay_dt2.setup.yaml` |
| f3of | decay_dt3 | F3OF platform, DT3 flap pitch decay (base locked) | `f3of/decay_dt3/f3of_decay_dt3.setup.yaml` |

## Directory layout

Each model follows this structure:

```
<model>/
  assets/           Shared geometry (.obj) and hydro data (.h5)
  <case>/           One directory per demo case
    <name>.setup.yaml
    <name>.model.yaml
    <name>.simulation.yaml
    <name>.hydro.yaml
    expected/       Reference baseline output (if available)
  signal_adapter.py Test signal extraction (used by test runner)
```

## Running the automated test suite

The `tests/` folder contains Python scripts that run these same demos
headless and compare the output against expected baselines. See
`tests/RUN-TESTS.ps1` for the entry point.
