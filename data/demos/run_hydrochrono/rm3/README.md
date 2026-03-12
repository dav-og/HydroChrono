# RM3 Reference Model 3

Two-body wave energy converter (WEC) consisting of a float and a
reaction plate connected by a prismatic joint with a linear PTO.

## Cases

- **decay** -- Free decay in still water (no PTO damping).
- **mooring** -- Irregular waves with MoorDyn mooring lines and PTO damping.

## Run

```
bin\run_hydrochrono.exe demos\rm3\decay\rm3_decay.setup.yaml --nogui
bin\run_hydrochrono.exe demos\rm3\mooring\rm3_mooring.setup.yaml --nogui
```

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
