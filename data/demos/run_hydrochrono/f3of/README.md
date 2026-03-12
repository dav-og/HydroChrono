# F3OF

Floating three-body oscillating flap device (platform with fore/aft flaps).

## Cases

- **decay_dt1** -- DT1: surge decay with flaps locked to base.
- **decay_dt2** -- DT2: pitch decay with flaps locked to base.
- **decay_dt3** -- DT3: flap pitch decay with base locked to ground.

## Run

```
bin\run_hydrochrono.exe demos\f3of\decay_dt1\f3of_decay_dt1.setup.yaml --nogui
bin\run_hydrochrono.exe demos\f3of\decay_dt2\f3of_decay_dt2.setup.yaml --nogui
bin\run_hydrochrono.exe demos\f3of\decay_dt3\f3of_decay_dt3.setup.yaml --nogui
```

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
