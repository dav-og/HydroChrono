# IEA Sphere

Floating hemisphere based on the IEA OES Task 10 benchmark.

## Cases

- **decay** -- Free decay in still water.
- **decay_ss** -- Free decay with state-space radiation approximation.
- **irregular_waves_ss** -- Irregular waves with state-space radiation.

## Run

```
bin\run_hydrochrono.exe demos\iea_sphere\decay\iea_sphere_decay.setup.yaml --nogui
bin\run_hydrochrono.exe demos\iea_sphere\decay_ss\iea_sphere_decay_ss.setup.yaml --nogui
bin\run_hydrochrono.exe demos\iea_sphere\irregular_waves_ss\iea_sphere_irregular_waves_ss.setup.yaml --nogui
```

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
