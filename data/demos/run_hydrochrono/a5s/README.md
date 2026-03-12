# A5S -- Attenuator, 5 Segments

Five cylindrical tube segments (4 m diameter, 36 m long, 180 m total)
connected end-to-end by four universal joints (pitch + yaw). Each joint
is fitted with 4 hydraulic-ram TSDAs in a cross pattern (top, bottom,
port, starboard) for a total of 16 dampers.

## Dimensions

| Parameter | Value |
|-----------|-------|
| Segment diameter | 4.0 m |
| Segment length | 36.0 m |
| Number of segments | 5 |
| Total length | 180.0 m |
| Draft | 1.8 m |
| Mass per segment | 438,293 kg |
| CoG offset (below waterline axis) | -0.2 m |
| TSDA damping coefficient | 500,000 N-s/m |
| TSDA free length | 2.0 m |
| TSDA moment arm | 1.5 m |

## Cases

- **regular_waves** -- Regular waves (H = 2 m, T = 10 s, 120 s duration).
- **irregular_waves** -- Irregular JONSWAP waves (Hs = 3 m, Tp = 10 s, 600 s duration).

## Run

```
bin\run_hydrochrono.exe demos\a5s\regular_waves
bin\run_hydrochrono.exe demos\a5s\irregular_waves
```

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.

Mesh generation and BEM analysis scripts:

- `generate_meshes.py` -- creates Nemoh (`.nemoh`) and OBJ (`.obj`) meshes from parametric segment profiles (no dependencies beyond the Python standard library).
- `run_bem.py` -- runs Capytaine BEM, computes impulse response functions, and writes the BEMIO HDF5 file (`a5s.h5`). Requires a Python environment with `capytaine`, `numpy`, `h5py`, and `matplotlib`.
