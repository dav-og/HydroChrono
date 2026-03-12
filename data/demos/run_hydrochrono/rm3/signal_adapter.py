"""Signal adapter for RM3 demo cases (decay and mooring)."""
from pathlib import Path
import numpy as np
import h5py


def _read_time(f):
	for key in ["/results/time/time", "/results/time", "/time"]:
		if key in f:
			return np.asarray(f[key][:], dtype=float).reshape(-1)
	raise KeyError("time vector not found")


def _body_dof(f, name_candidates, col, label):
	"""Read a single DOF column from body position data."""
	for name in name_candidates:
		for key in [
			f"/results/model/bodies/{name}/position",
			f"/results/bodies/{name}/position",
		]:
			if key in f:
				arr = np.asarray(f[key][:])
				if arr.ndim == 2 and arr.shape[1] > col:
					return arr[:, col]
				if arr.ndim == 1 and col == 0:
					return arr
	raise KeyError(f"RM3: DOF col {col} ({label}) not found for: {name_candidates}")


def _is_mooring_case(h5_path: Path) -> bool:
	return "mooring" in h5_path.parts


def select_signal(h5_path: Path):
	with h5py.File(h5_path, "r") as f:
		t = _read_time(f)
		if _is_mooring_case(h5_path):
			return t, _body_dof(f, ["body1", "float"], 0, "Surge"), "Surge (m)"
		return t, _body_dof(f, ["body2", "plate", "body1", "float"], 2, "Heave"), "Heave (m)"


def select_signals(h5_path: Path):
	with h5py.File(h5_path, "r") as f:
		t = _read_time(f)

		if _is_mooring_case(h5_path):
			return {
				"float_surge": (t, _body_dof(f, ["body1", "float"], 0, "Surge"), "Surge (m)"),
				"plate_surge": (t, _body_dof(f, ["body2", "plate"], 0, "Surge"), "Surge (m)"),
			}

		return {
			"float": (t, _body_dof(f, ["body1", "float"], 2, "Heave"), "Heave (m)"),
			"plate": (t, _body_dof(f, ["body2", "plate"], 2, "Heave"), "Heave (m)"),
		}


