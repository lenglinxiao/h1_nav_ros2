import argparse
import io
import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from scipy.spatial import cKDTree


def _read_header(stream) -> Tuple[Dict[str, List[str]], str]:
    header: Dict[str, List[str]] = {}
    while True:
        raw_line = stream.readline()
        if not raw_line:
            raise ValueError("PCD header ended before DATA")
        line = raw_line.decode("ascii").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == "DATA":
            return header, parts[1].lower()


def _numpy_type(type_name: str, size: int):
    mapping = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    try:
        return mapping[(type_name.upper(), size)]
    except KeyError as exc:
        raise ValueError(f"Unsupported PCD field type {type_name}{size}") from exc


def load_pcd_xyz(path: Path) -> np.ndarray:
    path = Path(path)
    with path.open("rb") as stream:
        header, data_kind = _read_header(stream)
        payload = stream.read()

    fields = header.get("FIELDS") or header.get("FIELD")
    if not fields or not all(axis in fields for axis in ("x", "y", "z")):
        raise ValueError(f"PCD file has no x/y/z fields: {path}")
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"Incomplete PCD field metadata: {path}")
    point_count = int((header.get("POINTS") or ["0"])[0])

    if data_kind == "ascii":
        values = np.loadtxt(io.BytesIO(payload), dtype=np.float64)
        values = np.atleast_2d(values)
        offsets = {}
        column = 0
        for field, count in zip(fields, counts):
            offsets[field] = column
            column += count
        points = values[:, [offsets["x"], offsets["y"], offsets["z"]]]
    elif data_kind == "binary":
        dtype_fields = []
        for field, size, type_name, count in zip(fields, sizes, types, counts):
            field_type = _numpy_type(type_name, size)
            dtype_fields.append((field, field_type) if count == 1 else (field, field_type, (count,)))
        records = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=point_count or -1)
        points = np.column_stack((records["x"], records["y"], records["z"])).astype(
            np.float64,
            copy=False,
        )
    else:
        raise ValueError(f"Unsupported PCD DATA mode '{data_kind}'; use ascii or binary")

    points = points[np.all(np.isfinite(points), axis=1)]
    if not len(points):
        raise ValueError(f"PCD file contains no finite XYZ points: {path}")
    return points


def deterministic_sample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def compare_point_clouds(
    reference: np.ndarray,
    actual: np.ndarray,
    overlap_distance_m: float = 0.20,
) -> Dict[str, float]:
    if not len(reference) or not len(actual):
        raise ValueError("Both point clouds must contain points")
    reference_to_actual = cKDTree(actual).query(reference, k=1, workers=-1)[0]
    actual_to_reference = cKDTree(reference).query(actual, k=1, workers=-1)[0]
    all_distances = np.concatenate((reference_to_actual, actual_to_reference))
    return {
        "reference_point_count": int(len(reference)),
        "actual_point_count": int(len(actual)),
        "reference_to_actual_rmse_m": float(np.sqrt(np.mean(reference_to_actual ** 2))),
        "actual_to_reference_rmse_m": float(np.sqrt(np.mean(actual_to_reference ** 2))),
        "symmetric_rmse_m": float(np.sqrt(np.mean(all_distances ** 2))),
        "symmetric_p95_m": float(np.percentile(all_distances, 95.0)),
        "symmetric_max_m": float(np.max(all_distances)),
        "reference_overlap_ratio": float(np.mean(reference_to_actual <= overlap_distance_m)),
        "actual_overlap_ratio": float(np.mean(actual_to_reference <= overlap_distance_m)),
        "symmetric_overlap_ratio": float(np.mean(all_distances <= overlap_distance_m)),
        "overlap_distance_m": float(overlap_distance_m),
    }


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(
        description="Compare two already aligned PCD maps with symmetric nearest-neighbor error."
    )
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--actual", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument("--overlap-distance-m", type=float, default=0.20)
    parser.add_argument("--max-p95-m", type=float, default=-1.0)
    parser.add_argument("--min-overlap-ratio", type=float, default=-1.0)
    return parser.parse_args(args)


def main(args=None):
    options = parse_arguments(args)
    reference = deterministic_sample(load_pcd_xyz(options.reference), options.max_points)
    actual = deterministic_sample(load_pcd_xyz(options.actual), options.max_points)
    summary = compare_point_clouds(reference, actual, options.overlap_distance_m)
    failures = []
    if options.max_p95_m >= 0.0 and summary["symmetric_p95_m"] > options.max_p95_m:
        failures.append(
            f"symmetric p95 {summary['symmetric_p95_m']:.6f} > {options.max_p95_m:.6f}"
        )
    if (
        options.min_overlap_ratio >= 0.0
        and summary["symmetric_overlap_ratio"] < options.min_overlap_ratio
    ):
        failures.append(
            "symmetric overlap "
            f"{summary['symmetric_overlap_ratio']:.6f} < {options.min_overlap_ratio:.6f}"
        )
    summary["alignment"] = "none; both maps must already use the same frame"
    summary["passed"] = not failures
    summary["failures"] = failures
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
