#!/usr/bin/env python3
"""Independent contract smoke test for the NavSide TRT extension skeleton.

This test does not build or run TensorRT. It validates the fixed input and
output contract that the C++ extension is expected to enforce.
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass

try:  # pragma: no cover - optional local dependency
    import numpy as np
except Exception:  # pragma: no cover - system python may not ship numpy
    np = None


@dataclass(frozen=True)
class TensorContract:
    name: str
    shape: tuple[int, ...]


@dataclass(frozen=True)
class FakeFlags:
    c_contiguous: bool


@dataclass(frozen=True)
class FakeArray:
    dtype: object
    shape: tuple[int, ...]
    flags: FakeFlags


ENCODER_INPUT = TensorContract("depth", (1, 1, 40, 64))
ENCODER_OUTPUT = TensorContract("depth_features", (1, 64, 5, 8))
POLICY_INPUTS = (
    TensorContract("obs", (1, 2576)),
    TensorContract("h_in", (1, 1, 512)),
    TensorContract("c_in", (1, 1, 512)),
)
POLICY_OUTPUTS = (
    TensorContract("actions", (1, 3)),
    TensorContract("h_out", (1, 1, 512)),
    TensorContract("c_out", (1, 1, 512)),
)


def require_contract(arr: object, contract: TensorContract) -> None:
    dtype = getattr(arr, "dtype", None)
    shape = tuple(getattr(arr, "shape", ()))
    flags = getattr(arr, "flags", None)
    c_contiguous = bool(getattr(flags, "c_contiguous", False))

    if np is not None:
        if np.dtype(dtype) != np.float32:
            raise AssertionError(f"{contract.name} must be float32, got {dtype}")
    else:
        if dtype != "float32":
            raise AssertionError(f"{contract.name} must be float32, got {dtype}")

    if not c_contiguous:
        raise AssertionError(f"{contract.name} must be C-contiguous")
    if shape != contract.shape:
        raise AssertionError(f"{contract.name} must have shape {contract.shape}, got {shape}")


def main() -> None:
    if np is not None:
        depth = np.zeros(ENCODER_INPUT.shape, dtype=np.float32)
        depth_features = np.zeros(ENCODER_OUTPUT.shape, dtype=np.float32)
        obs, h_in, c_in = (
            np.zeros(contract.shape, dtype=np.float32) for contract in POLICY_INPUTS
        )
        actions, h_out, c_out = (
            np.zeros(contract.shape, dtype=np.float32) for contract in POLICY_OUTPUTS
        )
        bad = np.zeros((1, 1, 40, 64), dtype=np.float64)
    else:
        depth = FakeArray("float32", ENCODER_INPUT.shape, FakeFlags(True))
        depth_features = FakeArray("float32", ENCODER_OUTPUT.shape, FakeFlags(True))
        obs = FakeArray("float32", POLICY_INPUTS[0].shape, FakeFlags(True))
        h_in = FakeArray("float32", POLICY_INPUTS[1].shape, FakeFlags(True))
        c_in = FakeArray("float32", POLICY_INPUTS[2].shape, FakeFlags(True))
        actions = FakeArray("float32", POLICY_OUTPUTS[0].shape, FakeFlags(True))
        h_out = FakeArray("float32", POLICY_OUTPUTS[1].shape, FakeFlags(True))
        c_out = FakeArray("float32", POLICY_OUTPUTS[2].shape, FakeFlags(True))
        bad = FakeArray("float64", ENCODER_INPUT.shape, FakeFlags(True))

    for arr, contract in (
        (depth, ENCODER_INPUT),
        (depth_features, ENCODER_OUTPUT),
        (obs, POLICY_INPUTS[0]),
        (h_in, POLICY_INPUTS[1]),
        (c_in, POLICY_INPUTS[2]),
        (actions, POLICY_OUTPUTS[0]),
        (h_out, POLICY_OUTPUTS[1]),
        (c_out, POLICY_OUTPUTS[2]),
    ):
        require_contract(arr, contract)

    try:
        require_contract(bad, ENCODER_INPUT)
    except AssertionError:
        pass
    else:
        raise AssertionError("dtype guard did not reject float64 input")

    ext_spec = importlib.util.find_spec("navside_trt")
    if ext_spec is None:
        print("navside_trt import not built yet; contract smoke test passed")
    else:
        print(f"navside_trt module found at {ext_spec.origin}")
        print("contract smoke test passed")


if __name__ == "__main__":
    main()
