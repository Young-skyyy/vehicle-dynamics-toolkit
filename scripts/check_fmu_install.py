"""Build and load the native FMU, optionally requiring an installed wheel.

This is an ABI/packaging smoke test, not full FMI conformance certification.
"""
from __future__ import annotations
import argparse
import ctypes
import _ctypes
import os
from pathlib import Path
import tempfile
import zipfile
import vehicle_dynamics_toolkit.fmu as fmu


def check(compiler: str | None = None, require_installed: bool = False) -> None:
    source_root = Path(__file__).resolve().parents[1] / "src"
    module = Path(fmu.__file__).resolve()
    if require_installed and source_root in module.parents:
        raise RuntimeError("Test imported source checkout instead of installed wheel")
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = fmu.build_fmu(root / "ecu.fmu", compiler=compiler)
        with zipfile.ZipFile(archive) as packed:
            packed.extractall(root)
        binaries = list((root / "binaries").rglob("ecu_fmu.*"))
        assert len(binaries) == 1
        lib = ctypes.CDLL(str(binaries[0]))
        component = None
        try:
            lib.fmi2GetVersion.restype = ctypes.c_char_p
            lib.fmi2GetTypesPlatform.restype = ctypes.c_char_p
            assert lib.fmi2GetVersion() == b"2.0"
            assert lib.fmi2GetTypesPlatform() == b"default"
            lib.fmi2Instantiate.restype = ctypes.c_void_p
            lib.fmi2Instantiate.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
                                            ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
            component = lib.fmi2Instantiate(b"smoke", 1, b"{vehicle-dynamics-toolkit-ecu-fmu-v1}",
                                            None, None, 0, 0)
            assert component
            lib.fmi2FreeInstance.argtypes = [ctypes.c_void_p]
            lib.fmi2SetupExperiment.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double,
                                                ctypes.c_double, ctypes.c_int, ctypes.c_double]
            assert lib.fmi2SetupExperiment(component, 0, 0, 0, 0, 0) == 0
            for name in ("fmi2EnterInitializationMode", "fmi2ExitInitializationMode"):
                function = getattr(lib, name)
                function.argtypes = [ctypes.c_void_p]
                assert function(component) == 0
            refs = (ctypes.c_uint * 1)(1)
            command = (ctypes.c_double * 1)(50.0)
            real_signature = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
                              ctypes.POINTER(ctypes.c_double)]
            lib.fmi2SetReal.argtypes = real_signature
            lib.fmi2GetReal.argtypes = real_signature
            assert lib.fmi2SetReal(component, refs, 1, command) == 0
            lib.fmi2DoStep.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_int]
            assert lib.fmi2DoStep(component, 0, .1, 1) == 0
            speed_ref = (ctypes.c_uint * 1)(3)
            speed = (ctypes.c_double * 1)()
            assert lib.fmi2GetReal(component, speed_ref, 1, speed) == 0
            assert abs(speed[0] - .3) < 1e-12
            assert lib.fmi2DoStep(component, .1, -1, 1) == 3
            lib.fmi2Reset.argtypes = [ctypes.c_void_p]
            assert lib.fmi2Reset(component) == 0
            assert lib.fmi2GetReal(component, speed_ref, 1, speed) == 0
            assert speed[0] == 0
            lib.fmi2Terminate.argtypes = [ctypes.c_void_p]
            assert lib.fmi2Terminate(component) == 0
        finally:
            if component:
                lib.fmi2FreeInstance(component)
            if os.name == "nt":
                _ctypes.FreeLibrary(lib._handle)
            else:
                _ctypes.dlclose(lib._handle)
        print("PASS: installed source resource, native compilation, load, step and reset")
        print("Imported:", module)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler")
    parser.add_argument("--require-installed", action="store_true")
    args = parser.parse_args()
    check(args.compiler, args.require_installed)
