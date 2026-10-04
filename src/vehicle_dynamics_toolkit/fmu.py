# -*- coding: utf-8 -*-
"""FMI 2.0 Co-Simulation packaging of the reference vehicle's native plant."""

from __future__ import annotations

import platform
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring


_VARIABLES = (
    ("throttle_command", 1, "input", "Real", 0.0),
    ("brake_command", 2, "input", "Real", 0.0),
    ("speed", 3, "output", "Real", 0.0),
    ("rpm", 4, "output", "Real", 0.0),
    ("coolant_temp", 5, "output", "Real", 25.0),
    ("gear", 6, "output", "Integer", 0),
    ("soc", 7, "output", "Real", 80.0),
    ("steering_command", 8, "input", "Real", 0.0),
    ("vx", 9, "output", "Real", 0.0),
    ("vy", 10, "output", "Real", 0.0),
    ("ax", 11, "output", "Real", 0.0),
    ("ay", 12, "output", "Real", 0.0),
    ("yaw_rate", 13, "output", "Real", 0.0),
    ("heading", 14, "output", "Real", 0.0),
    ("position_x", 15, "output", "Real", 0.0),
    ("position_y", 16, "output", "Real", 0.0),
    ("time", 17, "output", "Real", 0.0),
    ("engine_torque", 18, "output", "Real", 0.0),
    ("initial_speed_m_s", 19, "parameter", "Real", 0.0),
)

FMU_GUID = "{vehicle-dynamics-toolkit-vehicle-plant-v2}"
STATE_REFS = {"t": 17, "vx": 9, "ax": 11, "vy": 10, "ay": 12, "yaw_rate": 13,
              "heading": 14, "position_x": 15, "position_y": 16,
              "engine_rpm": 4, "engine_torque": 18}


def model_description() -> bytes:
    """Return the FMI 2.0 modelDescription.xml for the ECU FMU."""
    root = Element("fmiModelDescription", {
        "fmiVersion": "2.0",
        "modelName": "VehicleDynamicsToolkit.ReferenceVehicle",
        "guid": FMU_GUID,
        "generationTool": "vehicle-dynamics-toolkit",
        "generationDateAndTime": "1970-01-01T00:00:00Z",
        "variableNamingConvention": "flat",
        "numberOfEventIndicators": "0",
    })
    SubElement(root, "CoSimulation", {
        "modelIdentifier": "ecu_fmu",
        "needsExecutionTool": "false",
        "canHandleVariableCommunicationStepSize": "true",
        "canInterpolateInputs": "false",
        "maxOutputDerivativeOrder": "0",
        "canGetAndSetFMUstate": "false",
        "canSerializeFMUstate": "false",
        "providesDirectionalDerivative": "false",
    })
    units = SubElement(root, "UnitDefinitions")
    unit_defs = {"1": {}, "s": {"s": "1"}, "m": {"m": "1"},
                 "m/s": {"m": "1", "s": "-1"}, "m/s2": {"m": "1", "s": "-2"},
                 "rad": {"rad": "1"}, "rad/s": {"rad": "1", "s": "-1"},
                 "km/h": {"m": "1", "s": "-1", "factor": str(1/3.6)},
                 "rpm": {"rad": "1", "s": "-1", "factor": "0.10471975511965977"},
                 "degC": {"K": "1", "offset": "273.15"},
                 "%": {"factor": ".01"}, "N.m": {"kg": "1", "m": "2", "s": "-2"}}
    for name, attributes in unit_defs.items():
        SubElement(SubElement(units, "Unit", {"name": name}), "BaseUnit", attributes)
    unit_for = {1: "1", 2: "1", 3: "km/h", 4: "rpm", 5: "degC", 7: "%", 8: "rad",
                9: "m/s", 10: "m/s", 11: "m/s2", 12: "m/s2", 13: "rad/s", 14: "rad",
                15: "m", 16: "m", 17: "s", 18: "N.m", 19: "m/s"}
    model_variables = SubElement(root, "ModelVariables")
    for name, ref, causality, kind, start in _VARIABLES:
        var = SubElement(model_variables, "ScalarVariable", {
            "name": name, "valueReference": str(ref),
            "causality": causality,
            "variability": "fixed" if causality == "parameter" else "discrete" if kind == "Integer" else "continuous",
            **({"initial": "calculated"} if causality == "output" else {"initial": "exact"} if causality == "parameter" else {}),
        })
        attributes = {} if causality == "output" else {"start": str(start)}
        if kind == "Real":
            attributes["unit"] = unit_for[ref]
        if ref in (1, 2):
            attributes.update(min="0", max="1")
        if ref == 8:
            attributes.update(min="-0.7", max="0.7")
        if ref == 19:
            attributes["min"] = "0"
        SubElement(var, kind, attributes)
    outputs = SubElement(root, "ModelStructure")
    unknowns = SubElement(outputs, "Outputs")
    initials = SubElement(outputs, "InitialUnknowns")
    for name, ref, causality, kind, start in _VARIABLES:
        if causality == "output":
            SubElement(unknowns, "Unknown", {"index": str(ref)})
            SubElement(initials, "Unknown", {"index": str(ref)})
    return tostring(root, encoding="utf-8", xml_declaration=True)


def build_fmu(output: str | Path, source_dir: str | Path | None = None,
              compiler: str | None = None) -> Path:
    """Build a platform FMU from the bundled C++17 plant wrapper.

    The wrapper is deliberately native: FMU importers can load it without a
    Python installation. A C++17 compiler is required for the target platform.
    """
    output_path = Path(output).resolve()
    source = Path(source_dir or Path(__file__).resolve().parent / "native" / "ecu_fmu.cpp")
    if not source.exists():
        raise FileNotFoundError(source)
    cc = compiler or shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if cc is None:
        raise RuntimeError("未找到 C++17 编译器；请安装 clang++/g++，或指定 compiler。")
    if platform.system() not in ("Windows", "Linux") or platform.machine().lower() not in ("amd64", "x86_64"):
        raise RuntimeError("FMU packaging currently supports Windows/Linux x86-64 only")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    prefix = ([cc, "c++"] if Path(cc).stem.lower() == "zig" else [cc]) + ["-std=c++17"]
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        binary = tmp_path / ("ecu_fmu.dll" if platform.system() == "Windows" else "ecu_fmu.so")
        if platform.system() == "Windows":
            command = prefix + ["-shared", "-O2", str(source), "-o", str(binary)]
        else:
            command = prefix + ["-shared", "-fPIC", "-O2", str(source), "-o", str(binary)]
        subprocess.run(command, check=True)
        with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("modelDescription.xml", model_description())
            archive.write(binary, "binaries/" + ("win64/ecu_fmu.dll" if platform.system() == "Windows" else "linux64/ecu_fmu.so"))
    return output_path
