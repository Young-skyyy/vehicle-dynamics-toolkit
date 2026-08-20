# -*- coding: utf-8 -*-
"""FMI 2.0 Co-Simulation packaging helpers for the virtual ECU."""

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
    ("rpm", 4, "output", "Real", 800.0),
    ("coolant_temp", 5, "output", "Real", 25.0),
    ("gear", 6, "output", "Integer", 0),
    ("soc", 7, "output", "Real", 80.0),
)


def model_description() -> bytes:
    """Return the FMI 2.0 modelDescription.xml for the ECU FMU."""
    root = Element("fmiModelDescription", {
        "fmiVersion": "2.0",
        "modelName": "VehicleDynamicsToolkit.ECU",
        "guid": "{vehicle-dynamics-toolkit-ecu-fmu-v1}",
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
    })
    model_variables = SubElement(root, "ModelVariables")
    for name, ref, causality, kind, start in _VARIABLES:
        var = SubElement(model_variables, "ScalarVariable", {
            "name": name, "valueReference": str(ref),
            "causality": causality, "variability": "continuous",
        })
        SubElement(var, kind, {"start": str(start)})
    outputs = SubElement(root, "ModelStructure")
    unknowns = SubElement(outputs, "Outputs")
    for ref in (3, 4, 5, 6, 7):
        SubElement(unknowns, "Unknown", {"index": str(ref)})
    return tostring(root, encoding="utf-8", xml_declaration=True)


def build_fmu(output: str | Path, source_dir: str | Path | None = None,
              compiler: str | None = None) -> Path:
    """Build a platform FMU from the bundled FMI C wrapper.

    The wrapper is deliberately native: FMU importers can load it without a
    Python installation. A C compiler is required for the target platform.
    """
    output_path = Path(output).resolve()
    source = Path(source_dir or Path(__file__).resolve().parents[2] / "fmu" / "ecu_fmu.c")
    if not source.exists():
        raise FileNotFoundError(source)
    cc = compiler or shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if cc is None:
        raise RuntimeError("未找到 C 编译器；请安装 clang/gcc，或通过 compiler 参数指定。")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        binary = tmp_path / ("ecu_fmu.dll" if platform.system() == "Windows" else "ecu_fmu.so")
        if platform.system() == "Windows":
            command = [cc, "-shared", "-O2", str(source), "-o", str(binary)]
        else:
            command = [cc, "-shared", "-fPIC", "-O2", str(source), "-o", str(binary)]
        subprocess.run(command, check=True)
        with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("modelDescription.xml", model_description())
            archive.write(binary, "binaries/" + ("win64/ecu_fmu.dll" if platform.system() == "Windows" else "linux64/ecu_fmu.so"))
    return output_path
