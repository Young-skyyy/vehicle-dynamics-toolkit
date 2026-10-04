# -*- coding: utf-8 -*-
"""Tests for FMI 2.0 ECU packaging metadata."""

from xml.etree import ElementTree

from vehicle_dynamics_toolkit.fmu import model_description


def test_model_description_is_fmi2_cosimulation():
    root = ElementTree.fromstring(model_description())

    assert root.attrib["fmiVersion"] == "2.0"
    assert root.find("CoSimulation").attrib["modelIdentifier"] == "ecu_fmu"
    variables = root.findall("./ModelVariables/ScalarVariable")
    assert [item.attrib["name"] for item in variables] == [
        "throttle_command", "brake_command", "speed", "rpm",
        "coolant_temp", "gear", "soc", "steering_command", "vx", "vy", "ax", "ay",
        "yaw_rate", "heading", "position_x", "position_y", "time", "engine_torque", "initial_speed_m_s",
    ]
    assert [item.attrib["index"] for item in root.findall("./ModelStructure/Outputs/Unknown")] == [
        "3", "4", "5", "6", "7", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18",
    ]


def test_integer_output_has_discrete_variability_and_calculated_initial():
    root = ElementTree.fromstring(model_description())
    gear = root.find("./ModelVariables/ScalarVariable[@name='gear']")
    assert gear.attrib["variability"] == "discrete"
    assert gear.attrib["initial"] == "calculated"
    assert "start" not in gear.find("Integer").attrib


def test_native_source_is_a_package_resource():
    from pathlib import Path
    from vehicle_dynamics_toolkit import fmu
    for name in ("ecu_fmu.cpp", "dynamics_core.hpp"):
        assert (Path(fmu.__file__).parent / "native" / name).is_file()


def test_fmu_physical_contract_has_normalized_controls_and_units():
    root = ElementTree.fromstring(model_description())
    assert root.attrib["guid"] == "{vehicle-dynamics-toolkit-vehicle-plant-v2}"
    for name in ("throttle_command", "brake_command"):
        real = root.find(f"./ModelVariables/ScalarVariable[@name='{name}']/Real")
        assert real.attrib["max"] == "1"
    assert root.find("./ModelVariables/ScalarVariable[@name='speed']/Real").attrib["unit"] == "km/h"
    assert root.find("./ModelVariables/ScalarVariable[@name='vx']/Real").attrib["unit"] == "m/s"
