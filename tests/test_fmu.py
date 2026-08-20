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
        "coolant_temp", "gear", "soc",
    ]
    assert [item.attrib["index"] for item in root.findall("./ModelStructure/Outputs/Unknown")] == [
        "3", "4", "5", "6", "7",
    ]
