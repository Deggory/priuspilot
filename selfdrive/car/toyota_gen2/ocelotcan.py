#!/usr/bin/env python3
# PriusPilot — ocelot CAN message builders
# Adapted from RetroPilot retropilot/ocelotcan.py
# All messages are sent on CAN bus 0 (same bus as the Prius).


def create_steer_interceptor_command(packer, torque, enable, idx):
  """Steering torque command for the ocelot steering interceptor (0x300)."""
  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }
  if enable:
    # Interceptor uses a differential encoding: mid-point 1510 ± torque
    values["TORQUE_COMMAND1"] = 1510 + torque
    values["TORQUE_COMMAND2"] = 1510 - torque
  return packer.make_can_msg("INTERCEPTOR_STEERING_COMMAND", 0, values)


def create_gas_interceptor_command(packer, gas_amount, idx):
  """Gas-pedal interceptor command (0x200).  gas_amount is [0.0, 1.0]."""
  enable = gas_amount > 0.001
  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }
  if enable:
    # Scale factors match comma gas-pedal interceptor firmware defaults.
    # Adjust GAS_COMMAND / GAS_COMMAND2 offsets to match your car's pedal range.
    values["GAS_COMMAND"] = (gas_amount * 2400) + 850.
    values["GAS_COMMAND2"] = (gas_amount * 2000) + 480.
  return packer.make_can_msg("PEDAL_GAS_COMMAND", 0, values)


def create_gas_actuator_command(packer, enabled, gas_amount, idx):
  """Direct throttle actuator command (0x400).  gas_amount is [0.0, 1.0]."""
  values = {
    "ENABLE": enabled,
    "COUNTER": idx & 0xF,
  }
  if enabled:
    values["THROTTLE_REQ"] = gas_amount * 1500
  return packer.make_can_msg("ACTUATOR_GAS_COMMAND", 0, values)


def create_ibooster_cmd(packer, enabled, brake, raw_cnt):
  """iBooster brake command (0x20E).  brake is [0.0, 1.0]."""
  values = {
    "BRAKE_POSITION_COMMAND": brake * 7,
    "BRAKE_RELATIVE_COMMAND": 0,
    "BRAKE_MODE": enabled * 2.,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("IBOOSTER_BRAKE_COMMAND", 0, values)


def create_relay_command(packer, enabled, relay, idx):
  """RelayCore lighting/relay command (0x600)."""
  values = {
    "RELAY_COMMAND": relay,
    "ENABLE": enabled,
    "COUNTER": idx & 0xF,
  }
  return packer.make_can_msg("RELAY_CORE_COMMAND", 0, values)
