# PriusPilot — Prius Gen 2 (NHW20) car controller
# Sends ocelot actuator CAN commands on bus 0.
# When no ocelot ECUs are detected, behaves as a no-op (Phase 1 mode).
from common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.toyota_gen2.ocelotcan import (
  create_steer_interceptor_command,
  create_gas_interceptor_command,
  create_gas_actuator_command,
  create_ibooster_cmd,
  create_relay_command,
)
from selfdrive.car.toyota_gen2.values import DBC, DetectedEcus, SteerLimitParams


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.last_steer = 0
    self.steer_rate_limited = False
    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0

  def update(self, c, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):
    can_sends = []
    enabled = c.enabled
    active = c.active

    # ── Steering ──────────────────────────────────────────────────────────────
    # Torque scales inversely with speed so the car doesn't oversteer at
    # highway speeds while still having enough authority at low speed.
    steer_lim = SteerLimitParams.STEER_MAX * max(0.0, 1.0 - (CS.out.vEgo / 90))
    new_steer = int(round(actuators.steer * steer_lim))
    apply_steer = apply_toyota_steer_torque_limits(
      new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams
    )
    self.steer_rate_limited = new_steer != apply_steer
    self.last_steer = apply_steer

    apply_steer_req = 1 if enabled else 0
    if not enabled:
      apply_steer = 0

    # ── Gas / brake ───────────────────────────────────────────────────────────
    apply_gas = 0.0
    apply_brake = 0.0
    if active:
      apply_gas = clip(actuators.accel, 0.0, 1.0)
      if actuators.accel < 0:
        apply_brake = clip(-actuators.accel, 0.0, 1.0)
    # Don't brake while driver is on the gas
    if CS.out.gas > 450:
      apply_brake = 0.0

    # ── 50 Hz messages ────────────────────────────────────────────────────────
    if frame % 2 == 0:
      if DetectedEcus["GasInterceptor"]:
        can_sends.append(
          create_gas_interceptor_command(self.packer, apply_gas, frame // 2)
        )
      if DetectedEcus["GasActuator"]:
        can_sends.append(
          create_gas_actuator_command(self.packer, enabled, apply_gas, frame // 2)
        )
      if DetectedEcus["RelayCore"]:
        relay_cmd = getattr(c, 'bodycontrol', None)
        relay_val = relay_cmd.relayCoreCMD if relay_cmd is not None else 0
        can_sends.append(
          create_relay_command(self.packer, enabled, relay_val, frame // 2)
        )

    # ── 100 Hz messages ───────────────────────────────────────────────────────
    if DetectedEcus["SteerInterceptor"]:
      can_sends.append(
        create_steer_interceptor_command(self.packer, apply_steer, apply_steer_req, frame)
      )
    if DetectedEcus["iBooster"]:
      can_sends.append(
        create_ibooster_cmd(self.packer, enabled, apply_brake, frame)
      )

    # Record applied values for logging / next-frame limiting
    self.accel = actuators.accel
    self.speed = CS.out.vEgo
    self.gas = apply_gas
    self.brake = apply_brake

    new_actuators = actuators.copy()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake

    return new_actuators, can_sends
