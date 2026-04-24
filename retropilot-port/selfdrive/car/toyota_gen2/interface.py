#!/usr/bin/env python3
# Prius Gen 2 (NHW20, 2004-2009) car interface for retropilot (openpilot)
# Phase 1: read-only, no actuator control
from cereal import car
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.toyota_gen2.values import CAR

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "toyota_gen2"

    # Phase 1: NO_OUTPUT safety blocks all CAN transmissions from the comma device
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput)]

    # ---- Prius Gen 2 (NHW20) vehicle parameters ----
    ret.wheelbase = 2.70           # m — same as Gen 3 Prius
    ret.steerRatio = 17.1          # slightly higher than Gen 3 (15.74)
    ret.steerActuatorDelay = 0.3   # s — same delay as Gen 3 Prius
    ret.steerLimitTimer = 1.0

    tire_stiffness_factor = 0.6
    ret.mass = 1380. + STD_CARGO_KG   # curb weight ~1380 kg (NHW20)

    ret.centerToFront = ret.wheelbase * 0.44

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(
      ret.mass, ret.wheelbase, ret.centerToFront,
      tire_stiffness_factor=tire_stiffness_factor,
    )

    # ---- Feature flags ----
    ret.enableBsm = False                       # no blind spot monitor
    ret.enableDsu = False                       # no DSU on Gen 2
    ret.enableGasInterceptor = False

    # Gen 2 has full hybrid stop-and-go capability
    ret.minEnableSpeed = -1.0                   # no minimum engage speed
    ret.pcmCruise = True                        # cruise state from the car's PCM
    ret.stoppingControl = False                 # Phase 1: no longitudinal control

    # Phase 1: openpilot does NOT take longitudinal control
    ret.openpilotLongitudinalControl = False

    # ---- Lateral tuning (unused in Phase 1) ----
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kpV = [0.]
    ret.lateralTuning.pid.kiV = [0.]
    ret.lateralTuning.pid.kf = 0.

    # ---- Longitudinal tuning (unused in Phase 1) ----
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [0.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.]

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    # cp_cam is None for Gen 2 (no camera CAN bus)
    if self.cp_cam is not None:
      self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = False

    events = self.create_common_events(ret)

    # Phase 1: report startupNoControl once on startup
    if self.frame == 0:
      events.add(EventName.startupNoControl)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c.enabled, c.active, self.CS, self.frame,
                         c.actuators, c.cruiseControl.cancel,
                         hud_control.visualAlert, hud_control.leftLaneVisible,
                         hud_control.rightLaneVisible, hud_control.leadVisible,
                         hud_control.leftLaneDepart, hud_control.rightLaneDepart)
    self.frame += 1
    return ret
