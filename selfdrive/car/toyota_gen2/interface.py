# PriusPilot — Prius Gen 2 (NHW20) car interface
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.toyota_gen2.tunes import LatTunes, LongTunes, set_lat_tune, set_long_tune
from selfdrive.car.toyota_gen2.values import CAR, DetectedEcus

EventName = car.CarEvent.EventName

# Ocelot hardware CAN addresses (status / sensor messages used for ECU detection)
_ECU_FINGERPRINT = {
  "GasInterceptor":  0x201,  # PEDAL_GAS_SENSOR
  "GasActuator":     0x401,  # ACTUATOR_GAS_SENSOR
  "SteerInterceptor": 0x301, # INTERCEPTOR_STEERING_SENSOR
  "SteerActuator":   0x12F,  # ACTUATOR_STEERING_STATUS
  "SteerActuatorSSC": 0x22F, # STEERING_STATUS_SSC
  "iBooster":        0x20F,  # IBOOSTER_BRAKE_STATUS
  "BrakeActuator":   0x10F,  # ACTUATOR_BRAKE_STATUS
  "RelayCore":       0x601,  # RELAY_CORE_STATUS
}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "toyota_gen2"

    # ── Ocelot ECU detection ───────────────────────────────────────────────────
    # Update the module-level DetectedEcus dict so carstate / carcontroller can
    # branch on which hardware is actually present.
    for ecu, addr in _ECU_FINGERPRINT.items():
      if addr in fingerprint[0]:
        DetectedEcus[ecu] = True

    # ── Safety model ──────────────────────────────────────────────────────────
    # noOutput blocks ALL CAN TX — safe until ocelot panda safety is merged.
    # Switch to car.CarParams.SafetyModel.allOutput (or a custom ocelot model)
    # when the Pyboard firmware has a matching safety kernel.
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput)]

    # ── Vehicle parameters (Prius Gen 2 NHW20) ───────────────────────────────
    ret.wheelbase = 2.70           # metres
    ret.steerRatio = 17.1          # slightly higher than Gen 3
    ret.steerActuatorDelay = 0.3   # seconds
    ret.steerLimitTimer = 1.0
    ret.minEnableSpeed = -1.0      # hybrid: engage from standstill

    tire_stiffness_factor = 0.6
    ret.mass = 1380. + STD_CARGO_KG  # curb weight ~1380 kg

    ret.centerToFront = ret.wheelbase * 0.44
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(
      ret.mass, ret.wheelbase, ret.centerToFront,
      tire_stiffness_factor=tire_stiffness_factor
    )

    # ── Feature flags ─────────────────────────────────────────────────────────
    ret.enableBsm = False
    ret.enableDsu = False
    ret.enableGasInterceptor = DetectedEcus["GasInterceptor"]
    ret.stoppingControl = DetectedEcus["iBooster"] or DetectedEcus["BrakeActuator"]

    # Enable openpilot longitudinal when any gas actuator is present
    ret.openpilotLongitudinalControl = (
      DetectedEcus["GasInterceptor"] or DetectedEcus["GasActuator"]
    )
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # ── Lateral tuning ────────────────────────────────────────────────────────
    set_lat_tune(ret.lateralTuning, LatTunes.PID_A)

    # ── Longitudinal tuning ───────────────────────────────────────────────────
    if DetectedEcus["GasInterceptor"]:
      set_long_tune(ret.longitudinalTuning, LongTunes.PEDAL)
    else:
      set_long_tune(ret.longitudinalTuning, LongTunes.ACTUATOR)

    return ret

  def update(self, c, can_strings):
    # parse CAN — cp is ocelot parser, cp_cam is native Prius parser
    self.cp.update_strings(can_strings)
    if self.cp_cam is not None:
      self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    # canValid: prefer ocelot parser when hardware is present (it is the control
    # interface), otherwise fall back to the native Prius parser.
    ocelot_active = any(DetectedEcus.values())
    if ocelot_active:
      ret.canValid = self.cp.can_valid
    elif self.cp_cam is not None:
      ret.canValid = self.cp_cam.can_valid
    else:
      ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    events = self.create_common_events(ret)

    if self.frame == 0:
      events.add(EventName.startupNoControl)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c, self.CS, self.frame,
                         c.actuators,
                         False,   # pcm_cancel_cmd
                         hud_control.visualAlert,
                         False, False, False, False, False)
    self.frame += 1
    return ret
