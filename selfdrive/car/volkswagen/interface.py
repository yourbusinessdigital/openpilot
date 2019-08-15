#!/usr/bin/env python
from cereal import car, log
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.volkswagen.values import DBC, CAR
from selfdrive.car.volkswagen.carstate import CarState, get_gateway_can_parser, get_extended_can_parser
from common.params import Params
from common.vin import vin_model_year
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness


class CanBus(object):
  def __init__(self):
    self.gateway = 0
    self.extended = 1

class CarInterface(object):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.acc_active_prev = 0
    self.CC = None

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)
    self.gw_cp = get_gateway_can_parser(CP, canbus)
    self.ex_cp = get_extended_can_parser(CP, canbus)

    # sending if read only is False
    if CarController is not None:
      self.CC = CarController(canbus, CP.carFingerprint)


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint, vin="", is_panda_black=False):
    ret = car.CarParams.new_message()

    ret.carFingerprint = candidate
    ret.isPandaBlack = is_panda_black
    ret.carVin = vin
    year, make, model = None, None, None

    if candidate == CAR.GENERICMQB:
      # We should know the VIN; process that to get specific make and model details
      chassiscode = vin[6:8]

      # Set per-vehicle parameters
      if chassiscode == "CA":
        # Mk1 Volkswagen Atlas, 2018-present
        # FIXME: Placeholder tuning values, needs testing
        make = "Volkswagen"
        model = "Atlas"
        ret.mass = 2042
        ret.wheelbase = 2.97
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]
      elif chassiscode == "AU":
        # Mk7 and Mk7.5 Volkswagen Golf, ~2013-2020 depending on market
        make = "Volkswagen"
        model = "Golf"
        ret.mass = 1372
        ret.wheelbase = 2.64
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]
      elif chassiscode == "5E":
        # Mk3 Skoda Octavia, 2013-present
        # FIXME: Placeholder tuning values, needs testing
        ret.mass = 1360
        ret.wheelbase = 2.69
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.375], [0.1]]

      # Set common MQB parameters
      ret.carName = vin_model_year(vin) + " " + make + " " + model
      ret.safetyModel = car.CarParams.SafetyModel.volkswagen

      ret.enableCruise = True # Stock ACC still controls acceleration and braking
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.steerLimitAlert = True # Enable UI alert when steering torque is maxed out

      ret.mass += STD_CARGO_KG
      ret.centerToFront = ret.wheelbase * 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]  # m/s
      ret.steerActuatorDelay = 0.05
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]

    # TODO: gate this on detection
    ret.enableCamera = True
    ret.steerRatioRear = 0.

    # FIXME: from gm
    # Testing removal of unused longitudinal stuffs
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]

    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]

    ret.stoppingControl = True
    ret.startAccel = 0.8

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    canMonoTimes = []

    params = Params()

    self.gw_cp.update_strings(int(sec_since_boot() * 1e9), can_strings)
    self.ex_cp.update_strings(int(sec_since_boot() * 1e9), can_strings)

    self.CS.update(self.gw_cp, self.ex_cp)

    # create message
    ret = car.CarState.new_message()

    #ret.canValid = self.gw_cp.can_valid and self.ex_cp.can_valid
    ret.canValid = True

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = self.CS.steer_torque_driver

    # ACC cruise state
    ret.cruiseState.available = self.CS.acc_enabled
    ret.cruiseState.enabled = self.CS.acc_active
    ret.cruiseState.speed = self.CS.cruise_set_speed

    # Blinker updates
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    # doors open, seatbelt unfastened
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    # Gas, brakes and shifting
    ret.gas = self.CS.pedal_gas / 100
    ret.gasPressed = self.CS.pedal_gas > 0
    ret.brake = self.CS.user_brake / 250 # FIXME: approximated, verify
    ret.brakePressed = bool(self.CS.brake_pressed)
    ret.brakeLights = bool(self.CS.brake_lights)
    ret.gearShifter = self.CS.gear_shifter

    # Update the EON metric configuration to match the car at first startup, or if there's been a change.
    if self.CS.is_metric != self.CS.is_metric_prev:
      params.put("IsMetric", "1" if self.CS.is_metric == 1 else "0")

    buttonEvents = []

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = bool(self.CS.left_blinker_on)
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = bool(self.CS.right_blinker_on)
      buttonEvents.append(be)

    events = []

    # Observe the car's ACC engage and disengage behavior and set OP engagement
    # to match.
    # FIXME: Eventually move to intercepting GRA_ACC_01 and generating button events instead
    if self.CS.acc_active and not self.acc_active_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    if not self.CS.acc_active:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
    self.acc_active_prev = self.CS.acc_active

    # Vehicle operation safety checks and events
    if not ret.gearShifter == 'drive':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.park_brake:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))

    # Vehicle health safety checks and events
    if self.CS.acc_error:
      # ACC radar is alive but reporting a health or visibility problem.
      events.append(create_event('radarFault', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.steer_error:
      # Steering rack is not configured for Heading Control Assist, or there
      # has been a timeout or other error in its reception of HCA messages.
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    ret.events = events
    ret.canMonoTimes = canMonoTimes

    # cast to reader so it can't be modified
    return ret.as_reader()

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.audibleAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible)
    self.frame += 1
    return can_sends
