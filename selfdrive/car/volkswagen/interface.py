#!/usr/bin/env python
from cereal import car, log
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, get_events, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.volkswagen.values import DBC, CAR
from selfdrive.car.volkswagen.carstate import CarState, get_gateway_can_parser, get_extended_can_parser
from common.params import Params
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from common.vin import VIN_UNKNOWN

class CanBus(object):
  def __init__(self):
    self.gateway = 0
    self.extended = 1

class CarInterface(object):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.acc_active_prev = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
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

    if candidate == CAR.GENERICMQB:
      # Check to make sure we received the VIN; we should have this for all MQBs
      # XXX temp removed
      # assert(ret.carVin != VIN_UNKNOWN), "Fingerprinted as Generic MQB but did not detect VIN"

      # Set common MQB parameters
      ret.carName = "volkswagen"
      ret.safetyModel = car.CarParams.SafetyModel.volkswagen

      ret.enableCruise = True # Stock ACC still controls acceleration and braking
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.steerLimitAlert = True # Enable UI alert when steering torque is maxed out

      # Use the VIN to look up specific make and model details
      chassiscode = vin[6:8]
      # XXX temp hack
      if(chassiscode == "00"):
        chassiscode = "AU"

      if chassiscode == "3G":
        # B8 Passat, RoW only (North America Passat is PQ/NMS)
        # FIXME: Mass is average between the sedan and wagon, and the spread is pretty high, may need more detection here somehow.
        ret.mass = 1554
        ret.wheelbase = 2.79
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.375], [0.1]]
      elif chassiscode == "3H":
        # Mk1 Volkswagen Arteon 2018-present
        ret.mass = 1704
        ret.wheelbase = 2.84
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.375], [0.1]]
      elif chassiscode == "5E" or chassiscode == "NE":
        # Mk3 Skoda Octavia, 2013-present
        ret.mass = 1360
        ret.wheelbase = 2.69
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.375], [0.1]]
      elif chassiscode == "8V" or chassiscode == "FF":
        # Mk3 Audi A3, S3, and RS3
        # FIXME: Wheelbase will vary between some versions (hatch vs sportback) so we may need more detection here somehow
        ret.mass = 1910
        ret.wheelbase = 2.61
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]
      elif chassiscode == "AU":
        # Mk7 and Mk7.5 Volkswagen Golf, Alltrack, Sportwagen, GTI, Golf R, and e-Golf, 2013-2020 depending on market
        # Mass will vary a bit, but wheelbase is identical for all variants
        ret.mass = 1372
        ret.wheelbase = 2.64
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00008
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.1]]
      elif chassiscode == "BU":
        # Mk7 Volkswagen Jetta (Sagitar in Chinese market), 2019-present
        ret.mass = 1347
        ret.wheelbase = 2.69
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.375], [0.1]]
      elif chassiscode == "CA":
        # Mk1 Volkswagen Atlas (Teramont in some markets), 2018-present
        ret.mass = 2042
        ret.wheelbase = 2.97
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]
      elif chassiscode == "FV":
        # Mk3 Audi TT/TTS/TTRS, 2014-present
        ret.mass = 1328
        ret.wheelbase = 2.50
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]
      elif chassiscode == "GA":
        # Mk1 Audi Q2 2017-present
        ret.mass = 1205
        ret.wheelbase = 2.60
        # TODO: Untested vehicle, placeholder tuning values
        ret.steerRatio = 15
        ret.steerRateCost = 0.5
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.25]]

      # Additional common MQB parameters
      ret.mass += STD_CARGO_KG
      ret.centerToFront = ret.wheelbase * 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]  # m/s
      ret.steerActuatorDelay = 0.05
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]

    # TODO: gate this on detection
    ret.enableCamera = True
    ret.steerRatioRear = 0.
    tire_stiffness_factor = 1. # Placeholder in lieu of vehicle-specific tuning

    # No support for OP longitudinal control on Volkswagen at this time.
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

    ret.canValid = self.gw_cp.can_valid and self.ex_cp.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.CS.yaw_rate * CV.DEG_TO_RAD
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

    # Doors open, seatbelt unfastened
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    # Gas, brakes and shifting
    ret.gas = self.CS.pedal_gas / 100
    ret.gasPressed = self.CS.pedal_gas > 0
    ret.brake = self.CS.user_brake / 250 # FIXME: approximated, verify
    ret.brakePressed = bool(self.CS.brake_pressed)
    ret.brakeLights = bool(self.CS.brake_lights)
    ret.gearShifter = self.CS.gear_shifter

    # Update the EON metric configuration to match the car at first startup,
    # or if there's been a change.
    if self.CS.is_metric != self.CS.is_metric_prev:
      params.put("IsMetric", "1" if self.CS.is_metric == 1 else "0")

    # Update dynamic vehicle mass calculated by the drivetrain coordinator.
    # NOTE: At this time, OP probably won't make use of a mass value updated
    # after startup.
    ret.mass = self.CS.mass

    buttonEvents = []

    # Process button press or release events from ACC steering wheel or
    # control stalk buttons. We don't have enough room in capnp to capture
    # all seven buttons, even with the alt buttons, so the timegap button
    # is not seen as an event at this time.
    if self.CS.gra_acc_buttons != self.CS.gra_acc_buttons_prev:
      if self.CS.gra_acc_buttons["main"] != self.CS.gra_acc_buttons_prev["main"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton3'
        be.pressed = bool(self.CS.gra_acc_buttons["main"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["set"] != self.CS.gra_acc_buttons_prev["set"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton1'
        be.pressed = bool(self.CS.gra_acc_buttons["set"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["resume"] != self.CS.gra_acc_buttons_prev["resume"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton2'
        be.pressed = bool(self.CS.gra_acc_buttons["resume"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["cancel"] != self.CS.gra_acc_buttons_prev["cancel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'cancel'
        be.pressed = bool(self.CS.gra_acc_buttons["cancel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["accel"] != self.CS.gra_acc_buttons_prev["accel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'accelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["accel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["decel"] != self.CS.gra_acc_buttons_prev["decel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'decelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["decel"])
        buttonEvents.append(be)

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
    #if self.CS.acc_active and not self.acc_active_prev:
    #  events.append(create_event('pcmEnable', [ET.ENABLE]))
    #if not self.CS.acc_active:
    #  events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
    #self.acc_active_prev = self.CS.acc_active

    # Vehicle operation safety checks and events
    if not ret.gearShifter == 'drive':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev) or \
            (ret.brakePressed and (not self.brake_pressed_prev or not ret.standstill)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
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

    # Process engagement events based on the ACC set and resume buttons.
    # The ACC increase and decrease buttons should not trigger engagement.
    for b in buttonEvents:
      if b.type in ["altButton1", "altButton2"] and b.pressed:
        events.append(create_event('buttonEnable', [ET.ENABLE]))
      if b.type in ["cancel"] and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

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
