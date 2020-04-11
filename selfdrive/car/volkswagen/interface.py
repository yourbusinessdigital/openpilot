from cereal import car
from common.realtime import DT_CTRL
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
from selfdrive.car.volkswagen.values import CAR, BUTTON_STATES, NWL, TRANS, GEAR
from common.params import Params
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    self.buttonStatesPrev = BUTTON_STATES.copy()

    self.last_enable_pressed = 0
    self.last_enable_sent = 0

    # Set up an alias to PT/CAM parser for ACC depending on its detected network location
    self.cp_acc = self.cp if CP.networkLocation == NWL.fwdCamera else self.cp_cam

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    if candidate == CAR.GENERICMQB:
      # Set common MQB parameters that will apply globally
      ret.carName = "volkswagen"
      ret.radarOffCan = True
      ret.safetyModel = car.CarParams.SafetyModel.volkswagen

      # Additional common MQB parameters that may be overridden per-vehicle
      ret.steerRateCost = 1.0
      ret.steerActuatorDelay = 0.1  # Hopefully all MQB racks are similar here
      ret.steerLimitTimer = 0.4

      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]

      # FIXME: Per-vehicle parameters need to be reintegrated.
      # Until that time, defaulting to VW Golf Mk7 as a baseline.

      ret.mass = 1500 + STD_CARGO_KG
      ret.wheelbase = 2.64
      ret.centerToFront = ret.wheelbase * 0.45
      ret.steerRatio = 15.6
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]
      tire_stiffness_factor = 1.0

    ret.enableCamera = True  # Stock camera detection doesn't apply to VW
    ret.transmissionType = car.CarParams.TransmissionType.automatic

    # Determine installed network location by finding the ACC_06 radar message
    if 0x122 in fingerprint[0]:
      ret.networkLocation = NWL.fwdCamera
    else:
      ret.networkLocation = NWL.gateway

    # Determine transmission type by CAN message(s) present on the bus
    if 0xAD in fingerprint[0]:
      # Getribe_11 message detected: traditional automatic or DSG gearbox
      ret.transmissionType = TRANS.automatic
    elif 0x187 in fingerprint[0]:
      # EV_Gearshift message detected: e-Golf or similar direct-drive electric
      ret.transmissionType = TRANS.direct
    else:
      # No trans message at all, must be a true stick-shift manual
      ret.transmissionType = TRANS.manual

    # FIXME: Stub/test long parameters borrowed from Toyota
    ret.enableCruise = False
    ret.openpilotLongitudinalControl = True
    ret.minEnableSpeed = -1.
    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [0.5]
    ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
    ret.longitudinalTuning.kiV = [0.54, 0.36]

    cloudlog.warning("Detected network location: %r", ret.networkLocation)
    cloudlog.warning("Detected transmission type: %r", ret.transmissionType)

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
    buttonEvents = []
    params = Params()

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_acc, self.CP.transmissionType)
    ret.canValid = self.cp.can_valid  # FIXME: Restore cp_cam valid check after proper LKAS camera detect
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # Update the EON metric configuration to match the car at first startup,
    # or if there's been a change.
    if self.CS.displayMetricUnits != self.displayMetricUnitsPrev:
      params.put("IsMetric", "1" if self.CS.displayMetricUnits else "0")

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GEAR.eco, GEAR.sport])

    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if self.CS.steeringFault:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))

    # Engagement and longitudinal control using stock ACC. Make sure OP is
    # disengaged if stock ACC is disengaged.
    # if not ret.cruiseState.enabled:
    #  events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
    # Attempt OP engagement only on rising edge of stock ACC engagement.
    # elif not self.cruise_enabled_prev:
    #  events.append(create_event('pcmEnable', [ET.ENABLE]))

    # NOTE: Test non-cruise long stuff borrowed from Honda
    cur_time = self.frame * DT_CTRL
    enable_pressed = False
    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and not b.pressed:
        self.last_enable_pressed = cur_time
        enable_pressed = True

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    if self.CP.enableCruise:
      # KEEP THIS EVENT LAST! send enable event if button is pressed and there are
      # NO_ENTRY events, so controlsd will display alerts. Also not send enable events
      # too close in time, so a no_entry will not be followed by another one.
      # TODO: button press should be the only thing that triggers enable
      if ((cur_time - self.last_enable_pressed) < 0.2 and
          (cur_time - self.last_enable_sent) > 0.2 and
          ret.cruiseState.enabled) or \
         (enable_pressed and get_events(events, [ET.NO_ENTRY])):
        events.append(create_event('buttonEnable', [ET.ENABLE]))
        self.last_enable_sent = cur_time
    elif enable_pressed:
      events.append(create_event('buttonEnable', [ET.ENABLE]))

    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    # update previous car states
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.audibleAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible,
                   c.hudControl.setSpeed)
    self.frame += 1
    return can_sends
