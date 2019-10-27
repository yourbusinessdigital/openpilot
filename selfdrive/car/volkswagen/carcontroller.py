from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC
from selfdrive.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AUDIBLE_WARNINGS = [AudibleAlert.chimeWarning1, AudibleAlert.chimeWarning2]
EMERGENCY_WARNINGS = [AudibleAlert.chimeWarningRepeat]

class CarControllerParams:
  HCA_STEP_ACTIVE = 2            # HCA_01 message frequency 50Hz when applying torque
  LDW_STEP = 10                  # LDW_02 message frequency 10Hz
  GRA_ACC_STEP = 3               # GRA_ACC_01 message frequency 33Hz

  # Observe documented MQB limits: 3.00nm max, rate of change 5.00nm/sec
  STEER_MAX = 300                # Max heading control assist torque 3.00nm
  STEER_DELTA_UP = 10            # Max HCA reached in 0.600s (STEER_MAX / (50Hz * 0.600))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.600s (STEER_MAX / (50Hz * 0.600))
  STEER_DRIVER_ALLOWANCE = 100
  STEER_DRIVER_MULTIPLIER = 4    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

class CarController():
  def __init__(self, canbus, car_fingerprint):
    self.counter = 0
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint
    self.acc_vbp_type = None
    self.acc_vbp_endframe = None
    self.same_torque_cnt = 0
    self.non_zero_cnt = 0

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.packer_gw = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, visual_alert, audible_alert, leftLaneVisible, rightLaneVisible):
    """ Controls thread """

    P = CarControllerParams

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    #
    # Prepare HCA_01 steering torque message
    #
    if (frame % P.HCA_STEP_ACTIVE) == 0:

      # Don't send steering commands unless we've successfully enabled vehicle
      # ACC (prevent controls mismatch) and we're moving (prevent EPS fault).
      if enabled and not CS.standstill and not CS.steeringFault:
        lkas_enabled = True
        apply_steer = int(round(actuators.steer * P.STEER_MAX))

        apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steeringTorque, P)

        # Ugly hack to reset EPS hardcoded 180 second limit for HCA intervention.
        # Deal with this by disengaging HCA anytime we have a zero-crossing.
        if apply_steer == 0:
          lkas_enabled = False

      else:
        # Disable heading control assist
        lkas_enabled = False
        apply_steer = 0

      # torque can't be the same for 6 consecutive seconds or EPS will fault. Apply a unit adjustment
      if apply_steer != 0 and self.apply_steer_last == apply_steer:
        self.same_torque_cnt += 1
      else:
        self.same_torque_cnt = 0

      if self.same_torque_cnt >= 550:  # 5.5s
        apply_steer -= (1, -1)[apply_steer < 0]
        self.same_torque_cnt = 0

      # torque can't be non-zero for more than 3 consecutive minutes
      if apply_steer != 0:
        self.non_zero_cnt += 1
      else:
        self.non_zero_cnt = 0

      if self.non_zero_cnt >=  170 * 100:  # 170s ~ 3 minutes
        # TODO: do something (alert driver?, one step with lkas_enabled = False?)
        self.non_zero_cnt = 0

      self.apply_steer_last = apply_steer
      idx = (frame / P.HCA_STEP_ACTIVE) % 16
      can_sends.append(volkswagencan.create_mqb_steering_control(self.packer_gw, canbus.gateway, apply_steer, idx, lkas_enabled))

    #
    # Prepare LDW_02 HUD message with lane lines and confidence levels
    #
    if (frame % P.LDW_STEP) == 0:
      if enabled and not CS.standstill:
        lkas_enabled_hud = True
      else:
        lkas_enabled_hud = False

      if visual_alert == VisualAlert.steerRequired:
        if audible_alert in EMERGENCY_WARNINGS:
          hud_alert = 6 # "Emergency Assist: Please Take Over Steering", with beep
        elif audible_alert in AUDIBLE_WARNINGS:
          hud_alert = 7 # "Lane Assist: Please Take Over Steering", with beep
        else:
          hud_alert = 8 # "Lane Assist: Please Take Over Steering", silent
      else:
        hud_alert = 0

      can_sends.append(volkswagencan.create_mqb_hud_control(self.packer_gw, canbus.gateway, lkas_enabled_hud, hud_alert, leftLaneVisible, rightLaneVisible))

    #
    # Prepare GRA_ACC_01 message with ACC cruise control buttons
    #
    if (frame % P.GRA_ACC_STEP) == 0:

      idx = (frame / P.GRA_ACC_STEP) % 16

      # Copy the ACC control button state from the car, and transparently pass
      # most of it onto the ACC radar, with a few exceptions for conformance
      # to comma's more-strict safety model, and to support resume-from-stop.
      # NOTE: Later, could potentially add virtual button presses to tweak the
      # ACC speed setpoint for "longitudinal lite" control by OP.
      buttonStatesToSend = CS.buttonStates.copy()

      if enabled:
        if CS.standstill and frame % (P.GRA_ACC_STEP * 33) == 0:
          # Blip the Resume button ~1x/second if we're engaged at standstill
          # FIXME: This is a naive implementation, improve with visiond or radar input
          # A subset of MQBs like to "creep" too aggressively with this implementation.
          self.acc_vbp_type = "resumeCruise"
          self.acc_vbp_endframe = frame + 20

      else:
        # Cancel ACC if it's already active, for the cases where OP disengages
        # before platform ACC does.
        if CS.accEnabled:
          self.acc_vbp_type = "cancel"
          self.acc_vbp_endframe = frame + 20

      # Insert any virtual button press generated by openpilot controls, to
      # conform with safety requirements or to update the ACC speed setpoint.
      if self.acc_vbp_type is not None:
        if frame < self.acc_vbp_endframe:
          buttonStatesToSend[self.acc_vbp_type] = True
        else:
          self.acc_vbp_type = None
          self.acc_vbp_endframe = None

      can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_gw, canbus.extended, buttonStatesToSend, CS, idx))

    return can_sends
