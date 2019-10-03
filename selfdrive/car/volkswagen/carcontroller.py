from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import mqbcan
from selfdrive.car.volkswagen.values import DBC
from selfdrive.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AUDIBLE_WARNINGS = [AudibleAlert.chimeWarning1, AudibleAlert.chimeWarning2]
EMERGENCY_WARNINGS = [AudibleAlert.chimeWarningRepeat]

class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.HCA_STEP_ACTIVE = 2            # HCA_01 message frequency 50Hz when applying torque (100 / 2)
    self.HCA_STEP_INACTIVE = 10         # HCA_01 message frequency 10Hz when not applying torque (100 / 10)
    self.LDW_STEP = 10                  # LDW_02 message frequency 10Hz (100 / 10)

    self.STEER_MAX = 300                # Max heading control assist torque 3.00nm
    self.STEER_DELTA_UP = 16            # Max HCA reached in 0.375s (STEER_MAX / (50Hz * 0.375))
    self.STEER_DELTA_DOWN = 32          # Min HCA reached in 0.375s (STEER_MAX / (50Hz * 0.375))
    self.STEER_DRIVER_ALLOWANCE = 100
    self.STEER_DRIVER_MULTIPLIER = 4  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1  # from dbc


class CarController(object):
  def __init__(self, canbus, car_fingerprint):
    self.counter = 0
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer_gw = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, visual_alert, audible_alert, leftLaneVisible, rightLaneVisible):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    #
    # Prepare HCA_01 steering torque message
    #
    if (frame % P.HCA_STEP_ACTIVE) == 0:

      if enabled and not CS.standstill:
        lkas_enabled = 1
        apply_steer = int(round(actuators.steer * P.STEER_MAX))

        #apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_driver, P)
        self.apply_steer_last = apply_steer

        # Ugly hack to reset EPS hardcoded 180 second limit for HCA intervention.
        # Deal with this by disengaging HCA anytime we have a zero-crossing.
        if apply_steer == 0:
          lkas_enabled = 0

      else:
        # Disable heading control assist
        lkas_enabled = 0
        apply_steer = 0
        self.apply_steer_last = 0

      idx = (frame / P.HCA_STEP_ACTIVE) % 16
      can_sends.append(mqbcan.create_steering_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, apply_steer, idx, lkas_enabled))

    #
    # Prepare LDW_02 HUD message with lane lines and confidence levels
    #
    if (frame % P.LDW_STEP) == 0:
      if enabled and not CS.standstill:
        lkas_enabled = 1
      else:
        lkas_enabled = 0

      if visual_alert == VisualAlert.steerRequired:
        if audible_alert in EMERGENCY_WARNINGS:
          hud_alert = 6 # "Emergency Assist: Please Take Over Steering", with beep
        elif audible_alert in AUDIBLE_WARNINGS:
          hud_alert = 7 # "Lane Assist: Please Take Over Steering", with beep
        else:
          hud_alert = 8 # "Lane Assist: Please Take Over Steering", silent
      else:
        hud_alert = 0

      can_sends.append(mqbcan.create_hud_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, lkas_enabled, hud_alert, leftLaneVisible, rightLaneVisible))

    return can_sends

