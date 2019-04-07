from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.vw.carstate import CarState, get_gateway_can_parser, get_extended_can_parser
from selfdrive.car.vw import vwcan
from selfdrive.car.vw.values import CAR, DBC
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
    self.STEER_DELTA_INC = 16           # Max HCA reached in 0.375s (STEER_MAX / (50Hz * 0.375))
    self.STEER_DELTA_DEC = 16           # Min HCA reached in 0.375s (STEER_MAX / (50Hz * 0.375))


class CarController(object):
  def __init__(self, canbus, car_fingerprint):
    self.start_time = sec_since_boot()
    self.counter = 0
    self.apply_steer_prev = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer_gw = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, sendcan, enabled, CS, frame, actuators, visual_alert, audible_alert, leftLaneVisible, rightLaneVisible):
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
        # TODO: Verify our lkas_enabled DBC bit is correct, VCDS thinks it may not be
        lkas_enabled = 1
        plan_requested_torque = int(round(actuators.steer * P.STEER_MAX))

        # If the driver is actively providing steering input, prevent the planned torque request
        # from exceeding one-third of maximum. We adjust the plan prior to smoothing so we get
        # smooth ramp-down of HCA torque if we were above this before the driver intervened.
        if(CS.steer_override):
          plan_requested_torque = clip(plan_requested_torque, -P.STEER_MAX / 3, P.STEER_MAX / 3)

        # Apply increase and decrease rate limits for HCA torque in accordance with safety model.
        if self.apply_steer_prev >= 0:
          # Previously steering LEFT or STRAIGHT, normal calculations
          hca_steer_min = max(self.apply_steer_prev - P.STEER_DELTA_DEC, 0 - P.STEER_DELTA_INC)
          hca_steer_max = min(self.apply_steer_prev + P.STEER_DELTA_INC, P.STEER_MAX)
        else:
          # Previously steering RIGHT, inverted calculations
          hca_steer_min = max(self.apply_steer_prev - P.STEER_DELTA_INC, -P.STEER_MAX)
          hca_steer_max = min(self.apply_steer_prev + P.STEER_DELTA_DEC, 0 + P.STEER_DELTA_INC)

        apply_steer = clip(plan_requested_torque, hca_steer_min, hca_steer_max)
        self.apply_steer_prev = apply_steer

        # FIXME: Ugly hack to reset EPS hardcoded 180 second limit for HCA intervention.
        # Deal with this by disengaging HCA anytime we have a zero-crossing. Need to refactor
        # the up/down rate code above to enforce a zero-crossing on all changes of direction
        # just for additional safety margin.
        if apply_steer == 0:
          lkas_enabled = 0

      else:
        # Disable heading control assist
        lkas_enabled = 0
        apply_steer = 0
        self.apply_steer_prev = 0

      idx = (frame / P.HCA_STEP_ACTIVE) % 16
      can_sends.append(vwcan.create_steering_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, apply_steer, idx, lkas_enabled))

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

      can_sends.append(vwcan.create_hud_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, lkas_enabled, hud_alert, leftLaneVisible, rightLaneVisible))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
