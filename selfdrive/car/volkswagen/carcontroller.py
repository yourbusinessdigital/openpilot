from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC, MQB_LDW_MESSAGES
from selfdrive.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AUDIBLE_WARNINGS = [AudibleAlert.chimeWarning1, AudibleAlert.chimeWarning2]
EMERGENCY_WARNINGS = [AudibleAlert.chimeWarningRepeat]

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
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
    self.gra_acc_msgctr_last = None
    self.gra_acc_resume_last = None
    self.same_torque_cnt = 0
    self.hca_enabled_cnt = 0
    self.hca_msg_counter = 0

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
    if frame % P.HCA_STEP == 0:
      # We send HCA_01 at full rate of 50Hz. The factory camera sends at 50Hz
      # while steering and 1Hz when not. Rate-switching creates some confusion
      # in Cabana and doesn't seem to add value, so we send at 50Hz all the
      # time. The rack does accept HCA at 100Hz if we want to control at finer
      # resolution in the future.

      # FAULT CONDITION: HCA may not be enabled at standstill. Also stop
      # commanding HCA if there's a fault, so the steering rack recovers.
      if not enabled or (CS.standstill or CS.steeringFault):
        # Disable Heading Control Assist
        hca_enabled = False
        apply_steer = 0

      else:
        # FAULT CONDITION: Requested HCA torque may not exceeds 3.0nm. This is
        # inherently handled by scaling to STEER_MAX. The rack doesn't seem
        # to care about up/down rate, but we have some evidence it may do its
        # own rate limiting, and matching OP helps for accurate tuning.
        apply_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steeringTorque, P)

        # FAULT CONDITION: HCA may not be enabled for > 360 seconds. Sending
        # the HCA disabled flag for one frame is sufficient to work around it.
        if apply_steer == 0:
          # We can do this very frequently by disabling HCA when apply_steer
          # happens to be exactly zero, which naturally happens quite often
          # during direction changes. This could be expanded with a small
          # dead-zone to capture more zero crossings, but not seeing a need.
          hca_enabled = False
          self.hca_enabled_cnt = 0
        else:
          self.hca_enabled_cnt += 1
          if self.hca_enabled_cnt >=  118 * 100:  # 118s ~ about 2 minutes
            # The Kansas Crosswind Problem: what happens if we need to steer
            # left for six minutes? We have to disable HCA for a frame anyway.
            # Do so three times within that six minutes to make certain the
            # timer is reset even if a couple messages are lost.
            hca_enabled = False
            self.hca_enabled_cnt = 0
          else:
            hca_enabled = True
            # FAULT CONDITION: HCA torque may not be static for > 6 seconds.
            # This is intended to detect the sending camera being stuck or
            # frozen. OP can trip this on a curve when it wants => STEER_MAX.
            # Avoid this by reducing torque 0.01nm for one frame if it's been
            # level for more than 1.9 seconds. That makes sure we've sent at
            # least three trimmed messages within the 6 second span, resetting
            # the rack timer even if a couple messages are lost.
            if self.apply_steer_last == apply_steer:
              self.same_torque_cnt += 1
              if self.same_torque_cnt > 190:  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.same_torque_cnt = 0
            else:
              self.same_torque_cnt = 0

      self.apply_steer_last = apply_steer

      self.hca_msg_counter = (self.hca_msg_counter + 1) % 16
      can_sends.append(volkswagencan.create_mqb_steering_control(self.packer_gw, canbus.gateway, apply_steer,
                                                                 self.hca_msg_counter, hca_enabled))

    #
    # Prepare LDW_02 HUD message with lane lines and confidence levels
    #
    if frame % P.LDW_STEP == 0:
      if enabled and not CS.standstill:
        hca_enabled = True
      else:
        hca_enabled = False

      if visual_alert == VisualAlert.steerRequired:
        if audible_alert in EMERGENCY_WARNINGS:
          hud_alert = MQB_LDW_MESSAGES["emergencyAssistAudible"]
        elif audible_alert in AUDIBLE_WARNINGS:
          hud_alert = MQB_LDW_MESSAGES["laneAssistAudible"]
        else:
          hud_alert = MQB_LDW_MESSAGES["laneAssistSilent"]
      else:
        hud_alert = MQB_LDW_MESSAGES["none"]

      can_sends.append(volkswagencan.create_mqb_hud_control(self.packer_gw, canbus.gateway, hca_enabled,
                                                            CS.steeringPressed, hud_alert, leftLaneVisible,
                                                            rightLaneVisible))

    #
    # Prepare GRA_ACC_01 message with ACC cruise control buttons.
    #
    # In order to support integration at the camera, we cannot depend on Panda
    # to filter the car's message and insert our own. Instead, we wait for the
    # car's message counter to change, and then cram 16 of our own on the bus
    # to roll over the 4-bit counter in the ~3.3ms before the car sends again.
    #

    if CS.graCounter != self.gra_acc_msgctr_last:
      if enabled and CS.standstill and frame > (self.gra_acc_resume_last + 100):
        # Blip the Resume button ~1x/second if we're engaged at standstill
        # FIXME: This is a naive implementation, improve with visiond or radar input.
        # A subset of MQBs like to "creep" too aggressively with this implementation.
        self.acc_vbp_type = "resumeCruise"

      elif not enabled and CS.accEnabled:
        # Cancel ACC if it's engaged with OP disengaged.
        self.acc_vbp_type = "cancel"

      # Insert any virtual button press generated by openpilot controls, to
      # conform with safety requirements or to update the ACC speed setpoint.

      if self.acc_vbp_type is not None:
        # Copy the ACC control button state from the car, and transparently pass
        # most of it onto the ACC radar, with a few exceptions for conformance
        # to comma's more-strict safety model, and to support resume-from-stop.
        buttonStatesToSend = CS.buttonStates.copy()
        buttonStatesToSend[self.acc_vbp_type] = True

        for spamidx in range(16):
          idx = (CS.graCounter + spamidx + 1) % 16
          can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_gw, canbus.extended,
                                                                    buttonStatesToSend, CS, idx))

      self.gra_acc_msgctr_last = CS.graCounter

    return can_sends
