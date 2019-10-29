from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC, MQB_LDW_MESSAGES, BUTTON_STATES
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
    self.same_torque_cnt = 0
    self.hca_enabled_cnt = 0
    self.hca_msg_counter = 0
    self.acc_vbp_type = None
    self.gra_acc_msgctr_last = 0
    self.gra_acc_button_last = 0
    self.gra_acc_ondemand_trigger = False
    self.gra_acc_ondemand_sending = False
    self.gra_acc_ondemand_sent = 0
    self.buttonStatesToSend = BUTTON_STATES.copy()

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
    # Prepare HCA_01 Heading Control Assist messages with steering torque.
    #
    # The factory camera sends at 50Hz while steering and 1Hz when not. When
    # OP is active, Panda filters HCA_01 from the factory camera and OP emits
    # HCA_01 at 50Hz. Rate switching creates some confusion in Cabana and
    # doesn't seem to add value at this time. The rack will accept HCA_01 at
    # 100Hz if we want to control at finer resolution in the future.
    #

    if frame % P.HCA_STEP == 0:

      # FAULT AVOIDANCE: HCA may not be enabled at standstill. Also stop
      # commanding HCA if there's a fault, so the steering rack recovers.
      if enabled and not (CS.standstill or CS.steeringFault):

        # FAULT AVOIDANCE: Requested HCA torque may not exceed 3.0nm. This is
        # inherently handled by scaling to STEER_MAX. The rack doesn't seem
        # to care about up/down rate, but we have some evidence it may do its
        # own rate limiting, and matching OP helps for accurate tuning.
        apply_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steeringTorque, P)

        if apply_steer == 0:
          # FAULT AVOIDANCE: HCA may not be enabled for > 360 seconds. Setting
          # the HCA disabled flag for one frame is an effective workaround.
          # We can do this very frequently by disabling HCA when apply_steer
          # happens to be exactly zero, which happens naturally during a
          # subset of direction changes. This could be expanded with a small
          # dead-zone to capture all zero crossings, but not seeing a need.
          hca_enabled = False
          self.hca_enabled_cnt = 0
        else:
          self.hca_enabled_cnt += 1
          if self.hca_enabled_cnt >=  118 * (100 / HCA_STEP):  # 118s
            # The Kansas I-70 Crosswind Problem: if we truly do need to steer
            # in one direction for > 360 seconds, we have to disable HCA for a
            # frame while actively steering. Testing shows we can just set the
            # disabled flag, and keep sending a torque value, which keeps the
            # Panda torque rate limiting safety happy. Do so 3x within the 360
            # second window for safety.
            hca_enabled = False
            self.hca_enabled_cnt = 0
          else:
            hca_enabled = True
            # FAULT AVOIDANCE: HCA torque may not be static for > 6 seconds.
            # This is intended to detect the sending camera being stuck or
            # frozen. OP can trip this on a curve if steering is saturated.
            # Avoid this by reducing torque 0.01nm for one frame. Do so 3x
            # within the 6 second period for safety.
            if self.apply_steer_last == apply_steer:
              self.same_torque_cnt += 1
              if self.same_torque_cnt > 1.9 * (100 / HCA_STEP):  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.same_torque_cnt = 0
            else:
              self.same_torque_cnt = 0

      else:
        # Continue sending HCA_01 messages, with the enable flags turned off.
        hca_enabled = False
        apply_steer = 0


      self.apply_steer_last = apply_steer

      idx = (frame / P.HCA_STEP) % 16
      can_sends.append(volkswagencan.create_mqb_steering_control(self.packer_gw, canbus.gateway, apply_steer,
                                                                 idx, hca_enabled))

    #
    # Prepare LDW_02 HUD messages with lane borders, confidence levels, and
    # the LKAS status LED.
    #
    # The factory camera emits this message at 10Hz. When OP is active, Panda
    # filters LDW_02 from the factory camera and OP emits LDW_02 at 10Hz.
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

    # Create any virtual button press generated by openpilot controls, to
    # conform with safety requirements or to update the ACC speed setpoint.

    if not enabled and CS.accEnabled and frame > (self.gra_acc_button_last + 100):
      # Cancel ACC if it's engaged with OP disengaged.
      self.gra_acc_ondemand_trigger = True
      self.buttonStatesToSend["cancel"] = True

    elif enabled and CS.standstill and frame > (self.gra_acc_button_last + 100):
      # Blip the Resume button ~1x/second if we're engaged at standstill
      # FIXME: This is a naive implementation, improve with visiond or radar input.
      # A subset of MQBs like to "creep" too aggressively with this implementation.
      self.gra_acc_ondemand_trigger = True
      self.buttonStatesToSend["resumeCruise"] = True

    # Prepare GRA_ACC_01 cruise control button message. The car sends this
    # message at 33hz. OP sends it on-demand only for virtual button presses.
    #
    # OP/Panda can see this message but can't filter it when integrated at the
    # J242 LKAS camera. It could so so if integrated at the J533 gateway, but
    # we need a generalized solution that works for either.
    #
    # The message is counter protected, so we need to time our transmissions
    # very precisely to achieve fast and fault-free switching between the set
    # of messages the R242 ACC radar is willing to listen to.
    #
    # CAR @ 33Hz: 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  3  4  5  6 ...
    # EON @ 33Hz:           4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  3  GG
    #
    # If OP needs to send a button press, it waits to see a GRA_ACC_01 message
    # counter change, and then immediately follows up with the next increment.
    # It plays out another 16 (an arbitrary number, about half a second) in
    # lockstep with each new message from the car. Because the OP's message
    # counter is synced to the car, R242 pays attention to us immediately. The
    # messages from the car get discarded as duplicates. When OP stops, the
    # gap to the next car message is less than 2 * GRA_ACC_STEP and R242 seems
    # to tolerate this just fine.

    # FIXME: This is ugly and I almost hope it doesn't work.

    if CS.graCounter != self.gra_acc_msgctr_last:
      if self.gra_acc_ondemand_trigger:
        # We're ready to begin sending the spam frames.
        self.gra_acc_button_last = frame
        self.gra_acc_ondemand_trigger = False
        self.gra_acc_ondemand_sending = True
        self.gra_acc_ondemand_sent = 0
      if self.gra_acc_ondemand_sending:
        # Sequence the spam frames out on the bus, +1 from the car's own.
        idx = (CS.graCounter + 1) % 16
        can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_gw, canbus.extended, self.buttonStatesToSend, CS, idx))
        self.gra_acc_ondemand_sent += 1
        if self.gra_acc_ondemand_sent >= 16:
          self.gra_acc_ondemand_sending = False
          self.buttonStatesToSend = BUTTON_STATES.copy()

      self.gra_acc_msgctr_last = CS.graCounter

    return can_sends
