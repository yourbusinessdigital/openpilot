import numpy as np
from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser, CANDefine
from selfdrive.car.vw.values import DBC, CAR

# TODO: additional signals
#  * ACC following distance setpoint
#  * ACC distance to car in front
#  * Test ACC sensor blind/other error to make sure OP can react clearly and promptly
#  * GRA_ACC_01 steering wheel buttons for events, and to send cancels and speed changes
#  * Dimmung messages for EON screen brightness
#  * (( Klemmen_Status_01 for virtual terminal 15, but goes in Panda safety ))
#  * (( VIN_01 for auto platform ID, but probably goes in fingerprint or init ))
#  * ESP_05 ESP_Autohold_aktiv provides DSG and maybe even manual auto-hold state, may be important for auto-resume
#  * ESP_05 ESP_Verzoeg_EPB_verf may signify presence of an EPB, implying ACC stop-and-go support
#  * EPB_01 EPB_Status and friends tell us lots about the EPB, if present, still need to track down handbrake
#  * ACC_06 ACC_Typ may indicate stop-and-go support, @cben = 1, @jyoung8607 = 2

# TODO: Safety
#  * Seatbelt works to disable. Turns out stock ACC doesn't care, so OP gets to handle. Need to spam ACC cancel.
#  * Door open detection works to disable. ACC dies immediately anyway, but tested manually.
#  * Brake pressed. Obviously knocks out stock ACC. Have not made this explicit/redundant in OP yet.
#  * ESP disabled detection works, but stock ACC re-enables and creates a timing/conflict with OP, need to spam cancel.
#  * Gas pressed. Trivial but haven't done yet. Will need to spam ACC cancel.
#  * Control steering torque and ramp rate once we have a reasonable tune model.
#  * Look at Honda interface "last event" for possible example of showing Events even if stock cruise is off

# TODO: generic platform detection
#  * Collect 3x VIN_01 messages and detect chassis (e.g., AU=Golf Mk7) with inherent networking model
#  * Optionally, collect Motor_Code_01 for transmission type

# TODO: bus layout detection
#  * At J533 gateway/extended? Required to have any control over ACC long term, even a "lite" model
#  * At R242 camera? More traditional, but no possibility of controlling ACC
#  * At J533 gateway/running gear? Needed for lowline J533 in cars without ACC. Will fingerprint differently.
# FIXME: Temporarily use a hardcoded J533 vs R242 location during development.
CONNECTED_TO_GATEWAY = True

def get_gateway_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LWI_Lenkradwinkel", "LWI_01", 0),           # Absolute steering angle
    ("LWI_VZ_Lenkradwinkel", "LWI_01", 0),        # Steering angle sign
    ("LWI_Lenkradw_Geschw", "LWI_01", 0),         # Absolute steering rate
    ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),      # Steering rate sign
    ("ESP_HL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear left
    ("ESP_HR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear right
    ("ESP_VL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front left
    ("ESP_VR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front right
    ("ZV_FT_offen", "Gateway_72", 0),             # Door open, driver
    ("ZV_BT_offen", "Gateway_72", 0),             # Door open, passenger
    ("ZV_HFS_offen", "Gateway_72", 0),            # Door open, rear left
    ("ZV_HBFS_offen", "Gateway_72", 0),           # Door open, rear right
    ("ZV_HD_offen", "Gateway_72", 0),             # Trunk or hatch open
    ("BH_Blinker_li", "Gateway_72", 0),           # Left turn signal on
    ("BH_Blinker_re", "Gateway_72", 0),           # Right turn signal on
    ("GE_Fahrstufe", "Getriebe_11", 0),           # Transmission gear selector position
    ("AB_Gurtschloss_FA", "Airbag_02", 0),        # Seatbelt status, driver
    ("AB_Gurtschloss_BF", "Airbag_02", 0),        # Seatbelt status, passenger
    ("ESP_Fahrer_bremst", "ESP_05", 0),           # Brake pedal pressed
    ("ESP_Status_Bremsdruck", "ESP_05", 0),       # Brakes applied
    ("ESP_Bremsdruck", "ESP_05", 0),              # Brake pressure applied
    ("MO_Fahrpedalrohwert_01", "Motor_20", 0),    # Accelerator pedal value
    ("Driver_Strain", "EPS_01", 0),               # Absolute driver torque input
    ("Driver_Strain_VZ", "EPS_01", 0),            # Driver torque input sign
    ("HCA_Available", "EPS_01", 0),               # Steering rack HCA support configured
    ("ESP_Tastung_passiv", "ESP_21", 0),          # Stability control disabled
    ("KBI_MFA_v_Einheit_02", "Einheiten_01", 0),  # MPH vs KMH speed display
    ("KBI_Handbremse", "Kombi_01", 0),            # Manual handbrake applied
  ]

  checks = [
    # sig_address, frequency
    ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
    ("EPS_01", 100),      # From J500 Steering Assist with integrated sensors
    ("ESP_19", 100),      # From J104 ABS/ESP controller
    ("ESP_05", 50),       # From J104 ABS/ESP controller
    ("ESP_21", 50),       # From J104 ABS/ESP controller
    ("Motor_20", 50),     # From J623 Engine control module
    ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
    ("Getriebe_11", 20),  # From J743 Auto transmission control module
    ("Airbag_02", 5),     # From J234 Airbag control module
    ("Kombi_01", 2),      # From J285 Instrument cluster
    ("Einheiten_01", 1),  # From J??? not known if gateway, cluster, or BCM
  ]

  # FIXME: Temporarily use a hardcoded J533 vs R242 location during development.
  if not CONNECTED_TO_GATEWAY:
    signals += [("ACC_Status_ACC", "ACC_06", 0)]  # ACC engagement status
    signals += [("ACC_Typ", "ACC_06", 0)]         # ACC type (follow to stop, stop&go)
    signals += [("SetSpeed", "ACC_02", 0)]   # ACC set speed

    checks += [("ACC_06", 50)]  # From J428 ACC radar control module
    checks += [("ACC_02", 17)]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.gateway)


def get_extended_can_parser(CP, canbus):

  signals = [
    # sig_name, sig_address, default
  ]

  checks = [
    # sig_address, frequency
  ]

  # FIXME: Temporarily use a hardcoded J533 vs R242 location during development.
  if CONNECTED_TO_GATEWAY:
    signals += [("ACC_Status_ACC", "ACC_06", 0)]  # ACC engagement status
    signals += [("ACC_Typ", "ACC_06", 0)]         # ACC type (follow to stop, stop&go)
    signals += [("SetSpeed", "ACC_02", 0)]   # ACC set speed

    checks += [("ACC_06", 50)]  # From J428 ACC radar control module
    checks += [("ACC_02", 17)]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.extended)

def parse_gear_shifter(gear,vals):
  # Return mapping of gearshift position to selected gear. Eco is not a gear
  # understood by OP at this time, so map it to Drive. For other ports, Sport is
  # detected by OP as a no entry/soft cancel condition, so be consistent there.
  # Map Tiptronic (pseudo-manual mode) to Sport since OP doesn't have that either.
  #
  # Intention for the other ports was probably to provide consistent gas pedal behavior
  # for longitudinal use, but VW Bosch ACC provides m/s acceleration requests to the
  # ECU directly, pre-computed to match the Charisma driving profile as applicable,
  # so Drive/Sport/Eco don't really figure in to ACC behavior.
  val_to_capnp = {'P': 'park', 'R': 'reverse', 'N': 'neutral',
                  'D': 'drive', 'E': 'drive', 'S': 'sport', 'T': 'sport'}
  try:
    return val_to_capnp[vals[gear]]
  except KeyError:
    return "unknown"

class CarState(object):
  def __init__(self, CP, canbus):
    # initialize can parser
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = self.can_define.dv["Getriebe_11"]['GE_Fahrstufe']
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_torque_driver = 0
    self.steer_not_allowed = False
    self.main_on = False
    self.angle_steers_rate = 0
    self.steer_error = 0
    self.park_brake = 0
    self.esp_disabled = 0
    self.is_metric = 0

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

  def update(self, gw_cp, ex_cp):

    # FIXME: What does can_valid imply? Make sure we're handling it safely.
    self.can_valid = True

    # Check to make sure the electric power steering rack is configured
    # and ready to respond to HCA_01 messages.
    self.steer_error = not gw_cp.vl["EPS_01"]["HCA_Available"]

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.is_metric = not gw_cp.vl["Einheiten_01"]["KBI_MFA_v_Einheit_02"]

    # Update door and trunk/hatch lid open status
    self.door_all_closed = not any([gw_cp.vl["Gateway_72"]['ZV_FT_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_BT_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HFS_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HBFS_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HD_offen']])

    # Update turn signal status
    # TODO: Use a leading edge transition and timer to simulate real blinker state instead of momentary turnstalk
    #  * Right place for that might be in interface rather than carstate
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = gw_cp.vl["Gateway_72"]['BH_Blinker_li']
    self.right_blinker_on = gw_cp.vl["Gateway_72"]['BH_Blinker_re']

    # Update seatbelt fastened status
    self.seatbelt = 1 if gw_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] == 3 else 0

    # Update speed from ABS wheel speeds
    self.v_wheel_fl = gw_cp.vl["ESP_19"]['ESP_HL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_fr = gw_cp.vl["ESP_19"]['ESP_HR_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rl = gw_cp.vl["ESP_19"]['ESP_VL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rr = gw_cp.vl["ESP_19"]['ESP_VR_Radgeschw_02'] * CV.KPH_TO_MS
    speed_estimate = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))
    self.v_ego_raw = speed_estimate
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.1

    # Update steering angle
    if gw_cp.vl["LWI_01"]['LWI_VZ_Lenkradwinkel'] == 1:
      self.angle_steers = gw_cp.vl["LWI_01"]['LWI_Lenkradwinkel'] * -1
    else:
      self.angle_steers = gw_cp.vl["LWI_01"]['LWI_Lenkradwinkel']

    # Update steering rate
    if gw_cp.vl["LWI_01"]['LWI_VZ_Lenkradw_Geschw'] == 1:
      self.angle_steers_rate = gw_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw'] * -1
    else:
      self.angle_steers_rate = gw_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw']

    # Update driver steering torque input
    if gw_cp.vl["EPS_01"]['Driver_Strain_VZ'] == 1:
        self.steer_torque_driver = gw_cp.vl["EPS_01"]['Driver_Strain'] * -1
    else:
        self.steer_torque_driver = gw_cp.vl["EPS_01"]['Driver_Strain']
    self.steer_override = abs(self.steer_torque_driver) > 100

    # Update gas, brakes, and gearshift
    self.pedal_gas = gw_cp.vl["Motor_20"]['MO_Fahrpedalrohwert_01']
    self.brake_pressed = gw_cp.vl["ESP_05"]['ESP_Fahrer_bremst']
    self.brake_lights = gw_cp.vl["ESP_05"]['ESP_Status_Bremsdruck']
    self.user_brake = gw_cp.vl["ESP_05"]['ESP_Bremsdruck'] # TODO: this is pressure in Bar, not sure what OP expects
    self.park_brake = gw_cp.vl["Kombi_01"]['KBI_Handbremse'] # TODO: need to include an EPB check as well
    self.esp_disabled = gw_cp.vl["ESP_21"]['ESP_Tastung_passiv']
    can_gear_shifter = int(gw_cp.vl["Getriebe_11"]['GE_Fahrstufe'])
    self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)

    #
    # Update ACC engagement
    #
    if CONNECTED_TO_GATEWAY:
      self.acc_enabled = 1 if ex_cp.vl["ACC_06"]['ACC_Status_ACC'] > 1 else 0
      self.acc_active = 1 if ex_cp.vl["ACC_06"]['ACC_Status_ACC'] > 2 else 0
      self.cruise_set_speed = ex_cp.vl["ACC_02"]['SetSpeed'] * CV.KPH_TO_MS
    else:
      self.acc_enabled = 1 if gw_cp.vl["ACC_06"]['ACC_Status_ACC'] > 1 else 0
      self.acc_active = 1 if gw_cp.vl["ACC_06"]['ACC_Status_ACC'] > 2 else 0
      self.cruise_set_speed = gw_cp.vl["ACC_02"]['SetSpeed'] * CV.KPH_TO_MS
    self.main_on = self.acc_active
