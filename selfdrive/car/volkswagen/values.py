from cereal import car
from selfdrive.car import dbc_dict
Ecu = car.CarParams.Ecu

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
  LDW_STEP = 10                  # LDW_02 message frequency 10Hz
  GRA_ACC_STEP = 3               # GRA_ACC_01 message frequency 33Hz

  GRA_VBP_STEP = 100             # Send ACC virtual button presses once a second
  GRA_VBP_COUNT = 16             # Send VBP messages for ~0.5s (GRA_ACC_STEP * 16)

  # Observed documented MQB limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # Limiting rate-of-change based on real-world testing and Comma's safety
  # requirements for minimum time to lane departure.
  STEER_MAX = 300                # Max heading control assist torque 3.00 Nm
  STEER_DELTA_UP = 4             # Max HCA reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

class CANBUS:
  pt = 0
  cam = 2

NWL = car.CarParams.NetworkLocation
TRANS = car.CarParams.TransmissionType
GEAR = car.CarState.GearShifter
ECU = car.CarParams.Ecu

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

MQB_LDW_MESSAGES = {
  "none": 0,                            # Nothing to display
  "laneAssistUnavailChime": 1,          # "Lane Assist currently not available." with chime
  "laneAssistUnavailNoSensorChime": 3,  # "Lane Assist not available. No sensor view." with chime
  "laneAssistTakeOverUrgent": 4,        # "Lane Assist: Please Take Over Steering" with urgent beep
  "emergencyAssistUrgent": 6,           # "Emergency Assist: Please Take Over Steering" with urgent beep
  "laneAssistTakeOverChime": 7,         # "Lane Assist: Please Take Over Steering" with chime
  "laneAssistTakeOverSilent": 8,        # "Lane Assist: Please Take Over Steering" silent
  "emergencyAssistChangingLanes": 9,    # "Emergency Assist: Changing lanes..." with urgent beep
  "laneAssistDeactivated": 10,          # "Lane Assist deactivated." silent with persistent icon afterward
}

class CAR:
  VW_GOLF_GTI_MK7 = "Volkswagen Golf GTI Mk7"
  VW_GOLF_R_MK7 = "Volkswagen Golf R Mk7"

MQB_CARS = {
  CAR.VW_GOLF_GTI_MK7,
  CAR.VW_GOLF_R_MK7
}

# Volkswagen port using FP 2.0 exclusively
FINGERPRINTS = {}

FW_VERSIONS = {
  CAR.VW_GOLF_GTI_MK7: {
    # Mk7 2013-2017 and Mk7.5 facelift 2018-2020
    (Ecu.eps, 0x712, None): [b'5011', b'5043', b'5061', b'5063'],
    (Ecu.esp, 0x713, None): [b'0371', b'0385', b'0393', b'0557'],
    (Ecu.srs, 0x715, None): [b'0385', b'0386', b'0825', b'0830'],
  },
  CAR.VW_GOLF_R_MK7: {
    # Mk7 2013-2017 and Mk7.5 facelift 2018-2020
    (Ecu.eps, 0x712, None): [b'5043', b'5072', b'5081', b'5082'],
    (Ecu.esp, 0x713, None): [b'0108', b'0457', b'0523', b'0557', b'0643'],
    (Ecu.srs, 0x715, None): [b'0385', b'0386', b'0825', b'0830'],
  },
}

DBC = {
  CAR.VW_GOLF_GTI_MK7: dbc_dict('vw_mqb_2010', None),
  CAR.VW_GOLF_R_MK7: dbc_dict('vw_mqb_2010', None),
}