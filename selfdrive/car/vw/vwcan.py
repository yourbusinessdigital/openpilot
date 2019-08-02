import crcmod
from selfdrive.car.vw.values import CAR, DBC

# Python crcmod works differently from every other CRC calculator in the planet in some subtle
# way. The implied leading 1 on the polynomial isn't a big deal, but for some reason, we need
# to feed it initCrc 0x00 instead of 0xFF like it should be.
vw_checksum = crcmod.mkCrcFun(0x12F, initCrc=0x00, rev=False, xorOut=0xFF)

def create_steering_control(packer, bus, car_fingerprint, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_01_CRC": 0xB5, # Magic value that stands in for the CRC during calculation
    "HCA_01_BZ": idx,
    "3": 3,
    "254": 254,
    "7": 7,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
  }
  dat = packer.make_can_msg("HCA_01", 0, values)[2]
  dat = dat + '\0'
  checksum = vw_checksum(dat)
  values["HCA_01_CRC"] = checksum
  return packer.make_can_msg("HCA_01", 0, values)

def create_hud_control(packer, bus, car_fingerprint, lkas_enabled, hud_alert, leftLaneVisible, rightLaneVisible):

  if lkas_enabled:
    leftlanehud = 3 if leftLaneVisible else 1
    rightlanehud = 3 if rightLaneVisible else 1
  else:
    leftlanehud = 2 if leftLaneVisible else 1
    rightlanehud = 2 if rightLaneVisible else 1

  values = {
    "LDW_Unknown": 2, # FIXME: possible speed or attention relationship
    "Kombi_Lamp_Orange": 1 if lkas_enabled == 0 else 0,
    "Kombi_Lamp_Green": 1 if lkas_enabled == 1 else 0,
    "Left_Lane_Status": leftlanehud,
    "Right_Lane_Status": rightlanehud,
    "Alert_Message": hud_alert,
  }
  return packer.make_can_msg("LDW_02", 0, values)
