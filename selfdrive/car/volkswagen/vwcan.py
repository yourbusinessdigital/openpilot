from selfdrive.car.volkswagen.values import CAR, DBC

def create_steering_control(packer, apply_steer, idx, lkas_enabled):
  values = {
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
  return packer.make_can_msg("HCA_01", 0, values, idx)

def create_hud_control(packer, lkas_enabled, hud_alert, leftLaneVisible, rightLaneVisible):

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
