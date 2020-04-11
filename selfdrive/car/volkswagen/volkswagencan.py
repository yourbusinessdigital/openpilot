from selfdrive.config import Conversions as CV

# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "SET_ME_0X3": 0x3,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X07": 0x07,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_lkas_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, leftLaneVisible, rightLaneVisible):

  if hca_enabled:
    leftlanehud = 3 if leftLaneVisible else 1
    rightlanehud = 3 if rightLaneVisible else 1
  else:
    leftlanehud = 2 if leftLaneVisible else 1
    rightlanehud = 2 if rightLaneVisible else 1

  values = {
    "LDW_Unknown": 2, # FIXME: possible speed or attention relationship
    "Kombi_Lamp_Orange": 1 if hca_enabled and steering_pressed else 0,
    "Kombi_Lamp_Green": 1 if hca_enabled and not steering_pressed else 0,
    "Left_Lane_Status": leftlanehud,
    "Right_Lane_Status": rightlanehud,
    "Alert_Message": hud_alert,
  }
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_Typ468": CS.graTyp468,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }

  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)

def create_mqb_acc_control(packer, bus, acc_status, apply_accel, idx):
  values = {
    "ACC_Typ": 2,  # FIXME: locked to stop and go, need to tweak for cars that only support follow-to-stop
    "ACC_Status_ACC": acc_status,
    "ACC_StartStopp_Info": 1,  # FIXME: always set stop prevent flag for Stop-Start coordinator for now, get fancy later
    "ACC_Sollbeschleunigung_02": apply_accel if acc_status == 3 else 3.01,
    "ACC_zul_Regelabw_unten": 0.5,  # FIXME: need comfort regulation logic here
    "ACC_zul_Regelabw_oben": 0.5,  # FIXME: need comfort regulation logic here
    "ACC_neg_Sollbeschl_Grad_02": 3.0,  # FIXME: need gradient regulation logic here
    "ACC_pos_Sollbeschl_Grad_02": 3.0,  # FIXME: need gradient regulation logic here
    "ACC_Anfahren": 0,  # FIXME: set briefly when taking off from standstill
    "ACC_Anhalten": 0  # FIXME: hold true when at standstill
  }

  return packer.make_can_msg("ACC_06", bus, values, idx)

def create_mqb_acc_hud_control(packer, bus, acc_status, set_speed, idx):
  values = {
    "ACC_Status_Anziege": acc_status,
    "ACC_Wunschgeschw": set_speed * CV.MS_TO_KPH,
    "ACC_Gesetzte_Zeitluecke": 3,
    "ACC_Display_Prio": 3
  }

  return packer.make_can_msg("ACC_02", bus, values, idx)
