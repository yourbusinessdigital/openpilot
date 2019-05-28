from numpy import clip
#from cereal import car, log

# ***************************************************************************
#
# Please note:
# Below is a first cut at detailed VAG vehicle platform identification. This
# will provide an ESTIMATE of mass and other properties but does NOT YET try
# to identify sub-types (wagon vs hatch, etc). We can discover much of that
# by VIN in North America, but not in other markets.
#
# ***************************************************************************

# VIN character to model-year mapping. VINs do not contain the letters i, o,
# q, u, z, or the number 0 (zero) so those are skipped.
VIN_MODEL_YEARS = {
  "Y": 2000, "1": 2001, "2": 2002, "3": 2003, "4": 2004, "5": 2005, "6": 2006,
  "7": 2007, "8": 2008, "9": 2009, "A": 2010, "B": 2011, "C": 2012, "D": 2013,
  "E": 2014, "F": 2015, "G": 2016, "H": 2017, "J": 2018, "K": 2019,
}

class RADAR_BOSCH_MRR:
  pass

class RADAR_BOSCH_MRR1PLUS(RADAR_BOSCH_MRR):
  MaxDetectionRange = 160 # Meters
  MaxObjectsDetected = 32
  HorizontalFoV = 45 # Degrees
  VerticalFoV = 13 # Degrees

class RADAR_BOSCH_MRREVO14F(RADAR_BOSCH_MRR):
  MaxDetectionRange = 160 # Meters
  MaxObjectsDetected = 32
  HorizontalFoV = 45 # Degrees
  VerticalFoV = 13 # Degrees

class RADAR_BOSCH_LRR:
  # Various Audi, older Touareg and Phaeton, to be developed further later if needed
  pass

class RADAR_BOSCH_LRR1(RADAR_BOSCH_LRR):
  MaxDetectionRange = 150 # Meters

class RADAR_BOSCH_LRR2(RADAR_BOSCH_LRR):
  MaxDetectionRange = 200 # Meters

class RADAR_BOSCH_LRR3(RADAR_BOSCH_LRR):
  MaxDetectionRange = 250 # Meters

class RADAR_BOSCH_LRR4(RADAR_BOSCH_LRR):
  MaxDetectionRange = 250 # Meters
  MaxObjectsDetected = 24
  HorizontalFoV =
  pass

class CAMERA_BOSCH:
  pass

class CAMERA_BOSCH_MPC1:
  pass

class CAMERA_BOSCH_MPC2:
  pass


class VAG_GLOBAL:
  # Vehicle not supported unless explicitly set otherwise at platform or chassis level
  supported = False

  # OpenPilot tunables, defaults to be overridden at Chassis level
  steerRatio = 14
  steerActuatorDelay = 0.05
  steerRateCost = 0.5

  eonToFront = 0.5
  steerReactance = 0.7
  steerInductance = 1.0
  steerResistance = 1.0

#
# The Platform classes will contain platform-specific details for ACC radar,
# camera, steering, etc. Placeholders for now.
#

class PLATFORM_MQB(VAG_GLOBAL):
  # OpenPilot can generally support these vehicles
  platformName = "Modular Transverse Matrix"
  supported = True

class PLATFORM_MQBA0(PLATFORM_MQB):
  # Support TBD, tiny/city car version of MQB, appears to have different radar and steering equipment
  platformName = "Modular Transverse Matrix A0"
  supported = False # Subject to change when we see one

class PLATFORM_MQBEVO(PLATFORM_MQB):
  # Not yet in production, supposedly arriving with the 2020 VW Golf
  platformName = "Modular Transverse Matrix Evolution"
  supported = False # Subject to change when we see one

class PLATFORM_MLB(VAG_GLOBAL):
  # Unknown so far, but probably workable much like MQB
  platformName = "Modular Longitudinal Matrix"
  supported = False # Subject to change when we see one

class PLATFORM_MLBEVO(PLATFORM_MLB):
  # Unknown so far, but probably NOT supportable without FlexRay
  platformName = "Modular Longitudinal Matrix Evolution"
  supported = False # Almost certainly going to need FlexRay

class PLATFORM_PQ35(VAG_GLOBAL):
  # Supportability to be determined, but can probably make Mk2 facelifted Octavia work
  platformName = "PQ35"
  supported = False # Subject to change when we see one

class PLATFORM_PQ46(PLATFORM_PQ35):
  # Big brother of PQ35. B6/B7 Passat and Mk1 Tiguan.
  # Supportability to be determined, but can probably make facelifted Passat and Tiguan work
  platformName = "PQ46"
  supported = False # Subject to change when we see one

class PLATFORM_NMS(PLATFORM_PQ46):
  # When you want a MQB Passat, but cheaper, and with more cupholders, because MURICA
  # Theoretically a modified PQ46, may adjust inheritance later if needed
  # Supportability to be determined, but can probably make newer facelift NMS Passat work
  platformName = "New Midsize Sedan"
  supported = False # Subject to change when we see one

class PLATFORM_NSF(VAG_GLOBAL):
  # Support for these cars is unlikely in the near term. There is no factory ACC radar
  # or LKAS camera. It does have EPS but it's unclear if it can be made to respond to
  # heading control assist. It has city emergency braking at lower speeds from a laser
  # based sensor (not radar, not usable for cruise control).
  platformName = "New Small Family"
  supported = False # This will change on February 31st

#
# Individual vehicle definitions.
#

class Chassis_Unsupported:
  brand = "Unknown"
  modelName = "Unknown"
  tested = False
  supported = False

class Chassis_12(PLATFORM_NSF):
  # Volkswagen Up! micro/city car. Support is unlikely due to platform restrictions.
  brand = "Volkswagen"
  modelName = "Up!"
  tested = False

class Chassis_3G(PLATFORM_MQB):
  # Mk8 (B8) VW Passat with MQB underpinning, not in North America
  brand = "Volkswagen"
  modelName = "Passat"
  tested = False

class Chassis_3H(PLATFORM_MQB):
  # Mk1 VW Arteon four-door fastback
  brand = "Volkswagen"
  modelName = "Arteon"
  tested = False

class Chassis_5E(PLATFORM_MQB):
  brand = "Skoda"
  modelName = "Octavia"
  tested = True

class Chassis_5T(PLATFORM_MQB):
  brand = "Volkswagen"
  modelName = "Touran"
  tested = True

class Chassis_8V(PLATFORM_MQB):
  # Mk3 Audi A3 (!!! wheelbase varies slightly 3 door vs 5 door or sportback! !!!)
  brand = "Audi"
  modelName = "A3"
  tested = False

class Chassis_A3(PLATFORM_NMS):
  # North America specific variant of VW Passat, 2011-2019
  brand = "Volkswagen"
  modelName = "Passat"
  tested = False

class Chassis_AD(PLATFORM_MQB):
  # Mk2 VW Tiguan (!!! SWB and LWB variants !!!)
  brand = "Volkswagen"
  modelName = "Tiguan"
  tested = False

class Chassis_AU(PLATFORM_MQB):
  # Mk7 and Mk7.5 VW Golf, Golf SportWagen, Golf Alltrack, e-Golf, and Golf R
  brand = "Volkswagen"
  modelName = "Golf"
  tested = True

  # Chassis constants
  mass = 1372
  wheelbase = 2.64

class Chassis_AW(PLATFORM_MQBA0):
  # Support possibility TBD
  brand = "Volkswagen"
  modelName = "Polo"
  tested = False

class Chassis_BU(PLATFORM_MQB):
  # Mk7 VW Jetta
  brand = "Volkswagen"
  modelName = "Jetta"
  tested = False

class Chassis_CA(PLATFORM_MQB):
  # Mk1 VW Atlas (branded as VW Teramont in some markets)
  brand = "Volkswagen"
  modelName = "Atlas"
  tested = False

  # Chassis constants
  mass = 2042
  wheelbase = 2.97

class Chassis_FV(PLATFORM_MQB):
  # Mk3 Audi TT/TTS/TTRS
  brand = "Audi"
  modelName = "TT"
  tested = False

class Chassis_GA(PLATFORM_MQB):
  # Audi Q2 mini SUV
  brand = "Audi"
  modelName = "Q2"
  tested = False

  # Chassis constants
  mass = 1205
  wheelbase = 2.60

class Chassis_KF(PLATFORM_NSF):
  # Seat Mii, micro/city car, unsupported platform
  brand = "Seat"
  modelName = "Mii"
  tested = False

class Chassis_KJ(PLATFORM_MQBA0):
  # Support possibility TBD
  brand = "Seat"
  modelName = "Ibiza"
  tested = False

class Chassis_NF(PLATFORM_NSF):
  # Skoda Citigo, micro/city car, unsupported platform
  brand = "Skoda"
  modelName = "Citigo"
  tested = False

class Chassis_NS(PLATFORM_MQB):
  # Skoda Kodiaq, three-row midsize SUV
  brand = "Skoda"
  modelName = "Kodiaq"
  tested = False

#
# Chassis type detected by the two-character code at positions 7 and 8 in
# the VIN. This list is not necessarily exhaustive, particularly for older
# and not supported vehicles.
#
# TODO: Locate VIN code for Audi A1 MQB 2018--
# TODO: Locate VIN code for Mk2 Audi Q3 MQB 2018--
# TODO: Seat Ibiza and possibly others, radar but SWaP cruise control LOL!!

CHASSIS_DATA = {
  "8V": Chassis_8V,           # Audi A3 Mk3, 2013--
  "FF": Chassis_8V,           # Audi A3 Mk3 NAR, 2015-- (duplicates 8V)
  "GA": Chassis_GA,           # Audi Q2 SUV, 2016--
  "FV": Chassis_FV,           # Audi TT/TTS/TTRS, 2014--
  "3H": Chassis_3H,           # Volkswagen Arteon 2018--
  "CA": Chassis_CA,           # Volkswagen Atlas 2018--
  "5C": Chassis_Unsupported,  # Volkswagen Beetle 2012-- (PQ35, EPS but no factory ACC or LKAS)
  "AU": Chassis_AU,           # Volkswagen Golf Mk7 2013-2019
  "BU": Chassis_BU,           # Volkswagen Jetta Mk7 2019--
  "A3": Chassis_Unsupported,  # Volkswagen Passat NMS NAR 2012-2019 (need access to a PQ46 to test)
  "3G": Chassis_3G,           # Volkswagen Passat B8 RoW 2015-2019
  "3D": Chassis_Unsupported,  # Volkswagen Phaeton 2003-2016 (ACC and LDW available, but no EPS)
  "AW": Chassis_AW,           # Volkswagen Polo 2018--
  "AD": Chassis_AD,           # Volkswagen Tiguan Mk2 2016--
  "5T": Chassis_5T,           # Volkswagen Touran Mk2 2015--
  "7P": Chassis_Unsupported,  # Volkswagen Touareg 2011-2017 NAR (ACC and LDW available, but no EPS)
  "CR": Chassis_Unsupported,  # Volkswagen Touareg 2018-- (MLBevo, probably FlexRay)
  "12": Chassis_12,           # Volkswagen Up! 2012-- (ACC and LKAS not available)
  "KJ": Chassis_KJ,           # Seat Ibiza 2017--
  "KF": Chassis_KF,           # Seat Mii 2012-- (ACC and LKAS not available)
  "AA": Chassis_KF,           # Conflict - any of Up!, Mii, or Citigo
  "NF": Chassis_NF,           # Skoda Citigo 2012--
  "NS": Chassis_NS,           # Skoda Kodiaq 2016--
  "5E": Chassis_5E,           # Skoda Octavia 2013--
  "NE": Chassis_5E,           # Skoda Octavia 2013-- (duplicates 5E)
}

def identify_by_vin(vin):
  chassis_id = vin[6:8]
  my_id = vin[9]
  model_year = VIN_MODEL_YEARS[my_id]
  return CHASSIS_DATA[chassis_id]

testvin = "WVWVF7AU2JW177386"
thisCar = identify_by_vin(testvin)

print thisCar.brand, thisCar.modelName, thisCar.platformName

#print clip(100,50,200)