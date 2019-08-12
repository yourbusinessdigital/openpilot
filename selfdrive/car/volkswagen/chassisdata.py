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

class VAG_GLOBAL:
  # Vehicle not supported unless explicitly set otherwise at platform or chassis level
  supported = False
  carName = None
  modelYear = None

  # OpenPilot tunables, defaults to be overridden at Chassis level.
  # Do not edit these defaults, edit the chassis.
  steerRatio = 15
  steerActuatorDelay = 0.12
  steerRateCost = 0.5

  kiBP, kpBP = [[0.],[0.]] # m/s
  kpV, kiV = [[0.5],[0.12]]
  kf = 0.000006
  maxBP = [0.] # m/s
  maxV = [1.]

# The Platform classes may contain platform-specific details for ACC radar,
# camera, steering, etc if needed later. Placeholders for now.

class PLATFORM_MQB(VAG_GLOBAL):
  # Modular Transverse Matrix
  # OpenPilot can generally support these vehicles
  platformName = "MQB"
  supported = True

class PLATFORM_MQBA0(PLATFORM_MQB):
  # Modular Transverse Matrix A0
  # Support TBD, tiny/city car version of MQB, appears to have different radar and steering equipment
  platformName = "MQBA0"
  supported = False # Could potentially change when we see one, if steering supports HCA

class PLATFORM_MQBEVO(PLATFORM_MQB):
  # Modular Transverse Matrix Evolution
  # Not yet in production, supposedly arriving with the 2020 VW Golf
  platformName = "MQBevo"
  supported = False # Subject to change when we see one

class PLATFORM_MLB(VAG_GLOBAL):
  # Modular Longitudinal Matrix
  # Not supported due to FlexRay-based powertrain comms
  platformName = "MLB"
  supported = False # Requires FlexRay support

class PLATFORM_MLBEVO(PLATFORM_MLB):
  # Modular Longitudinal Matrix Evolution
  # Not supported due to FlexRay-based powertrain comms
  platformName = "MLBevo"
  supported = False # Requires FlexRay support

class PLATFORM_PQ35(VAG_GLOBAL):
  # PQ35 legacy platforms, Mk4 Golf-style messaging.
  # Supportability to be determined, but can probably be done
  platformName = "PQ35"
  supported = False # Subject to change when we see one

class PLATFORM_PQ46(PLATFORM_PQ35):
  # Big brother of PQ35. B6/B7 Passat and Mk1 Tiguan.
  # Supportability to be determined, but can probably make facelifted Passat and Tiguan work
  platformName = "PQ46"
  supported = False # Subject to change when we see one

class PLATFORM_NMS(PLATFORM_PQ46):
  # New Midsize Sedan
  # When you want a MQB Passat, but cheaper, and with more cupholders, because MURICA
  # Theoretically a modified PQ46, may adjust inheritance later if needed
  # Supportability to be determined, but can probably make newer facelift NMS Passat work
  platformName = "NMS"
  supported = False # Subject to change when we see one

class PLATFORM_NSF(VAG_GLOBAL):
  # New Small Family
  # Support for these cars is unlikely in the near term. There is no factory ACC radar
  # or LKAS camera. It does have EPS but it's unclear if it can be made to respond to
  # heading control assist. It has city emergency braking at lower speeds from a laser
  # based sensor (not radar, not usable for cruise control).
  platformName = "NSF"
  supported = False # Unlikely to change in the near term

#
# SUPPORTED VEHICLES (MQB only at this time)
#

class Chassis_3G(PLATFORM_MQB):
  # Mk8 (B8) VW Passat with MQB underpinning, different from North America NMS sedan of the same name
  brand = "Volkswagen"
  modelName = "Passat"

class Chassis_3H(PLATFORM_MQB):
  # Mk1 VW Arteon four-door fastback
  brand = "Volkswagen"
  modelName = "Arteon"

class Chassis_5E(PLATFORM_MQB):
  brand = "Skoda"
  modelName = "Octavia"

  mass = 1360
  wheelbase = 2.69
  kpV, kiV = [[0.375],[0.1]]
  kf = 0.00006

class Chassis_5T(PLATFORM_MQB):
  brand = "Volkswagen"
  modelName = "Touran"

class Chassis_8V(PLATFORM_MQB):
  # Mk3 Audi A3 (!!! wheelbase varies slightly 3 door vs 5 door or sportback! !!!)
  brand = "Audi"
  modelName = "A3"

class Chassis_AD(PLATFORM_MQB):
  # Mk2 VW Tiguan (!!! SWB and LWB variants !!!)
  brand = "Volkswagen"
  modelName = "Tiguan"

class Chassis_AU(PLATFORM_MQB):
  # Mk7 and Mk7.5 VW Golf, Golf Sportwagen, Golf Alltrack, e-Golf, and Golf R, 2013-2019 depending on market
  brand = "Volkswagen"
  modelName = "Golf"

  mass = 1372
  wheelbase = 2.64 # Might need tweaking for Sportwagen and Alltrack
  kpV, kiV = [[0.5],[0.25]]
  kf = 0.00006

class Chassis_BU(PLATFORM_MQB):
  # Mk7 VW Jetta, 2019--
  brand = "Volkswagen"
  modelName = "Jetta"

class Chassis_CA(PLATFORM_MQB):
  # Mk1 VW Atlas (branded as VW Teramont in some markets)
  brand = "Volkswagen"
  modelName = "Atlas"

  # Chassis constants
  mass = 2042
  wheelbase = 2.97
  kpV, kiV = [[0.5],[0.25]]
  kf = 0.00006

class Chassis_FV(PLATFORM_MQB):
  # Mk3 Audi TT/TTS/TTRS
  brand = "Audi"
  modelName = "TT"

class Chassis_GA(PLATFORM_MQB):
  # Audi Q2 mini SUV
  brand = "Audi"
  modelName = "Q2"

  # Chassis constants
  mass = 1205
  wheelbase = 2.60

class Chassis_NS(PLATFORM_MQB):
  # Skoda Kodiaq, three-row midsize SUV
  brand = "Skoda"
  modelName = "Kodiaq"

  # Chassis constants (assumed from VW Atlas)
  mass = 2042
  wheelbase = 2.97
  kpV, kiV = [[0.5],[0.25]]
  kf = 0.00006

#
# UNSUPPORTED VEHICLES (everything except MQB at this time)
# Included for sake of completion, and for future support work
#

class Chassis_Unsupported:
  brand = "Unsupported"
  modelName = "Unsupported"

#
# Chassis type detected by the two-character code at positions 7 and 8 in
# the VIN. This list is not necessarily exhaustive, particularly for older
# and not supported vehicles.
#
# TODO: Locate VIN code for Audi A1 MQBA0 2018--
# TODO: Locate VIN code for Mk2 Audi Q3 MQB 2018--
# TODO: Seat Ibiza and possibly others, radar but SWaP cruise control LOL!!

CHASSIS_DATA = {
  "8V": Chassis_8V,           # MQB Audi A3 Mk3, 2013--
  "FF": Chassis_8V,           # MQB Audi A3 Mk3 NAR, 2015-- (duplicates 8V)
  "GA": Chassis_GA,           # MQB Audi Q2 SUV, 2016--
  "FV": Chassis_FV,           # MQB Audi TT/TTS/TTRS, 2014--
  "3H": Chassis_3H,           # MQB Volkswagen Arteon 2018--
  "CA": Chassis_CA,           # MQB Volkswagen Atlas 2018--
  "5C": Chassis_Unsupported,  # PQ35 Volkswagen Beetle 2012-- (EPS but no factory ACC or LKAS)
  "AU": Chassis_AU,           # MQB Volkswagen Golf Mk7 2013-2019
  "BU": Chassis_BU,           # MQB Volkswagen Jetta Mk7 2019--
  "A3": Chassis_Unsupported,  # NMS Volkswagen Passat NAR 2012-2019 (need access to a PQ46 to test)
  "3G": Chassis_3G,           # MQB Volkswagen Passat B8 RoW 2015-2019
  "3D": Chassis_Unsupported,  # ??? Volkswagen Phaeton 2003-2016 (ACC and LDW available, but no EPS)
  "AW": Chassis_Unsupported,  # MQBA0 Volkswagen Polo 2018--
  "AD": Chassis_AD,           # MQB Volkswagen Tiguan Mk2 2016--
  "5T": Chassis_5T,           # MQB Volkswagen Touran Mk2 2015--
  "7P": Chassis_Unsupported,  # ??? Volkswagen Touareg 2011-2017 NAR (ACC and LDW available, but no EPS)
  "CR": Chassis_Unsupported,  # MLBevo Volkswagen Touareg 2018-- (MLBevo, probably FlexRay)
  "12": Chassis_Unsupported,  # MQBA0 Volkswagen Up! 2012-- (ACC and LKAS not available)
  "KJ": Chassis_Unsupported,  # MQBA0 Seat Ibiza 2017--
  "KF": Chassis_Unsupported,  # MQBA0 Seat Mii 2012-- (ACC and LKAS not available)
  "AA": Chassis_Unsupported,  # MQBA0 Conflict - any of Up!, Mii, or Citigo
  "NF": Chassis_Unsupported,  # MQBA0 Skoda Citigo 2012--
  "NS": Chassis_NS,           # MQB Skoda Kodiaq 2016--
  "5E": Chassis_5E,           # MQB Skoda Octavia 2013--
  "NE": Chassis_5E,           # MQB Skoda Octavia 2013-- (duplicates 5E)
}

def vw_identify_by_vin(vin):
  chassis = CHASSIS_DATA[vin[6:8]]
  chassis.modelYear = VIN_MODEL_YEARS[vin[9]]
  chassis.carName = str(chassis.modelYear) + " " + chassis.brand + " " + chassis.modelName
  return chassis
