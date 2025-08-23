from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_party = can_parsers[Bus.party]
    ret = structs.CarState()

    # car speed
    # Basic vehicle state from BCM2_SPEED
    ret.vEgoRaw = cp_party.vl["BCM2_SPEED"]["SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas
    ret.gasPressed = False # TODO: add gas pedal

    # brake
    ret.brakePressed = bool(cp_party.vl["BCM2"]["BRAKE_PEDAL_PRESSED_A"] or cp_party.vl["BCM2"]["BRAKE_PEDAL_PRESSED_B"])
    ret.parkingBrake = False # TODO: add parking brake

    # steering wheel
    ret.steeringAngleDeg = cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR'] # TODO: Fix units and scaling

    # For torque-based control, we need steering torque feedback
    # TODO: Find actual steering torque signals in the DBC or reverse engineer them
    ret.steeringTorque = abs(cp_party.vl['PSCM']['DRIVER_INPUT_DEVIATION'])/10.0  # Driver torque
    ret.steeringTorqueEps = 0  # EPS torque - placeholder until signal is found
    ret.steeringPressed = abs(cp_party.vl['PSCM']['DRIVER_INPUT_DEVIATION']) > 0 # TODO: Use torque signal instead of deviation, if found...

    # EPS status - placeholder until actual signal is found
    self.eps_active = True  # Assume EPS is active for now

    # cruise
    # Cruise control / Pilot Assist status from BCM2
    ret.cruiseState.enabled = cp.vl["BCM2"]["CRUISE_OR_PILOT_ASSIST_ENGAGED"] == 1
    ret.cruiseState.available = True  # TODO: Determine actual availability
    ret.cruiseState.speed = 0  # TODO: Find cruise set speed
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = False

    # gear TODO
    #if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
    #  ret.gearShifter = GearShifter.reverse
    #else:
    #  ret.gearShifter = GearShifter.drive
    gearPosition = cp.vl['GEAR_POSITION']['GEAR_POSITION'] # 0: Parked; 1: R; 2: N; 3: D
    if gearPosition == 0:
      ret.gearShifter = GearShifter.park
    elif gearPosition == 1:
      ret.gearShifter = GearShifter.reverse
    elif gearPosition == 2:
      ret.gearShifter = GearShifter.neutral
    elif gearPosition == 3:
      ret.gearShifter = GearShifter.drive

    # blinkers TODO
    ret.leftBlinker = False
    ret.rightBlinker = False

    # lock info
    ret.doorOpen = False # TODO: add door open
    ret.seatbeltUnlatched = False # TODO: add seatbelt unlatched
    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], 2),
    }
