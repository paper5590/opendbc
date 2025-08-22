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
    ret.vEgoRaw = cp_party.vl["BCM2_SPEED"]["SPEED"] * CV.MS_TO_KPH
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas
    ret.gasPressed = False # TODO: add gas pedal

    # brake
    ret.brakePressed = bool(cp_party.vl["BCM2"]["BRAKE_PEDAL_PRESSED_A"] or cp_party.vl["BCM2"]["BRAKE_PEDAL_PRESSED_B"])
    ret.parkingBrake = False # TODO: add parking brake

    # steering wheel
    ret.steeringAngleDeg = cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR']
    #ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    #ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    #ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    #self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect

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
