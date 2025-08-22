from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.volvocan import create_lca_steering
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control - torque-based steering
    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Convert normalized torque to raw torque value
      apply_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))

      # Apply driver torque limits
      apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

      # Disable torque when not active
      if not CC.latActive:
        apply_torque = 0

      can_sends.append(create_lca_steering(self.packer, CC.latActive, apply_torque))

      self.apply_torque_last = apply_torque

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
