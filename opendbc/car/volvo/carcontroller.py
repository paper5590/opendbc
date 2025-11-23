from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.helpers import LCA3CounterSync
from opendbc.car.volvo.volvocan import create_lca_steering, create_pscm_message, create_lca_3_message, create_lca_2_message, create_lca_4_message, create_lca_5_message, create_speed_2_message, create_speed_3_message, create_0x1a_message, create_gear_position_message, create_egsm_message, create_pscm_related_message
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.party])
    self.apply_torque_last = 0

    self.gear_acc = 60
    self.lca_4_acc = 0  # Bresenham accumulator for 29 Hz

    # Counter management for LCA_2
    self.lca_2_counter_1 = None  # Will grab initial value from CarState
    self.lca_2_counter_2 = None

    # Counter management for PSCM_RELATED
    self.pscm_related_counter = None  # Will grab initial value from CarState

    # Counter management for LCA_3 (pattern-based)
    self.lca_3_counter_sync = LCA3CounterSync()

    # Counter management for LCA_5 (formerly SPEED_1)
    self.lca_5_counter = None  # Will grab initial value from CarState

    self.last_lat_active = False  # Track state

  def update(self, CC, CS, now_nanos):
    CS.CC_frame = self.frame
    can_sends = []
    actuators = CC.actuators

    # Detect disengagement
    if not CC.latActive and self.last_lat_active:
      self.lca_commands.reset()  # Clear state ← IMPORTANT!

    # lateral control - torque-based steering
    # NOTE: LCA message is sent every frame (even when inactive) to replace stock LCA
    # Stock LCA is permanently blocked by panda safety, so we must always send
    if self.frame % CarControllerParams.STEER_STEP == 0: # 100 Hz
      # Convert normalized torque to raw torque value
      #apply_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))

      # Your apply_torque: +1.0 = left, -1.0 = right
      apply_torque = actuators.torque  # Already normalized [-1.0, +1.0]

      # Disable torque when not active
      if not CC.latActive:
        apply_torque = 0
        lca_steer = 0
      else:
        # Calculate LCA_5_STEER (signed int8: -128 to 127)
        # Scale normalized torque to signed byte range
        lca_steer = int(round(apply_torque * 127.0))  # Maps [-1.0, 1.0] to [-127, 127]

      # Apply driver torque limits
      # apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last,
      #                                                CS.out.steeringTorque, CarControllerParams)

      # LCA - 0x58 - 100 Hz
      can_sends.append(create_lca_steering(self.packer, CC.latActive, lca_steer, CS.msg_lca))
      self.apply_torque_last = apply_torque

      # Check if PA hands-on-wheel spoof toggle is enabled (bit 7 of alternativeExperience)
      spoof_pa_hands_enabled = bool(self.CP.alternativeExperience & 128)
      spoof_pa_hands = CS.pilot_assist_engaged and spoof_pa_hands_enabled
      # PSCM (bus 2 -> 0) - 0x16 - 100 Hz
      can_sends.append(create_pscm_message(self.packer, CC.latActive, CS.msg_pscm, self.frame, spoof_pa_hands))
      # EGSM - 0x45 - 100 Hz
      #can_sends.append(create_egsm_message(self.packer, CS.msg_egsm))

      # PSCM_RELATED (bus 2 -> 0) - 0x17 - 100 Hz
      # Initialize counter from CarState on first run
      if self.pscm_related_counter is None:
        self.pscm_related_counter = CS.msg_pscm_related['SIG1_BYTE_1_HI_NIBBLE']

      # Increment counter by +1, wrap from 14 → 0 (modulo 15)
      self.pscm_related_counter = (self.pscm_related_counter + 1) % 15

      can_sends.append(create_pscm_related_message(self.packer, CC.latActive, CS.pilot_assist_engaged,
                                                     CS.msg_pscm_related, self.pscm_related_counter))

    # LCA_3 - 0x57 - avg 66.66 Hz
    #if (self.frame * 67) % 100 < 67: # if (self.frame % 3) < 2:
    # 0x57 at ~66.67 Hz: send on 2 out of every 3 frames
    # Pattern: send on frame % 3 == 0 or 2, skip when frame % 3 == 1
    if self.frame % 3 != 1:  # → 2/3 * 100 Hz = 66.67 Hz
      # Update counter with observed value, get counter to send
      counter, is_synced = self.lca_3_counter_sync.update(CS.msg_lca_3['COUNTER_1'])
      can_sends.append(create_lca_3_message(self.packer, CC.latActive, lca_steer, CS.msg_lca_3, counter))
      #can_sends.append(create_0x1a_message(self.packer, CS.msg_0x1a))
      pass

    # SPEED messages - 0x60, 0x67, 0x68 - 50 Hz
    if self.frame % 2 == 0: # 50 Hz
      #can_sends.append(create_speed_3_message(self.packer, CS.msg_speed_3))
      #can_sends.append(create_speed_1_message(self.packer, CS.msg_speed_1))
      #can_sends.append(create_speed_2_message(self.packer, CS.msg_speed_2))
      pass

    # LCA_2 - 0x69 - 50 Hz
    # Spoof PILOT_ASSIST_ENGAGED to keep PSCM accepting LCA commands
    if self.frame % 2 == 0: # 50 Hz
      # Initialize counters from CarState on first run
      if self.lca_2_counter_1 is None:
        self.lca_2_counter_1 = CS.msg_lca_2['COUNTER_1']
        self.lca_2_counter_2 = CS.msg_lca_2['COUNTER_2']

      # Increment counters (COUNTER_1 by +2, COUNTER_2 by +4, both modulo 16)
      self.lca_2_counter_1 = (self.lca_2_counter_1 + 2) % 16
      self.lca_2_counter_2 = (self.lca_2_counter_2 + 4) % 16

      can_sends.append(create_lca_2_message(self.packer, CC.latActive, CS.msg_lca_2,
                                            self.lca_2_counter_1, self.lca_2_counter_2))
      pass

    # LCA_5 (formerly SPEED_1) - 0x67 - 50 Hz
    # Contains wheel speeds + LCA signals (LCA_TURN_BITS, LCA_5_STEER)
    if self.frame % 2 == 0: # 50 Hz
      # Initialize counter from CarState on first run
      if self.lca_5_counter is None:
        self.lca_5_counter = CS.msg_lca_5['COUNTER']

      # Increment counter by +4, wrap at 15 (0xF never used)
      self.lca_5_counter = (self.lca_5_counter + 4) % 15

      can_sends.append(create_lca_5_message(self.packer, CC.latActive, lca_steer, CS.msg_lca_5, self.lca_5_counter, CS.out.steeringAngleDeg))

    # LCA_4 - 0x90 - 29 Hz
    # Spoof LCA_ENABLE bits to maintain PA ON state when openpilot is active
    # Using Bresenham-style accumulator for precise 29 Hz
    self.lca_4_acc += 29
    if self.lca_4_acc >= 100:
      self.lca_4_acc -= 100
      can_sends.append(create_lca_4_message(self.packer, CC.latActive, CS.msg_lca_4))

    # GEAR_POSITION - 0x80 - 40 Hz
    #self.gear_acc += 40 # Bresenham-style approach
    #if self.gear_acc >= 100:
    #    self.gear_acc -= 100
    if self.frame % 5 == 0 or self.frame % 5 == 2:  # 2/5 * 100 Hz = 40 Hz # openpilot forward delay causes DTC in EGSM, but fixes DTC in PSCM
      #can_sends.append(create_gear_position_message(self.packer, CS.msg_gear_position))
      pass

    new_actuators = actuators.as_builder()
    #new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torque = self.apply_torque_last
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    self.last_lat_active = CC.latActive
    return new_actuators, can_sends
