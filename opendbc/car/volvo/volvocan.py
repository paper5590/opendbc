def volvo_checksum(address: int, sig, d: bytearray) -> int:
  # Volvo CMA platform checksum calculation
  # TODO: Update with actual Volvo checksum algorithm once reverse engineered
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lca_steering(packer, lat_active: bool, apply_torque: int):
  """
  Create LCA (Lane Centering Assist) steering command for Volvo CMA platform.
  Uses torque-based control via the LCA_STEER signal.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    apply_torque: Steering torque to apply (-255 to 255)
  """
  values = {
    'LCA_STEER_ACTIVE_INCOHERENT': 0,
    'LCA_STEER_ACTIVE_PENDING_VERIFICATION': 1 if lat_active else 0,
    'LCA_STEER_LOOSELY_1': 0,
    'LCA_STEER_LOOSELY_2': 0,
    'BITWISE_FLAGS_1': 0,
    'CURVE_RIGHT': 0,
    'BITWISE_FLAGS_2': 0,
    'LCA_STEER': apply_torque,  # Signed 8-bit torque value
    'ASSIST_MAGNITUDE': 0,
  }

  return packer.make_can_msg('LCA', 0, values)
