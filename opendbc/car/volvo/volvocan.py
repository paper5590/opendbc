def create_lca_steering(packer, lat_active: bool, apply_torque: int):
  """
  Create LCA (Lane Centering Assist) steering command for Volvo CMA platform.
  Uses torque-based control via the LCA_STEER signal.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    apply_torque: Steering torque to apply (-255 to 255)
  """
  if apply_torque < 0: # If torque is negative
    curve_right = 63 # Turn right
  else:
    curve_right = 0 # Turn left

  if not lat_active:
    values = {
      'NEW_SIGNAL_3': 0,
      'LCA_ENABLE_INV': 1,
      'NEW_SIGNAL_1': 3,
      'LCA_STEER_LOOSELY_1': 0,
      'LCA_STEER_ACTIVE_INCOHERENT': 0,
      'LCA_STEER_ACTIVE': 0,
      'NEW_SIGNAL_7': 7,
      'LCA_STEER_LOOSELY_2': 0,
      'NEW_SIGNAL_4': 251,
      'CURVE_RIGHT': 0,
      'NEW_SIGNAL_5': 3,
      'LCA_STEER': 0,
      'NEW_SIGNAL_6': 15,
    }
  else:
    values = {
      'NEW_SIGNAL_3': 0,
      'LCA_ENABLE_INV': 0,
      'NEW_SIGNAL_1': 3,
      'LCA_STEER_LOOSELY_1': 0,
      'LCA_STEER_ACTIVE_INCOHERENT': 1,
      'LCA_STEER_ACTIVE': 3,
      'NEW_SIGNAL_7': 7,
      'LCA_STEER_LOOSELY_2': 0,
      'NEW_SIGNAL_4': 25, # ?
      'CURVE_RIGHT': curve_right,
      'NEW_SIGNAL_5': 3,
      'LCA_STEER': apply_torque,
      'NEW_SIGNAL_6': 15, # ?
    }

  return packer.make_can_msg('LCA', 2, values)
