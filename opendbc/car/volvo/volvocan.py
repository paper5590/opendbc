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
    loosely_1 = 79
    loosely_2 = 115
  else:
    curve_right = 0 # Turn left
    loosely_1 = 115
    loosely_2 = 79
  if abs(apply_torque) < 5: # assume straight road
    loosely_1 = 100
    loosely_2 = 140

  values = {
    'NEW_SIGNAL_3': 0,
    'LCA_ENABLE_INV': 0 if lat_active else 1,
    'NEW_SIGNAL_1': 3,
    'LCA_STEER_LOOSELY_1': loosely_1 if lat_active else 0,
    'LCA_STEER_ACTIVE_INCOHERENT': 1 if lat_active else 0,
    'LCA_STEER_ACTIVE': 3 if lat_active else 0,
    'NEW_SIGNAL_7': 7,
    'LCA_STEER_LOOSELY_2': loosely_2 if lat_active else 0,
    'NEW_SIGNAL_4': 25 if lat_active else 251, # ?
    'CURVE_RIGHT': curve_right,
    'NEW_SIGNAL_5': 3,
    'LCA_STEER': apply_torque if lat_active else 0,
    'NEW_SIGNAL_6': 15, # ?
  }

  return packer.make_can_msg('LCA', 2, values)
