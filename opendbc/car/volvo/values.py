from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1  # 100 Hz LCA command frequency (controlsd runs at 100 Hz)

  # Torque-based steering parameters
  STEER_MAX = 255                      # Max torque value (8-bit signed in DBC)
  STEER_DELTA_UP = 10                  # Torque increase per refresh
  STEER_DELTA_DOWN = 25                # Torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15          # Allowed driver torque before limiting
  STEER_DRIVER_MULTIPLIER = 3          # Weight driver torque heavily
  STEER_DRIVER_FACTOR = 1              # From DBC
  STEER_ERROR_MAX = 100                # Max delta between torque cmd and torque motor

  # Keep angle limits for reference (not used in torque mode)
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, .2]),
    ([0., 5., 25.], [5., 2., .3]),
  )


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Pilot Assist & Adaptive Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class VolvoPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_cma',
    Bus.main: 'volvo_cma',
    Bus.party: 'volvo_cma',
  })


class CAR(Platforms):
  VOLVO_XC40_RECHARGE = VolvoPlatformConfig(
    [VolvoCarDocs("Volvo XC40 Recharge 2023")],
    CarSpecs(mass=2030, wheelbase=2.702, steerRatio=15.8),
  )


# FW Query configuration for Volvo CMA platform
# FW_QUERY_CONFIG = FwQueryConfig(
#   requests=[
#     Request(
#       [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
#       [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
#       bus=0,
#     ),
#   ],
# )

DBC = CAR.create_dbc_map()
