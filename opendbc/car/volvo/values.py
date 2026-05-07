from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1  # 100 Hz LCA command frequency (controlsd runs at 100 Hz)

  # Max absolute error between commanded and actual steering angle. Bounds EPS
  # fight torque during driver override: driver torque needed scales ~linearly
  # with |cmd - actual|, so capping the gap caps the driver effort. 6° matches
  # stock Pilot Assist's observed steady-state override gap.
  # See docs/plans/2026-04-21-override-softening-investigation.md.
  #MAX_ERR_DEG = 6.0
  MAX_ERR_DEG = 3.0

  # LCA torque-authority envelope (signals LCA_STEER_LOOSELY / _INV).
  # Two-state model with combined drv+err trigger and conservative release:
  #
  # BASELINE: not overriding. Authority = ±BASELINE (full, matches stock for
  #           proper crosswind/disturbance rejection).
  # LATCHED:  driver overriding. Authority = ±LATCHED (high enough for EPS to
  #           still drive the wheel back to cmd after release; low enough
  #           that the override feel is comparable to stock's ~120 plateau).
  #
  # Trigger (latch fires on any single frame where ANY of these is true):
  #   - |drv| >= DRV_LATCH_THRESH (5)   — direct driver-torque trigger
  #   - |err| >= ERROR_LATCH_THRESH (1°) — wheel deflected off cmd; catches
  #     mild-pressure cases (e.g. drv~3 mid-rebuild causing wheel to drift)
  #
  # Release (latch releases only after BOTH below hold continuously for
  # RELEASE_QUIET_FRAMES frames; |err| is the primary release signal,
  # |drv| condition is a sanity check that the user isn't still pushing):
  #   - |drv| < DRV_RELEASE_THRESH (4)
  #   - |err| < ERROR_RELEASE_THRESH (0.4°)
  #
  # Both arms (pos / neg) kept symmetric — no directional logic — so nothing
  # flips on zero-crossings of driver torque. Authority slews fast toward
  # latched (collapse), slow toward baseline (rebuild). Trigger conditions
  # are checked every frame including during rebuild, so any new override
  # interrupts the rebuild and snaps authority back to LATCHED.
  #
  # See route_analysis/lca_override_mechanism.md for design history.
  LCA_AUTH_MAX = 614                      # signal saturation cap (clamp on slew)
  LCA_AUTH_BASELINE = 614                 # authority when not overriding (matches stock; full crosswind rejection)
  LCA_AUTH_LATCHED = 130                  # authority while latched (~stock plateau; enough for EPS to still drive)
  LCA_AUTH_DRV_LATCH_THRESH = 5           # raw |drv| ≥ this triggers latch
  LCA_AUTH_ERROR_LATCH_THRESH = 1.0       # deg; |err| ≥ this also triggers latch (catches re-engagement during rebuild)
  LCA_AUTH_DRV_RELEASE_THRESH = 4         # raw |drv| < this counts as quiet for release (1-unit hysteresis vs trigger)
  LCA_AUTH_ERROR_RELEASE_THRESH = 0.4     # deg; |err| < this counts as quiet for release
  LCA_AUTH_RELEASE_QUIET_FRAMES = 100     # ~1 s of both-quiet before release
  LCA_AUTH_REBUILD_RATE = 230             # counts/s (slow rebuild — release direction)
  LCA_AUTH_COLLAPSE_RATE = 2500           # counts/s (fast collapse — latch direction)

  # Angle limits for rate limiting
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    540, # deg - 1.5 turns to lock
    ([0., 5., 25.], [2.5, 1.5, .2]),  # rate up limits at different speeds
    ([0., 5., 25.], [5., 2., .3]),    # rate down limits at different speeds
  )


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Pilot Assist & Adaptive Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class VolvoCMAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.main: 'volvo_mid_1',
    Bus.party: 'volvo_mid_1',
    Bus.pt: 'volvo_front_1_cma',
  })

@dataclass
class VolvoSPAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.main: 'volvo_mid_1',
    Bus.party: 'volvo_mid_1',
    Bus.pt: 'volvo_front_1_spa',
  })


class CAR(Platforms):
  VOLVO_XC40_RECHARGE = VolvoCMAPlatformConfig(
    [VolvoCarDocs("Volvo XC40 Recharge 2021-2023")],
    CarSpecs(
      mass=2170,
      wheelbase=2.702,
      steerRatio=15.8,
      centerToFrontRatio=0.52,
    ),
  )

  VOLVO_S60_RECHARGE = VolvoSPAPlatformConfig(
    [VolvoCarDocs("Volvo S60 Recharge 2024")],
    CarSpecs(
      mass=2020,
      wheelbase=2.872,
      steerRatio=16.2,
      centerToFrontRatio=0.516,
    ),
  )

  # Polestar 2 is technically CMA, but appears to use SPA DBC for CAN 1 bus
  POLESTAR_2 = VolvoSPAPlatformConfig(
    [VolvoCarDocs("Polestar 2 2020+")],
    CarSpecs(
      mass=2123,
      wheelbase=2.735,
      steerRatio=15.8,
      centerToFrontRatio=0.52,
    ),
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
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[]
)

DBC = CAR.create_dbc_map()
