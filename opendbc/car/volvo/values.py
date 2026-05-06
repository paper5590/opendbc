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
  # Two-state, error-driven model:
  #   - BASELINE: not overriding. Authority = ±BASELINE (lower than max so
  #     openpilot's "rest" state is already easier to push than stock would be).
  #   - LATCHED: driver is overriding. Authority = ±LATCHED (very light EPS
  #     counter-torque so even sustained intentional bias is RSI-friendly).
  #
  # The latch is driven by the *angle error* |steeringAngle − cmd|, not the
  # driver-torque magnitude. Error is naturally clean (PSCM sensor is hardware-
  # filtered) and naturally zero in normal driving — no false-positives from
  # resting-hand torque jitter, no need for LP filters or rising-edge tricks.
  #
  # Latch transitions:
  #   - Latched immediately when |error| > ERROR_LATCH_THRESH (any frame)
  #   - Released only after |error| < ERROR_RELEASE_THRESH for at least
  #     RELEASE_QUIET_FRAMES consecutive frames. Prevents "re-grab" ripple
  #     during a maneuver where the user briefly relaxes mid-transition.
  #
  # Authority slews toward target with asymmetric rate: fast collapse (toward
  # latched), slow rebuild (toward baseline). Both arms (pos / neg) are kept
  # symmetric — no directional logic — so there's nothing to flip on
  # zero-crossings of driver torque.
  #
  # See route_analysis/lca_override_mechanism.md for design history.
  LCA_AUTH_MAX = 614                    # signal saturation cap (clamp on slew)
  LCA_AUTH_BASELINE = 614               # authority when not overriding (matches stock for crosswind/disturbance rejection)
  LCA_AUTH_LATCHED = 50                 # authority while latched (very light counter-torque, lighter than stock's ~120 plateau)
  # Latch triggers on either of two paths (mirrors stock LCA's observed behavior):
  #   (a) STRONG ERROR: |angle - cmd| ≥ ERROR_LATCH_THRESH alone
  #       — catches hard overrides where user has clearly moved the wheel
  #   (b) COMBINED: filtered |drv| ≥ DRV_LATCH_THRESH AND |error| ≥ ERROR_COMBINED_THRESH
  #       — catches gentle co-steering where the user is applying low driver
  #       torque (~drv 2-3) AND the wheel has *also* started drifting off-cmd.
  #       Without (b), initial-push effort at baseline 614 would be stock-level
  #       firm; with (b), the latch fires at stock-equivalent sensitivity
  #       (~drv 1.5 sustained for ~150 ms, ~0.3° wheel deviation).
  # Filtered |drv| uses an LP filter (~100 ms tau) to absorb single-frame
  # noise spikes without lagging real intent.
  LCA_AUTH_ERROR_LATCH_THRESH = 1.0       # deg; (a) strong-error path
  LCA_AUTH_DRV_LATCH_THRESH = 1.5         # filtered-|drv| ; (b) combined-trigger path
  LCA_AUTH_ERROR_COMBINED_THRESH = 0.3    # deg; (b) combined-trigger path
  LCA_AUTH_DRV_LP_ALPHA = 0.1             # LP-filter coefficient on |drv| (~100 ms tau at 100 Hz)
  # Release: BOTH error and filtered-|drv| must be low for QUIET_FRAMES — adds
  # symmetry with the trigger and prevents releasing while the user is still
  # applying torque (even if the wheel has already returned toward cmd).
  LCA_AUTH_ERROR_RELEASE_THRESH = 0.4     # deg; |error| ≤ this counts as quiet
  LCA_AUTH_DRV_RELEASE_THRESH = 1.0       # filtered |drv| ≤ this counts as quiet
  LCA_AUTH_RELEASE_QUIET_FRAMES = 100     # ~1 s of quiet (both signals) before release
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
