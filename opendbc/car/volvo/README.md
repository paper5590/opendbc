# Volvo / Polestar dev notes

## Safety tests

`opendbc/safety/tests/test_volvo.py` covers `opendbc/safety/modes/volvo.h` for CMA and SPA:
gas/brake/cruise disengage, standstill, TX allowed buses, relay malfunction, and that the
speed scale / gas thresholds match `carstate.py`.

Not tested yet because `volvo.h` doesn't check them yet: LCA_5 angle limits, RX counters/checksums.

### Setup

```bash
git clone -b master-cma-dev5 https://github.com/paper5590/opendbc   # default branch is master-cma, use dev5
cd opendbc
bash setup.sh       # bash, not source: setup.sh has set -e and will close your terminal on any error
source .venv/bin/activate
```

### Run

```bash
python -m unittest opendbc.safety.tests.test_volvo   # Volvo only, should end with OK (skipped=9)
unittest-parallel -s opendbc/safety/tests -t .       # all cars, run this if you touch common.py or safety.h
bash opendbc/safety/tests/test.sh                    # what CI runs, also requires 100% line coverage
```

### Gotchas

- `GcovrMergeAssertionError ... on multiple lines` from test.sh: stale coverage files from an older
  build or another branch. `git clean -fdX opendbc/safety/tests/libsafety` and rerun.
- If you change the speed scale or a gas threshold, change `volvo.h`, `carstate.py` and
  `test_volvo.py` together. The self-consistency tests fail otherwise.
- New TX message: add it to `VOLVO_TX_MSGS` in `volvo.h` and to the `_TX_*_BUS_ADDRS` lists in the test.
