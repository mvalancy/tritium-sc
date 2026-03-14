# Test Reporting Engine

**Where you are:** `tritium-sc/src/engine/testing/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The testing subsystem provides automated test report generation. It runs pytest across tritium-sc and tritium-lib, collects results, computes quality metrics (test density, untested modules, trends), and generates JSON + HTML reports stored in `data/test_reports/`.

## Key Files

| File | Purpose |
|------|---------|
| `report_generator.py` | Unified test report generator — runs pytest, collects results, outputs JSON + HTML |

## Output

Reports are written to `tritium-sc/data/test_reports/` and include:
- Test pass/fail counts per module
- Test density (tests per source file)
- List of untested modules
- Historical trend data
- HTML dashboard with visual metrics

## Related

- [../../../test.sh](../../../test.sh) — Main test runner script (tiers 1-10)
- [../../../tests/](../../../tests/) — All test files
- [../../../../tritium-lib/tests/](../../../../tritium-lib/tests/) — Lib tests also collected by this reporter
- [../../../docs/TESTING-PHILOSOPHY.md](../../../docs/TESTING-PHILOSOPHY.md) — Testing philosophy and strategy
