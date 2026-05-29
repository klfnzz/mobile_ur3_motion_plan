# Task Plan: Ranger T_start/T_end Search and Verification

## Goal
Find and validate a `T_start` / `T_end` pair for the ranger scenario such that only those two poses change, the transport translation is greater than 1.2 m, and the unconstrained/contact-constrained pass rates satisfy the latest user target (`unconstrained < 0.75`, `contact > 0.9`); if possible also retain a meaningful contact-vs-unconstrained duration difference, then write the selected poses back to the scene file.

## Current Phase
Phase 7 complete

## Phases
### Phase 1: Requirements & Discovery
- [x] Understand user intent
- [x] Identify constraints and requirements
- [x] Document findings in findings.md
- **Status:** complete

### Phase 2: Search & Candidate Screening
- [x] Run the existing ranger search/evaluation scripts with the requested thresholds
- [x] Inspect accepted candidates and select the best stable pair
- [x] Record candidate metrics and any failures
- **Status:** complete

### Phase 3: Scene Update
- [ ] Update only `T_start` and `T_end` (`T_goal` in the YAML)
- [ ] Preserve all other scene settings
- [ ] Record the selected pose pair in findings/progress
- **Status:** pending

### Phase 4: Verification
- [x] Re-run `check_contact_pass_rate.py` on candidate scenes and on pose overrides
- [ ] Confirm duration delta, translation, and pass-rate targets
- [x] Record outputs in progress.md
- **Status:** complete

### Phase 5: Delivery
- [x] Summarize the chosen pair and verification result
- [x] Call out any residual gap if the thresholds cannot be matched exactly
- [x] Deliver file references and test summary
- **Status:** complete

### Phase 6: Reference-Pose Search
- [x] Evaluate the user's new reference pose pair under `_15`
- [x] Search locally around that pair with larger orientation variation
- [x] If strict pass-rate thresholds cannot be met, prioritize higher static pass-rate while preserving contact improvement
- [x] Record best candidates and give exp3-form YAML blocks
- **Status:** complete

### Phase 7: Connected NTU Outline Generation
- [x] Read `task_plan.md`, `progress.md`, and `findings.md` for existing context
- [x] Connect the T top bar directly between the N and U sides
- [x] Keep only the U lower side as a circular arc and all other primitives as straight lines
- [x] Regenerate contour point, SVG, and PNG outputs with nearly uniform spacing
- [x] Record verification statistics in `progress.md` and `findings.md`
- **Status:** complete

## Key Questions
1. Does the existing search script already match the user's requested filter logic?
2. Which accepted candidate best balances large duration delta with the target pass-rate gap?
3. Can the updated scene be validated end-to-end by `check_contact_pass_rate.py` without changing anything besides `T_start` and `T_goal`?

## Decisions Made
| Decision | Rationale |
|----------|-----------|
| Use existing ranger pass-rate search pipeline first | The repository already has a script that evaluates both transport variants and filters by pass-rate and duration delta. |
| Treat user `T_end` as scene `T_goal` | The ranger scenario YAML uses `T_goal`, not `T_end`. |
| Restrict scene edits to `T_start` and `T_goal` only | This matches the explicit user constraint. |
| Add pose override arguments to `check_contact_pass_rate.py` | This lets us validate arbitrary `T_start`/`T_goal` candidates without hand-editing scenario files each time. |
| Add random-search controls to `search_ranger_pose_pair_passrate_cases.py` | Focused local search around promising seeds is faster and more informative than repeatedly scanning the current scene. |
| Keep the NTU contour parametric | The requested T/N/U connection, straight segments, U circular arc, and uniform point spacing are easier to guarantee with explicit primitives than with image/font tracing. |

## Errors Encountered
| Error | Attempt | Resolution |
|-------|---------|------------|
| `Demo did not export a transport summary.` during focused random search | 1 | Interpreted as an over-aggressive candidate; used seed scenes and smaller local random scales. |
| `I/O operation on closed file.` in focused random search | 1 | Logged as a script-side robustness issue; did not block evaluation of successful samples. |
| Search process ended before writing `exp3_ranger_ref2_pose_expand_s241` outputs | 1 | Recomputed deterministic seed 241 candidate 14, saved it as its own scene, and re-ran exact evaluation. |

## Notes
- Current target thresholds from user request: translation `> 1.2`, duration delta `> 0.5`, unconstrained pass rate about `< 0.6`, contact pass rate about `> 0.9`.
- Latest user target for the current search iteration: translation `> 1.2`, unconstrained pass rate `< 0.75`, contact pass rate `> 0.9`, with larger orientation changes preferred.
- New reference-pose search starts from the active first trajectory in `exp3.scenario_ranger.yaml`, including `base_goal: [2.23, -1.203819, 3.141593]`.
- User reference-pose exact evaluation (`exp3.scenario_ranger_ref2_temp.yaml`) returned `translation=1.640 m`, `rotation=132.35 deg`, `duration_delta=0.000 s`, `unconstrained=100%`, `contact=100%`.
- Best strict reference-local hit is `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml`: `translation=1.706 m`, `rotation=134.75 deg`, `duration_delta=5.826 s`, `unconstrained=64.3%`, `contact=90.7%`.
- Different strict hit from the pasted baseline is `data/scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml`: `translation=1.7348 m`, `rotation=112.11 deg`, `duration_delta=1.5691 s`, `unconstrained=69.2%`, `contact=92.7%`; relative to the pasted baseline, `T_start` changed by `0.1384 m / 8.98 deg` and `T_goal` changed by `0.1279 m / 23.43 deg`.
- More obvious different hit is `data/scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml`: `translation=1.6322 m`, `rotation=132.95 deg`, `duration_delta=2.7050 s`, `unconstrained=54.2%`, `contact=100.0%`; relative to the pasted baseline, `T_start` changed by `0.1773 m / 20.59 deg` and `T_goal` changed by `0.1856 m / 24.05 deg`.
- Need to confirm whether the repository's reported pass rate under the current implemented model lines up with the user's expectation.
- Current blocking finding: under the present `analytical_rigid_bottle_1_20` setup and current code path, the exact re-evaluations of the main seed pairs all returned `duration_delta = 0.0`, `unconstrained_pass_rate = 1.0`, `contact_pass_rate = 1.0`.
- Latest NTU outline output: `521` contour points, length `10.4559 m`, closed distance `0`, adjacent spacing range `0.01905-0.02160 m`; U-bottom world-space arc radius is `0.33968254 m` in both X and Y.
