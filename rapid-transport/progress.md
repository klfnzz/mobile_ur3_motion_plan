# Progress Log

## Session: 2026-04-18

### Phase 1: Requirements & Discovery
- **Status:** complete
- **Started:** 2026-04-18
- Actions taken:
  - Inspected the ranger scene configuration and current `T_start` / `T_goal`.
  - Located all `T_start`, `T_end`/`T_goal`, `contact`, and `check_contact_pass_rate` references.
  - Read the pass-rate search and verification scripts to confirm the available pipeline.
- Files created/modified:
  - `task_plan.md` (created)
  - `findings.md` (created)
  - `progress.md` (created)

### Phase 2: Search & Candidate Screening
- **Status:** complete
- Actions taken:
  - Ran baseline export-based pass-rate checking on the current active scene.
  - Inspected historical candidate outputs to identify promising seed pairs.
  - Created focused seed inputs and scenario copies for `near_60`, `keep_goal_yaw_m8`, and `soft_a_goal_yaw_m10`.
  - Ran focused random searches around the `near_60` seed with broad and local random scales.
- Files created/modified:
  - `compare_outputs/exp3_ranger_pose_pair_passrate_focus_seeds.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_seed018.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_keep_m8.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_softa_m10.yaml` (created)
  - `task_plan.md` (updated)
  - `progress.md` (updated)

### Phase 4: Verification
- **Status:** complete
- Actions taken:
  - Added `--t-start-json` / `--t-goal-json` support to `scripts/check_contact_pass_rate.py`.
  - Added `--skip-handcrafted-candidates` and `--random-scale` to `scripts/search_ranger_pose_pair_passrate_cases.py`.
  - Further added separate translation/rotation and start/goal random scale controls to `scripts/search_ranger_pose_pair_passrate_cases.py`.
  - Re-validated three exact seed scenes via the export-based search path.
  - Added the missing `_15` profile entry to `data/contacts.yaml`.
  - Re-ran exact seed evaluations and focused local searches under `_15`.
  - Verified both modified scripts compile with `python -m py_compile`.
- Files created/modified:
  - `data/contacts.yaml` (updated)
  - `scripts/check_contact_pass_rate.py` (updated)
  - `scripts/search_ranger_pose_pair_passrate_cases.py` (updated)
  - `compare_outputs/exp3_ranger_seed018_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_keep_m8_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_softa_m10_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_contact15_exact_seed_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_contact15_relaxed_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_contact15_keepm8_local_target_76_90.yaml` (created)
  - `compare_outputs/exp3_ranger_contact15_keepm8_fine_target_751_90.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_keep_m8_local76.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_keep_m8_fine751.yaml` (created)

## Test Results
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Script discovery | `rg -n "T_start|T_end|contact|check_contact_pass_rate"` | Find relevant scene and search/check scripts | Found matching scene and dedicated search/check scripts | pass |
| Script syntax | `python -m py_compile scripts/check_contact_pass_rate.py scripts/search_ranger_pose_pair_passrate_cases.py` | Modified scripts parse successfully | Passed | pass |
| Current active scene export check | `python scripts/check_contact_pass_rate.py --scene scenarios/exp3.scenario_ranger.yaml --samples 120 --export-summary compare_outputs/exp3_ranger_transport1_transport_compare_summary.json --json` | See whether active scene is near boundary | Both variants `pass_rate=1.0`, equal durations | pass |
| Exact seed018 export eval | `search_ranger_pose_pair_passrate_cases.py ... --scene scenarios/exp3.scenario_ranger_seed018.yaml --random-scale 0` | Reproduce a near-boundary historical seed | `duration_delta=0.0`, `u_pass=1.0`, `c_pass=1.0` | fail |
| Exact keep_m8 export eval | `search_ranger_pose_pair_passrate_cases.py ... --scene scenarios/exp3.scenario_ranger_keep_m8.yaml --random-scale 0` | Reproduce large-delta historical seed | `duration_delta=0.0`, `u_pass=1.0`, `c_pass=1.0` | fail |
| Exact softa_m10 export eval | `search_ranger_pose_pair_passrate_cases.py ... --scene scenarios/exp3.scenario_ranger_softa_m10.yaml --random-scale 0` | Reproduce large-delta historical seed | `duration_delta=0.0`, `u_pass=1.0`, `c_pass=1.0` | fail |
| Orientation-heavy relaxed-threshold search | `search_ranger_pose_pair_passrate_cases.py ... --max-unconstrained-pass-rate 0.75 --min-contact-pass-rate 0.9` | Find a larger-rotation candidate under the relaxed thresholds | No verified hit before searches fell into `100%/100%` or planning/export failures | fail |
| `_15` profile load | `search_ranger_pose_pair_passrate_cases.py ... --scene scenarios/exp3.scenario_ranger.yaml` | Load the user-selected `_15` contact profile | Initially failed because `data/contacts.yaml` had no `_15` entry; fixed by adding the entry | pass |
| `_15` exact seed eval | `... --scene scenarios/exp3.scenario_ranger.yaml --seed-yaml compare_outputs/exp3_ranger_pose_pair_passrate_focus_seeds.yaml --random-scale 0` | Re-check promising seeds under `_15` | Recovered meaningful non-zero deltas on `keep_m8`/`soft_a`; `near_60` kept low `u_pass` but no contact trajectory | pass |
| `_15` keep_m8 local search | `... --scene scenarios/exp3.scenario_ranger_keep_m8.yaml --max-unconstrained-pass-rate 0.76 --min-contact-pass-rate 0.9` | Recover a near-hit under `_15` | Found `u_pass=75.9%`, `c_pass=91.7%`, `delta=0.7240s` | pass |
| `_15` keep_m8 fine local search | `... --scene scenarios/exp3.scenario_ranger_keep_m8_local76.yaml --max-unconstrained-pass-rate 0.751 --min-contact-pass-rate 0.9` | Push the near-hit toward the strict target | Found `u_pass=75.0%`, `c_pass=91.4%`, `delta=0.6916s`; still not strictly below 75% | near_miss |
| `_15` seed018 local search | `... --scene scenarios/exp3.scenario_ranger_seed018.yaml --max-unconstrained-pass-rate 0.75 --min-contact-pass-rate 0.9` | Recover contact while keeping low unconstrained pass | `u_pass≈71–74%`, but contact trajectory stayed unavailable | fail |

## Error Log
| Timestamp | Error | Attempt | Resolution |
|-----------|-------|---------|------------|
| 2026-04-18 | `Demo did not export a transport summary.` in focused random search | 1 | Switched from broad random search to smaller local random search and exact seed evaluations. |
| 2026-04-18 | `I/O operation on closed file.` in focused random search | 1 | Logged as non-fatal script issue; continued with exact-evaluation workflow. |
| 2026-04-18 | `Object with id [analytical_rigid_bottle_1_15] not found in table [contact]` | 1 | Added the missing `_15` profile entry to `data/contacts.yaml`. |

### Phase 6: Reference-Pose Search
- **Status:** complete
- Actions taken:
  - Evaluated the user's active first trajectory exactly under `analytical_rigid_bottle_1_15`.
  - Added static pass-rate fields to `scripts/search_ranger_pose_pair_passrate_cases.py`.
  - Added `--skip-scene-seed` so searches can focus only on supplied boundary seeds.
  - Ran a larger-orientation search around the user's reference pair.
  - Captured a low-unconstrained hard boundary seed (`u_pass = 64%`) and searched locally around it.
  - Ran a straight interpolation scan from the user's reference pose to the hard boundary.
- Files created/modified:
  - `scripts/search_ranger_pose_pair_passrate_cases.py` (updated)
  - `compare_outputs/exp3_ranger_reference_active_exact_eval.yaml`
  - `compare_outputs/exp3_ranger_reference_active_exact_eval.json`
  - `compare_outputs/exp3_ranger_reference_pose_search_strict_s20260426.yaml`
  - `compare_outputs/exp3_ranger_reference_pose_search_strict_s20260426.json`
  - `compare_outputs/exp3_ranger_reference_u64_nocontact_seed.yaml`
  - `compare_outputs/exp3_ranger_reference_delta1316_c846_seed.yaml`
  - `compare_outputs/exp3_ranger_reference_u64_local_first4_eval.yaml`
  - `compare_outputs/exp3_ranger_reference_u64_local_first4_eval.json`
  - `compare_outputs/exp3_ranger_reference_delta1316_local_first5_eval.yaml`
  - `compare_outputs/exp3_ranger_reference_delta1316_local_first5_eval.json`
  - `compare_outputs/exp3_ranger_reference_to_hard_interp_seeds.yaml`
  - `compare_outputs/exp3_ranger_reference_to_hard_interp_eval.yaml`
  - `compare_outputs/exp3_ranger_reference_to_hard_interp_eval.json`
- Best strict result:
  - `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml` satisfies `duration_delta > 0.5`, `translation > 1.2`, `unconstrained < 0.75`, and `contact > 0.9`.
  - Exact metrics: `delta = 5.8256 s`, `translation = 1.7056 m`, `unconstrained = 64.3%`, `contact = 90.7%`, `unconstrained_static = 100.0%`, `contact_static = 100.0%`.
- Best static-focused fallback:
  - `delta = 1.5324 s`, `translation = 1.5445 m`, `unconstrained = 64.0%`, `contact = 85.4%`, `unconstrained_static = 100.0%`, `contact_static = 100.0%`.

## 5-Question Reboot Check
| Question | Answer |
|----------|--------|
| Where am I? | Delivery phase, after the `_15` search round and follow-up local searches around the best near-hit. |
| Where am I going? | Report the current best `_15` candidate clearly and avoid overwriting the main scene without a strict verified hit. |
| What's the goal? | Find a ranger `T_start` / `T_goal` pair that meets the latest translation and pass-rate targets while only changing those poses. |
| What have I learned? | `_15` is much closer to the requested regime than `_20/_14`, but the best verified point still sits at `u_pass=75.0%` rather than strictly below 75%. |
| What have I done? | Added `_15` to the contact database, re-ran exact seed checks, and narrowed the search to a high-quality `keep_m8` boundary point. |

## Session: 2026-04-26

### Phase 6: Reference-Pose Search
- **Status:** complete
- Actions taken:
  - Read `planning-with-files` instructions plus existing `task_plan.md`, `findings.md`, and `progress.md`.
  - Created `data/scenarios/exp3.scenario_ranger_ref_user_temp.yaml` from the active ranger scenario and replaced only the active `T_start` / `T_goal` with the user's reference pair.
  - Ran exact evaluation for the user's reference pair: translation `1.6951 m`, rotation `132.57 deg`, `duration_delta=0.0 s`, `unconstrained_pass_rate=100.0%`, `contact_pass_rate=100.0%`.
  - Started an orientation-heavy local search around that pair; before an abnormal process exit after 7 candidates, successful candidates stayed at `100%/100%` with `duration_delta=0.0`, while aggressive poses failed to export a transport summary.
  - Created `data/scenarios/exp3.scenario_ranger_ref2_temp.yaml` from the active ranger scenario and promoted the user's commented second `T_goal` to the active temporary target.
  - Re-ran exact evaluation for `exp3.scenario_ranger_ref2_temp.yaml`: translation `1.640 m`, rotation `132.35 deg`, `duration_delta=0.000 s`, `unconstrained_pass_rate=100.0%`, `contact_pass_rate=100.0%`.
  - Ran broader orientation-heavy search from the reference pair; deterministic seed 241 candidate 14 reached `duration_delta=6.594 s`, `unconstrained_pass_rate=64.3%`, but only `contact_pass_rate=85.1%`.
  - Saved seed 241 candidate 14 as `data/scenarios/exp3.scenario_ranger_ref2_s241_i014.yaml` and exactly re-evaluated it.
  - Ran a smaller local search from that boundary point with seed 314; candidate 5 satisfied all strict filters.
  - Saved the final strict hit as `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml` and exactly re-evaluated it: translation `1.706 m`, rotation `134.75 deg`, `duration_delta=5.826 s`, `unconstrained_pass_rate=64.3%`, `contact_pass_rate=90.7%`.
- Files created/modified:
  - `data/scenarios/exp3.scenario_ranger_ref_user_temp.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_ref2_temp.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_ref2_s241_i014.yaml` (created)
  - `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml` (created)
  - `compare_outputs/exp3_ranger_ref_user_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_ref_user_exact_eval.json` (created)
  - `compare_outputs/exp3_ranger_ref2_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_ref2_exact_eval.json` (created)
  - `compare_outputs/exp3_ranger_ref2_s241_i014_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_ref2_s241_i014_exact_eval.json` (created)
  - `compare_outputs/exp3_ranger_ref2_i014_local_s314.yaml` (created)
  - `compare_outputs/exp3_ranger_ref2_i014_local_s314.json` (created)
  - `compare_outputs/exp3_ranger_ref2_strict_hit_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_ref2_strict_hit_exact_eval.json` (created)
  - `task_plan.md` (updated)
  - `findings.md` (updated)
  - `progress.md` (updated)

## Test Results Added 2026-04-26
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| User reference exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_ref_user_temp.yaml --random-scale 0` | Establish baseline metrics around the requested pose pair | `u_pass=100.0%`, `c_pass=100.0%`, `delta=0.0s`, `translation=1.6951m` | pass |
| User second-reference exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_ref2_temp.yaml --random-scale 0` | Establish baseline metrics for the requested commented pair | `u_pass=100.0%`, `c_pass=100.0%`, `delta=0.0s`, `translation=1.640m` | pass |
| Seed 241 boundary exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_ref2_s241_i014.yaml --random-scale 0` | Verify low-unconstrained boundary candidate | `u_pass=64.3%`, `c_pass=85.1%`, `delta=6.594s`, `translation=1.689m` | near_miss |
| Seed 314 strict hit exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml --random-scale 0` | Verify all strict filters | `u_pass=64.3%`, `c_pass=90.7%`, `delta=5.826s`, `translation=1.706m` | pass |

## Error Log Added 2026-04-26
| Timestamp | Error | Attempt | Resolution |
|-----------|-------|---------|------------|
| 2026-04-26 | Orientation-heavy search exited abnormally after 7 candidates before writing output | 1 | Switch to smaller search batches so each batch can write result files. |
| 2026-04-26 | Seed 241 broader search exited before writing output after candidate 14 | 1 | Recomputed candidate 14 from the deterministic RNG seed, saved it as a standalone scene, and exactly re-evaluated it. |

## Session: 2026-04-27

### Different Strict-Hit Search
- **Status:** complete
- Actions taken:
  - Treated the user's pasted `T_start` / `T_goal` as the previous strict-hit baseline to avoid.
  - Ran a randomized search from `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml` with minimum start/goal position-change filters.
  - Found a near miss with `u_pass=65.4%`, `contact_pass=89.8%`, `duration_delta=2.277s`, then searched locally around it.
  - Found and saved a second strict hit as `data/scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml`.
  - Exact re-evaluation confirmed `translation=1.7348m`, `duration_delta=1.5691s`, `unconstrained_pass_rate=69.2%`, `contact_pass_rate=92.7%`.
  - Measured difference from the pasted baseline: `T_start` position/rotation deltas `0.1384m / 8.98deg`, `T_goal` position/rotation deltas `0.1279m / 23.43deg`.
- Files created/modified:
  - `data/scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_s42710.yaml` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_s42710.json` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_near_seed.yaml` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_local_s42711.yaml` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_local_s42711.json` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_hit_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_diff_from_strict_hit_exact_eval.json` (created)
  - `findings.md` (updated)
  - `progress.md` (updated)

## Test Results Added 2026-04-27
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Different strict-hit exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml --random-scale 0` | Verify a candidate different from the pasted baseline satisfies all filters | `u_pass=69.2%`, `contact=92.7%`, `delta=1.5691s`, `translation=1.7348m` | pass |

### More Obvious Difference Search
- **Status:** complete
- Actions taken:
  - Scanned existing accepted candidates and confirmed only the previous strict hit and the 13cm-different hit were available.
  - Tried hard endpoint-position thresholds (`>=25cm`, then `>=20cm`); evaluable candidates were mostly `100%/100%` with zero duration delta, while boundary candidates tended to fail export.
  - Tried extrapolating along the 13cm-different hit direction; extrapolated candidates up to `~30cm` became fully stable (`100%/100%`, `delta=0`).
  - Ran a larger orientation-difference search with minimum `10cm` endpoint-position changes.
  - Reconstructed seed `42740` candidate 47 after the long search hung, saved it as `data/scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml`, and exact re-evaluated it.
- Files created/modified:
  - `data/scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml` (created)
  - `compare_outputs/exp3_ranger_very_diff_s42720.yaml` (created)
  - `compare_outputs/exp3_ranger_very_diff_s42720.json` (created)
  - `compare_outputs/exp3_ranger_very_diff_s42721.yaml` (created)
  - `compare_outputs/exp3_ranger_very_diff_s42721.json` (created)
  - `compare_outputs/exp3_ranger_extrapolate_from_diff_seeds.yaml` (created)
  - `compare_outputs/exp3_ranger_extrapolate_from_diff_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_extrapolate_from_diff_eval.json` (created)
  - `compare_outputs/exp3_ranger_far_stable_seeds.yaml` (created)
  - `compare_outputs/exp3_ranger_far_stable_local_s42731.yaml` (created)
  - `compare_outputs/exp3_ranger_far_stable_local_s42731.json` (created)
  - `compare_outputs/exp3_ranger_big_orientation_diff_s42740_i047_seed.yaml` (created)
  - `compare_outputs/exp3_ranger_big_orientation_diff_hit_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_big_orientation_diff_hit_exact_eval.json` (created)
  - `findings.md` (updated)
  - `progress.md` (updated)

## Test Results Added 2026-04-27 More Obvious Search
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Very different strict-hit exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml --random-scale 0` | Verify a visibly different candidate satisfies all filters | `u_pass=54.2%`, `contact=100.0%`, `delta=2.7050s`, `translation=1.6322m`; relative to pasted baseline: start `0.1773m / 20.59deg`, goal `0.1856m / 24.05deg` | pass |

### Y-Reference Search
- **Status:** paused for report
- Actions taken:
  - Used the active `data/scenarios/exp3.scenario_ranger.yaml` as the new Y-reference scene.
  - Exact baseline under the current scene contact profile (`analytical_rigid_bottle_1_10_new`) was fully stable: `translation=1.951m`, `duration_delta=0.0s`, `unconstrained=100.0%`, `contact=100.0%`.
  - Ran a broad pose search around the Y-reference. Low-unconstrained cases appeared (`u_pass≈65.8%` and `71.1%`) but contact trajectory export was unavailable for those cases.
  - Ran local searches around reconstructed boundary seeds (`s42750_i002`, `s42750_i016`, `s42750_i022`). No strict hit was found.
  - Ran interpolation between contact-valid and low-unconstrained seeds. The closest overlap was `delta=3.581s`, `u_pass=78.9%`, `contact=89.2%`, so it missed both strict thresholds.
  - Created a temporary `_15` scene for comparison only, then stopped that search when the user asked for a result summary and not to modify contact files.
- Files created/modified:
  - `compare_outputs/exp3_ranger_yref_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_exact_eval.json` (created)
  - `compare_outputs/exp3_ranger_yref_boundary_seeds.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_boundary_local_s42751.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_boundary_local_s42751.json` (created)
  - `compare_outputs/exp3_ranger_yref_i016_seed.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_i016_medium_s42752.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_i016_medium_s42752.json` (created)
  - `compare_outputs/exp3_ranger_yref_i016_interp_seeds.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_i016_interp_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_i016_interp_eval.json` (created)
  - `data/scenarios/exp3.scenario_ranger_yref_contact15_temp.yaml` (temporary comparison scene created)
  - `compare_outputs/exp3_ranger_yref_contact15_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_yref_contact15_exact_eval.json` (created)
  - `progress.md` (updated)

## Test Results Added 2026-04-27 Y-Reference
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Y-reference exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger.yaml --random-scale 0` | Establish baseline metrics | `u_pass=100.0%`, `contact=100.0%`, `delta=0.0s`, `translation=1.951m` | pass |
| Y-reference boundary local search | `search_ranger_pose_pair_passrate_cases.py --seed-yaml compare_outputs/exp3_ranger_yref_boundary_seeds.yaml` | Find strict overlap near low-u/contact-valid boundary | No accepted candidates; low-u cases had `contact=n/a`, contact-valid cases had `u_pass>=92.1%` or `100%` | no_hit |
| Y-reference i016 medium search | `search_ranger_pose_pair_passrate_cases.py --seed-yaml compare_outputs/exp3_ranger_yref_i016_seed.yaml` | Push contact-valid side toward `u<75%` | No accepted candidates; best contact-valid dynamic candidates had `u_pass=81.6-84.2%`, `contact=100%`, `delta>1.4s`; low-u candidates had `contact=n/a` | no_hit |
| Y-reference interpolation scan | `search_ranger_pose_pair_passrate_cases.py --seed-yaml compare_outputs/exp3_ranger_yref_i016_interp_seeds.yaml` | Check for strict overlap between contact-valid and low-u seeds | No accepted candidates; closest point was `delta=3.581s`, `u_pass=78.9%`, `contact=89.2%` | near_miss |

### Userbase3 Search
- **Status:** complete
- Actions taken:
  - Used the active `data/scenarios/exp3.scenario_ranger.yaml` as the new userbase3 reference scene, preserving its existing contact profile and not modifying any contact files.
  - Exact baseline failed to export a transport summary, so the exact reference pair itself was not usable for comparison.
  - Ran a broad local search around the reference. Most samples failed to export; the best near miss was `u_pass=78.1%`, `contact=97.4%`, `duration_delta=0.660s`.
  - Searched locally around that near miss and found 3 strict hits.
  - Saved the best strict hit as `data/scenarios/exp3.scenario_ranger_userbase3_strict_hit.yaml`.
  - Exact re-evaluation confirmed `translation=2.0224m`, `duration_delta=1.2334s`, `unconstrained_pass_rate=72.7%`, `contact_pass_rate=97.8%`.
- Files created/modified:
  - `compare_outputs/exp3_ranger_userbase3_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_exact_eval.json` (created)
  - `compare_outputs/exp3_ranger_userbase3_search_s42770.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_search_s42770.json` (created)
  - `compare_outputs/exp3_ranger_userbase3_near_seed.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_near_local_s42771.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_near_local_s42771.json` (created)
  - `data/scenarios/exp3.scenario_ranger_userbase3_strict_hit.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_strict_hit_exact_eval.yaml` (created)
  - `compare_outputs/exp3_ranger_userbase3_strict_hit_exact_eval.json` (created)
  - `progress.md` (updated)

## Test Results Added 2026-04-27 Userbase3
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Userbase3 exact baseline | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger.yaml --random-scale 0` | Establish reference metrics | Failed to export transport summary | fail |
| Userbase3 broad search | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger.yaml --seed 42770` | Find near-boundary candidates | No accepted candidates; best near miss `u=78.1%`, `contact=97.4%`, `delta=0.660s` | near_miss |
| Userbase3 strict hit exact eval | `search_ranger_pose_pair_passrate_cases.py --scene scenarios/exp3.scenario_ranger_userbase3_strict_hit.yaml --random-scale 0` | Verify strict filters | `u_pass=72.7%`, `contact=97.8%`, `delta=1.2334s`, `translation=2.0224m` | pass |

## Session: 2026-05-29

### Connected NTU Outline Update
- **Status:** complete
- Actions taken:
  - Read `task_plan.md`, `progress.md`, and `findings.md` for current repository context.
  - Updated `reproductions/ntu_2024_icra_mobocontp/parametric_ntu_outline.py` so the T top bar connects directly from the N right side to the U right side.
  - Changed the U lower primitive to a true circular arc in the final 3.0 x 0.75 m world footprint; all other primitives remain straight lines.
  - Unified the line and arc sampling target step so neighboring contour points stay nearly evenly spaced.
  - Regenerated candidate CSV/SVG/PNG outputs.
  - Updated current README wording for the single-layer NTU contour.
- Files created/modified:
  - `reproductions/ntu_2024_icra_mobocontp/parametric_ntu_outline.py` (updated)
  - `reproductions/ntu_2024_icra_mobocontp/tools/generate_ntu_outline_candidate.py` (updated)
  - `reproductions/ntu_2024_icra_mobocontp/README.md` (updated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_points.csv` (regenerated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_parametric_program.csv` (regenerated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_preview.svg` (regenerated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_numbered.svg` (regenerated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_preview.png` (regenerated)
  - `reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_numbered.png` (regenerated)
  - `findings.md` (updated)
  - `progress.md` (updated)

## Test Results Added 2026-05-29 NTU Outline
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Script syntax | `python -m py_compile reproductions/ntu_2024_icra_mobocontp/parametric_ntu_outline.py reproductions/ntu_2024_icra_mobocontp/tools/generate_ntu_outline_candidate.py` | Updated scripts parse successfully | Passed | pass |
| Candidate generation | `python reproductions/ntu_2024_icra_mobocontp/tools/generate_ntu_outline_candidate.py` | Regenerate connected NTU contour files | `521` points, length `10.4559 m`, closed distance `0` | pass |
| Spacing check | CSV segment length scan | Neighbor intervals are close and no large jumps appear | `min=0.01905 m`, `max=0.02160 m`, `mean=0.02011 m`; no segments `<0.018 m` or `>0.023 m` | pass |
