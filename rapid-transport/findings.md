# Findings & Decisions

## Requirements
- Search for a ranger `T_start` / `T_end` pair in `data/scenarios/exp3.scenario_ranger.yaml`.
- Only modify `T_start` and `T_end` in the scene; for this repository that means `T_start` and `T_goal`.
- Prefer a pair where translation distance is greater than 1.2 m.
- Prefer a pair where `duration(contact) - duration(unconstrained) > 0.5 s`.
- Add `check_contact_pass_rate.py` into the validation flow.
- Target trajectory behavior: unconstrained pass rate around or below 60%, contact-constrained pass rate around or above 90%.
- Updated target trajectory behavior for the latest search round: unconstrained pass rate below 75%, contact-constrained pass rate above 90%, with larger pose/orientation changes preferred.

## Research Findings
- `data/scenarios/exp3.scenario_ranger.yaml` already exports both transport variants because `transport_constraint_mode: both` is enabled.
- `scripts/check_contact_pass_rate.py` can evaluate exported `contact` and `unconstrained` trajectories and report sampled `Fw <= g` pass rates.
- `scripts/search_ranger_pose_pair_passrate_cases.py` already implements the exact high-level filter chain needed here:
  - translation distance threshold
  - duration delta threshold
  - unconstrained pass-rate upper bound
  - contact-constrained pass-rate lower bound
- The pass-rate search script writes temporary scenes, changes only `T_start` and `T_goal`, runs the full ranger demo, then reuses exports for the pass-rate check.
- The current scene uses `contact_profile: analytical_rigid_bottle_1_20`.
- Under the current code state, three exact seed scenes re-evaluated through the export-based search pipeline all produced:
  - `duration_delta = 0.0`
  - `unconstrained_pass_rate = 1.0`
  - `contact_pass_rate = 1.0`
- Focused random search around the near-60 historical seed produced mostly two outcomes:
  - successful trajectories with `delta = 0.0` and `100% / 100%` pass rates
  - failures where the demo could not export a transport summary
- This means the present `_20` contact profile plus current planning stack is not reproducing the older near-boundary behavior stored in some historical output files.
- Additional orientation-focused searches around `soft_a_goal_yaw_m10` and `keep_goal_yaw_m8` also did not produce a verified `< 0.75 / > 0.9` hit under `_20` before either:
  - returning `duration_delta = 0.0`, `u_pass = 1.0`, `c_pass = 1.0`, or
  - failing to export / hanging in planning for more aggressive orientation-only cases.
- After switching the main scenario to `_15`, the `.npz` file already existed in `data/contact_data`, but the profile entry was missing from `data/contacts.yaml`; adding that YAML entry unblocked loading.
- Under `_15`, the old relaxed candidates started to recover meaningful separation again:
  - `keep_goal_yaw_m8`: `delta = 0.524 s`, `u_pass = 79.3%`, `c_pass = 85.3%`
  - `soft_a_goal_yaw_m10`: `delta = 0.460 s`, `u_pass = 78.6%`, `c_pass = 84.8%`
  - `near_60`: `u_pass = 71.0%` but contact retiming/export remained unavailable
- The best verified near-hit under `_15` is centered on the `keep_m8` branch:
  - translation `1.6354 m`
  - duration delta `0.6916 s`
  - unconstrained pass rate `75.0%`
  - contact pass rate `91.4%`
- A second nearby `_15` near-hit is:
  - translation `1.6450 m`
  - duration delta `0.7240 s`
  - unconstrained pass rate `75.9%`
  - contact pass rate `91.7%`
- Repeated local searches around that `_15 keep_m8` boundary point with multiple random seeds kept landing at `u_pass = 75.0%` or `75.9%`, while `contact` stayed above 90% in the good region.
- Local searches around the `_15 near_60` branch consistently preserved `u_pass ≈ 71–74%`, but the contact-constrained transport still failed to retime/export, so that branch did not produce a valid final candidate.
- New reference-pose search from the user's active first trajectory initially evaluated as fully stable:
  - translation `1.557 m`
  - rotation `82.16 deg`
  - duration delta `0.000 s`
  - unconstrained/contact pass rate `100% / 100%`
- Larger orientation search around that reference found a hard boundary point with `u_pass = 64%`, `u_static = 84%`, but no contact-constrained export; this confirmed the direction can make the unconstrained trajectory fail but was too aggressive for a usable paired comparison.
- Local recovery around that boundary produced a strong fallback with `delta = 1.316 s`, `u_pass = 64.0%`, `c_pass = 84.6%`, `u_static = 92.0%`, `c_static = 100.0%`.
- A second local recovery improved the static margins further and gave the best static-focused fallback:
  - translation `1.5445 m`
  - rotation `36.89 deg`
  - duration delta `1.5324 s`
  - unconstrained pass rate `64.0%`
  - contact pass rate `85.4%`
  - unconstrained/contact static pass rate `100.0% / 100.0%`
- Straight interpolation from the user's reference pose to the hard boundary showed no strict `u < 75%` and `c > 90%` overlap. The clean static-maximized region had `u_static = c_static = 100%`, but contact dynamic pass rate stayed around `81-84%` once the unconstrained pass rate dropped below `75%`.
- A separate expanded reference branch produced a strict verified hit:
  - scene: `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml`
  - exact eval: `compare_outputs/exp3_ranger_ref2_strict_hit_exact_eval.json`
  - translation `1.7056 m`
  - rotation `134.75 deg`
  - duration delta `5.8256 s`
  - unconstrained pass rate `64.3%`
  - contact pass rate `90.7%`
  - unconstrained/contact static pass rate `100.0% / 100.0%`
- The user's commented second trajectory, promoted to an active temporary scene in `data/scenarios/exp3.scenario_ranger_ref2_temp.yaml`, is feasible but not near the desired contact/unconstrained split by itself:
  - translation `1.640 m`
  - rotation `132.35 deg`
  - duration delta `0.000 s`
  - unconstrained pass rate `100.0%`
  - contact pass rate `100.0%`
- A broader orientation-heavy search around that reference found a useful low-unconstrained boundary point (`seed 241`, random candidate 14):
  - translation `1.689 m`
  - rotation `136.69 deg`
  - duration delta `6.594 s`
  - unconstrained pass rate `64.3%`
  - contact pass rate `85.1%`
- A smaller local search around the seed-241 candidate found a strict hit in `data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml`:
  - translation `1.706 m`
  - rotation `134.75 deg`
  - duration delta `5.826 s`
  - unconstrained pass rate `64.3%`
  - contact pass rate `90.7%`
  - unconstrained/static pass rates both `100.0%` on the final exact re-evaluation
- Follow-up search for a candidate substantially different from `exp3.scenario_ranger_ref2_strict_hit.yaml` found a second strict hit:
  - scene: `data/scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml`
  - exact eval: `compare_outputs/exp3_ranger_diff_from_strict_hit_exact_eval.yaml`
  - translation `1.7348 m`
  - rotation `112.11 deg`
  - duration delta `1.5691 s`
  - unconstrained pass rate `69.2%`
  - contact pass rate `92.7%`
  - relative to the previous strict hit: `T_start` position delta `0.1384 m`, `T_start` rotation delta `8.98 deg`, `T_goal` position delta `0.1279 m`, `T_goal` rotation delta `23.43 deg`
- A stronger "obviously different" orientation-heavy search found a better strict hit:
  - scene: `data/scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml`
  - exact eval: `compare_outputs/exp3_ranger_big_orientation_diff_hit_exact_eval.yaml`
  - translation `1.6322 m`
  - rotation `132.95 deg`
  - duration delta `2.7050 s`
  - unconstrained pass rate `54.2%`
  - contact pass rate `100.0%`
  - relative to the previous strict hit: `T_start` position delta `0.1773 m`, `T_start` rotation delta `20.59 deg`, `T_goal` position delta `0.1856 m`, `T_goal` rotation delta `24.05 deg`
  - More aggressive `>=20 cm` endpoint-position searches mostly produced either `100%/100%` stable trajectories or planning/export failures, so this hit is the best verified tradeoff found in this round.
- The connected NTU outline generator now matches the requested contour behavior:
  - `T` top bar connects from the `N` right/top side to the `U` right/top side.
  - Only the bottom of `U` uses an arc; it is circular after mapping to the final `3.0 x 0.75 m` footprint.
  - All other contour primitives are straight lines.
  - Regenerated candidate output has `521` points, total length `10.4559 m`, and closed distance `0`.
  - Adjacent point spacing is nearly uniform: min `0.01905 m`, max `0.02160 m`, mean `0.02011 m`.

## Technical Decisions
| Decision | Rationale |
|----------|-----------|
| Use `search_ranger_pose_pair_passrate_cases.py` instead of adding new search logic first | It already matches the requested screening logic and reduces risky code changes. |
| Validate final scene with `check_contact_pass_rate.py` after editing the YAML | This gives an end-to-end confirmation on the exact selected scene. |
| Extend `check_contact_pass_rate.py` with pose override arguments | Faster candidate validation without repeatedly editing scenario files. |
| Extend `search_ranger_pose_pair_passrate_cases.py` with `--skip-handcrafted-candidates` and `--random-scale` | Lets us do targeted local random search around promising seeds. |
| Further extend `search_ranger_pose_pair_passrate_cases.py` with separate translation/rotation and start/goal random scales | Lets us follow the user's suggestion and bias the search toward larger orientation changes while keeping position nearly fixed. |
| Add the missing `_15` entry to `data/contacts.yaml` | Required so the user's chosen contact profile can actually load from the repository database. |
| Add static pass-rate fields to the ranger search output | Needed for the user's fallback request to prioritize larger `static_pass_rate` when the strict dynamic contact threshold cannot be met. |
| Add `--skip-scene-seed` to the ranger search script | Avoids spending repeated evaluations on the known-stable active scene when searching around a supplied boundary seed. |
| Keep the NTU contour parametric instead of tracing an image/font | The user asked for the target trajectory as connected `T/N/U` contour points with controlled straight/arc geometry and spacing. |

## Issues Encountered
| Issue | Resolution |
|-------|------------|
| User asked for `T_end`, but scene schema uses `T_goal` | Treat `T_end` as `T_goal` and keep that mapping explicit. |
| Historical result files suggested near-boundary cases, but exact re-evaluation under the current code/profile did not reproduce them | Recorded the mismatch and avoided editing the main scene without a verified candidate. |
| `analytical_rigid_bottle_1_15` existed as an `.npz` but not as a YAML profile entry | Added the missing profile entry so the planner/checker could load it. |

## Resources
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger.yaml`
- `/workspaces/Openrave/rapid-transport/scripts/search_ranger_pose_pair_passrate_cases.py`
- `/workspaces/Openrave/rapid-transport/scripts/check_contact_pass_rate.py`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_seed018_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_keep_m8_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_softa_m10_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_contact15_exact_seed_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_contact15_relaxed_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_contact15_keepm8_local_target_76_90.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_contact15_keepm8_fine_target_751_90.yaml`
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger_ref2_temp.yaml`
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger_ref2_s241_i014.yaml`
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger_ref2_strict_hit.yaml`
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger_diff_from_strict_hit.yaml`
- `/workspaces/Openrave/rapid-transport/data/scenarios/exp3.scenario_ranger_big_orientation_diff_hit.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_ref2_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_ref2_s241_i014_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_ref2_i014_local_s314.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_ref2_strict_hit_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_diff_from_strict_hit_exact_eval.yaml`
- `/workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_big_orientation_diff_hit_exact_eval.yaml`
- `/workspaces/复现/rapid-transport/reproductions/ntu_2024_icra_mobocontp/parametric_ntu_outline.py`
- `/workspaces/复现/rapid-transport/reproductions/ntu_2024_icra_mobocontp/tools/generate_ntu_outline_candidate.py`
- `/workspaces/复现/rapid-transport/reproductions/ntu_2024_icra_mobocontp/outputs/ntu_outline_candidate_points.csv`

## Visual/Browser Findings
- No browser or image inspection used.
