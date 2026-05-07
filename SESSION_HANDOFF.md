# Session Handoff

## Repository State

- Local worktree: `D:\2026 projects\Autonomous 3d mapping swarm-v1`
- Active branch: `swarm-v1`
- Remote: `origin https://github.com/Abhijeet-Singh-MUN/autonomous-3d-mapping.git`
- Baseline branch: `main` keeps the posted single-drone room project preserved.
- Current branch direction: terrain/swarm simulator for adaptive 3D mapping and point-cloud dataset capture.
- Active swarm algorithm model: `greybox-policy-v1` / `greybox-policy-v1.0`.
- Research direction summary: `RESEARCH_ROADMAP.md`.

## Run Commands

```bash
npm.cmd install
npm.cmd run dev
npm.cmd run build
```

If the global Node install is still `v21.4.0`, prepend the local portable Node runtime before running Vite:

```powershell
$env:Path = "$PWD\.tools\node-v22.12.0-win-x64;$env:Path"
npm.cmd run dev -- --host 127.0.0.1 --port 5183 --strictPort
```

The local Vite URL is usually:

```text
http://127.0.0.1:5180/
```

Current portable-node dev URL:

```text
http://127.0.0.1:5183/
```

If Vite chooses another port, use the URL printed by `npm.cmd run dev`.

## Known Warnings

- PowerShell may try to open `npm.ps1`; use `npm.cmd` on Windows.
- Vite may warn that the minified bundle is larger than 500 kB. That is expected for the current Three.js prototype.
- If Node is outside Vite's preferred range, build/dev may show an engine warning; the latest local builds still pass.

## Current Defaults

- Mission mode: Swarm V1 preview.
- Environment: mountain-valley terrain sandbox.
- Swarm drones: `24`.
- K-nearest comms links: `6`.
- Link dropout: `0`.
- Scan density: `Economy`.
- Performance budget: `Capture`.
- Visible point cap path supports up to `1,200,000` points in custom mode.
- AOI color cue: red local Z-depth gradient around object/AOI scale.

## Good Enough Visual State

- Drones launch from a distributed square ground lattice, then take off into formation behavior.
- Swarm Behavior Kernel V1 is active: roles rebalance among scouts, mappers, relays, and verifiers based on communication health, frontier pressure, terrain class, and AOI presence.
- `src/swarm/behavior-profile.js` is now the first central parameter registry for behavior weights, role shares, task scoring, movement, sensing, network, relay, formation, and objective profiles.
- The active behavior model is a grey-box policy-coordinate hierarchy: `coverage_area`, `aoi_detail`, `risk_safety`, and `resource_efficiency` derive the lower-level behavior profile through documented coupling coefficients.
- The four policy coordinates are exposed as mission-control sliders; values are normalized before the active behavior profile is derived.
- Runtime nudge profiles use one uniform base cap across all policy coordinates and are calibrated before optimizer work: very low, low, current, strong, and very strong.
- `Run Policy Batch` cycles through four starter policy presets across five nudge profiles on the current scenario, using the configured batch seconds value, and saves each preset/profile as a normal telemetry run with experiment metadata. Telemetry groups policy-batch runs by batch id.
- `Policy batch mode` selects either the 20-run nudge-calibration sweep or a 4-run Pareto-only comparison that keeps the currently selected nudge profile fixed.
- The 20-run policy/nudge batch is a nudge-calibration phase. Once a default nudge profile is chosen, later batches should mostly vary the four Pareto policy coordinates.
- Telemetry batch headers summarize valid/invalid/cancelled counts, average fixed score/loss, policy-aligned score/loss, role entropy, best run, lowest loss, and per-preset/per-nudge averages for quick in-sim checks before external analysis.
- `Dataset workspace` tags telemetry runs and export filenames. Use a new workspace name for new calibration/model/data campaigns; old IndexedDB runs remain available.
- Saved workspace names are remembered locally, and Telemetry can filter by active workspace, all workspaces, legacy/untagged runs, or a named workspace.
- Workspace clearing deletes IndexedDB telemetry for one selected workspace only after typing the exact workspace name. It can optionally remove the saved workspace name, but it never clears all workspaces.
- Telemetry exports support JSON for full nested records and CSV for external analysis/optimizer ingestion.
- Policy batch seconds default to `120` and accept `30..900`; use fixed time windows for fair comparisons, then empirically decide whether AOI/Pareto runs need `180+`.
- Do not add auto-duration yet. For the next calibration/data pass, keep the fixed `120s` budget, compare runs within the same AOI scenario, and record cross-AOI distance/position bias as dataset context instead of normalizing it away.
- Role behavior is still shared-mapping with role bias: every drone contributes raw LiDAR mapping, while mapper/verifier/relay/scout differ through task pull, focused AOI rays, role counts, movement speed, network compliance, and derived controls. Future work should give each role a clearer downstream control surface so Pareto/effective-psi changes produce more distinct expert behavior.
- Telemetry samples include sparse per-drone trajectory vectors synchronized with the normal sample cadence; CSV exports only aggregate trajectory fields to stay compact.
- Mission Controls are grouped into collapsible clusters. The hidden A*/Weighted/Greedy planner controls should be kept for future swarm-algorithm comparisons.
- The controller computes normalized signals, behavior weights, derived controls, and a dependency graph; runtime motion, sensing focus, network compliance, and formation spread consume those derived controls.
- Soft constraints are now normalized controller signals too: AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure. They influence avoidance/efficiency behavior and the `constraintSafety` score without adding a hard collision-physics layer.
- `src/swarm/run-telemetry.js` persists swarm run records to IndexedDB, and the `Telemetry` sidebar panel can refresh saved runs or export them as JSON.
- The `Algorithm` sidebar panel shows live behavior weights, normalized signals, derived controls, role counts, and objective profile.
- Saved telemetry runs include objective scoring components for AOI quality, coverage, network resilience, time efficiency, compute efficiency, and energy/path proxy. The default comparison score is now the equal-weight fixed Pareto evaluator; psi-weighted scoring remains a policy-aligned diagnostic.
- `src/swarm/resource-model.js` adds first-pass placeholder resource accounting for hover, motion, sensor, compute, communication, total Wh, and estimated battery remaining. Treat these as schema/relative metrics until calibrated from sourced hardware specs.
- `src/swarm/run-telemetry.js` now saves per-sample temporal metrics and per-run temporal summaries, including new voxels/sec, AOI hit rate after contact, energy/compute rate, network fragmentation time, and behavior-weight change rate.
- LiDAR footprint coverage is now tracked as metrics only: total/unique/redundant footprint area, redundancy ratio, and resolution score. It does not create synthetic point-cloud samples.
- Inter-drone footprint redundancy now feeds back into topology by expanding formation radius/vertical spacing for area coverage, while low resolution can tighten spacing for detail.
- `src/swarm/behavior-profile.js` now includes `DEFAULT_SWARM_EVALUATION_PROFILE` for scoring/validity thresholds and `OPTIMIZER_PARAMETER_REGISTRY` for the four policy-coordinate Bayesian-optimization search space.
- Telemetry records now carry model family/version, base policy coordinates, runtime nudge, effective policy coordinates, delta policy coordinates, derived profile summaries, Pareto vector scores, scalar loss, and scalar display score. The telemetry panel/export default to `greybox-policy-v1`, leaving old runs stored but hidden from current-model comparisons.
- Telemetry can filter to valid runs or AOI scenario and sort saved runs by total score, confidence, or individual scoring components under the selected objective profile.
- The point-cloud overlay uses a compact `shown / total pts` readout to avoid small-window layout jitter when visible point counts update.
- Telemetry saves on Stop, Reset, or completion. Pause/Continue keeps the same run open. Formation, communication, scan-density, and performance changes are live-tunable and logged; terrain/AOI/swarm-size changes save the previous run and clear the map first.
- Role-aware task allocation includes frontier, AOI, relay, and verification tasks, with generated relay waypoints from launch/base toward the active mission center.
- Formation targets and visible drone motion are smoothed before terrain clearance, object avoidance, drone separation, and network-envelope corrections.
- Terrain generation has usable mountains, a valley corridor, village props, trees, complex building AOI, and a valley aperture object field.
- AOI markers are excluded from LiDAR raycast targets, so focused capture does not oversample marker spheres.
- Focused AOI rays spread across local Z-depth lanes and nearby X/Y offsets inside the AOI/object area.
- The live point cloud is raw LiDAR voxel evidence only; 9-neighbor/footprint splats are deferred to future post-scan reconstruction.
- Point cloud colors support general X/Z/Y depth perception plus a softer red AOI local-Z emphasis.
- Main and point-cloud viewports support mouse free-look, wheel zoom, right-drag pan, and camera-relative `W/A/S/D/Q/E` movement.

## Next Milestone Order

1. Run the fixed-120s nudge calibration and inspect validity/summary/export fields.
2. Fix mapper/verifier clumping with spatially distributed AOI sectors or lane ownership.
3. Strengthen role-specific controls so scouts, mappers, relays, and verifiers remain common enough for shared mapping but distinct enough for downstream behavior analysis.
4. Start Pareto-only batches across repeated seeds/AOI scenarios after nudge calibration.
5. Continue documenting changes in `README.md`, `DEVLOG.md`, `ARCHITECTURE.md`, `SWARM_ALGORITHM.md`, `RESEARCH_ROADMAP.md`, and this handoff file.

## Do Not Regress

- Do not skip drones to improve performance; every drone scans every scan pass.
- Do not use terrain height math as privileged LiDAR knowledge.
- Do not collapse the AOI scan back onto the marker center.
- Do not make export depend on only the visible sampled cloud.
- Do not add strike behavior, target selection, weapons, military autonomy, or harmful jamming instructions. Adversarial non-combat stressors are allowed for resilience and safety testing.

## New Session Startup

1. Open `SESSION_HANDOFF.md`, `ARCHITECTURE.md`, `README.md`, and the tail of `DEVLOG.md`.
2. Run `git status --short --branch`.
3. Run `npm.cmd run build`.
4. Start Vite with `npm.cmd run dev -- --host 127.0.0.1 --port 5180`.
5. Test the default mission, then switch to Dense or Survey only after checking FPS, scan time, cloud flush time, and visible point count.
