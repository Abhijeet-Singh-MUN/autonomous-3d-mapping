# Session Handoff

## Repository State

- Local worktree: `D:\2026 projects\Autonomous 3d mapping swarm-v1`
- Active branch: `swarm-v1`
- Remote: `origin https://github.com/Abhijeet-Singh-MUN/autonomous-3d-mapping.git`
- Baseline branch: `main` keeps the posted single-drone room project preserved.
- Current branch direction: terrain/swarm simulator for adaptive 3D mapping and point-cloud dataset capture.

## Run Commands

```bash
npm.cmd install
npm.cmd run dev
npm.cmd run build
```

The local Vite URL is usually:

```text
http://127.0.0.1:5180/
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
- Scan density: `Light`.
- Performance budget: `Quality`.
- Visible point cap path supports up to `1,200,000` points in custom mode.
- AOI color cue: red local Z-depth gradient around object/AOI scale.

## Good Enough Visual State

- Drones launch from a distributed square ground lattice, then take off into formation behavior.
- Terrain generation has usable mountains, a valley corridor, village props, trees, complex building AOI, and a valley aperture object field.
- AOI markers are excluded from LiDAR raycast targets, so focused capture does not oversample marker spheres.
- Focused AOI rays spread across local Z-depth lanes and nearby X/Y offsets inside the AOI/object area.
- Point cloud colors support general X/Z/Y depth perception plus red AOI local-Z emphasis.
- Main and point-cloud viewports support mouse free-look, wheel zoom, right-drag pan, and camera-relative `W/A/S/D/Q/E` movement.

## Next Milestone Order

1. Stabilize dense/survey performance in the browser-first local simulator.
2. Keep performance diagnostics visible while tuning.
3. Extract point-cloud and LiDAR scanning modules once behavior is stable.
4. Design and implement the custom swarm behavior kernel after Dense/Survey capture is usable.
5. Continue documenting changes in `README.md`, `DEVLOG.md`, `ARCHITECTURE.md`, and this handoff file.

## Do Not Regress

- Do not skip drones to improve performance; every drone scans every scan pass.
- Do not use terrain height math as privileged LiDAR knowledge.
- Do not collapse the AOI scan back onto the marker center.
- Do not make export depend on only the visible sampled cloud.
- Do not add weaponized behavior, targeting logic, jamming, strike mechanics, or military autonomy.

## New Session Startup

1. Open `SESSION_HANDOFF.md`, `ARCHITECTURE.md`, `README.md`, and the tail of `DEVLOG.md`.
2. Run `git status --short --branch`.
3. Run `npm.cmd run build`.
4. Start Vite with `npm.cmd run dev -- --host 127.0.0.1 --port 5180`.
5. Test the default mission, then switch to Dense or Survey only after checking FPS, scan time, cloud flush time, and visible point count.
