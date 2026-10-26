# Implementation Plan: Update `arena_layout.yaml` from Real Field Photos (`01`–`29`)

## Goal Description
Update [`arena_layout.yaml`](file:///C:/Users/Aljamea%20Nairobi/.claude/jobs/e01d35ae/tmp/course/arena_layout.yaml) so our Gazebo model accurately reflects the physical Robotics Dojo 2026 arena documented in the 29 workshop photos (`small/01_*.jpg` through `small/29_*.jpg`).

## User Review Required

> [!IMPORTANT]
> **Fundamental Footprint & Topology Choice**
> The physical arena built in photos `01`–`29` is constructed on **four standard `1220 × 2440 mm` (4 ft × 8 ft) plywood sheets** laid side-by-side (`4880 × 2440 mm`, aspect ratio `2:1`), rather than the rulebook's `6703.56 × 4868 mm` (`1.38:1`).
> Before applying edits to [`arena_layout.yaml`](file:///C:/Users/Aljamea%20Nairobi/.claude/jobs/e01d35ae/tmp/course/arena_layout.yaml), please confirm whether you want:
> 1. **Option A (True As-Built `4880 × 2440 mm` Model — Recommended for real-world sim-to-real transfer)**: Update both outer dimensions (`length: 4880.0`, `width: 2440.0`), `wall_height: 400`, all internal walls, the two outer-corridor ramps + grass strip, the 3 T-shaped dividers, and the 3-bay V-notched station to match the real plywood arena.
> 2. **Option B (Rulebook `6703.56 × 4868 mm` Footprint + Measured Component Updates)**: Keep the rulebook outer dimensions (`6703.56 × 4868.0`) while updating `wall_height: 400`, ramp dimensions/surfaces (including the grass corridor), the T-shaped dividers, and the 400 mm V-notched station.

## Proposed Changes

### Arena Layout Configuration

#### [MODIFY] [arena_layout.yaml](file:///C:/Users/Aljamea%20Nairobi/.claude/jobs/e01d35ae/tmp/course/arena_layout.yaml)

##### Option A: Complete As-Built Layout (`4880 × 2440 mm` frame, origin at bottom-left of End A in `01, 04`, $x \in [0, 4880]$ along the long axis toward End B, $y \in [0, 2440]$ across the width from the grass side $y=0$ to the ramp side $y=2440$)

```yaml
field:
  length: 4880.0          # 4 x 1220 mm plywood sheets (matches rulebook 4868 mm within saw kerf)
  width: 2440.0           # 1 x 2440 mm plywood sheet wide (photos 01, 04, 08, 16, 17)
  wall_height: 400        # Measured from "40 x ..." cm pencil markings on walls (05, 10, 12, 13, 21)
  wall_thickness: 18      # 18 mm pine plywood (05, 06, 07, 12, 21, 27)

# Internal walls: [x1, y1, x2, y2] centre-lines (mm)
walls:
  # Ramp 1 (gravel/soil) inner wall - closed to x=0 outer wall (01, 04, 06, 07, 14)
  - [0, 1840, 1550, 1840]
  # Ramp 2 (bare wood) inner wall and end stop (08, 15, 16, 20)
  - [2150, 1840, 3350, 1840]
  - [2150, 1840, 2150, 2440]
  # Grass corridor inner wall (01, 04, 08, 16, 22, 23)
  - [1220, 580, 3660, 580]
  # Central enclosed box & chicane walls with pencil markings (05, 08, 09, 10, 11, 12, 13, 16)
  - [1220, 580, 1220, 1605]     # "40 x 102.5" cm wall (10, 12, 13)
  - [1220, 1605, 2220, 1605]    # upper side of central box (08, 09, 16)
  - [2220, 915, 2220, 1605]     # "40 x 69" cm wall (05, 12, 13)
  - [2920, 580, 2920, 1280]     # "40 x 70" cm wall (05, 12, 13)
  - [2920, 1280, 3875, 1280]    # "40 x 95.5" cm wall / central divider (05, 16, 17, 18)
  - [3875, 1280, 3875, 1400]    # lateral step/jog in central divider (16, 17, 18)
  - [3875, 1400, 4880, 1400]    # End B room divider wall (17, 18, 19)
  # End B narrow robot slot wall (17, 18, 19)
  - [3875, 1850, 3875, 2440]
  # End B corner small box (19, 25, 28, 29)
  - [4330, 1400, 4330, 1800]
  - [4330, 1800, 4880, 1800]
  # End B L-shaped enclosure in front of V-notched deck (23, 24, 25, 26, 28, 29)
  - [3660, 580, 3660, 1400]
  # 3 freestanding T-shaped dividers at End A (01, 04, 06, 07, 08, 15, 16, 21)
  - [550, 550, 1100, 550]       # T1 stem (55 cm)
  - [1100, 410, 1100, 690]      # T1 crossbar
  - [550, 1050, 1100, 1050]     # T2 stem (55 cm)
  - [1100, 910, 1100, 1190]     # T2 crossbar
  - [550, 1550, 1100, 1550]     # T3 stem (55 cm)
  - [1100, 1410, 1100, 1690]    # T3 crossbar

# 3-bay V-notched station at End B (23, 24, 25, 26, 27, 28, 29)
offloading_bays:
  depth: 560
  x_edges: [4320, 4880]
  y_edges: [0, 467, 933, 1400]
  colours: [unknown, unknown, unknown]  # unpainted bare plywood in photos
  divider_height: 400
  top_cover_height: 400                 # horizontal deck with 3 V-notches (26, 27, 28, 29)

# Terrain & Ramps (01, 04, 06, 07, 08, 14, 15, 16, 20, 22, 23)
ramps:
  - {name: ramp_gravel, centre: [850, 2140],  length: 1300, width: 580, height: 70, surface: gravel}
  - {name: ramp_wood,   centre: [2750, 2140], length: 1150, width: 580, height: 80, surface: wood}
  - {name: patch_grass, centre: [1830, 290],  length: 1220, width: 580, height: 12, surface: grass}
```

## Verification Plan
### Automated Verification
- After approval, apply the selected edits to [`arena_layout.yaml`](file:///C:/Users/Aljamea%20Nairobi/.claude/jobs/e01d35ae/tmp/course/arena_layout.yaml) and run the layout preview generator (if available) or validate YAML syntax with Python (`python -c "import yaml; yaml.safe_load(open('arena_layout.yaml'))"`).
### Manual Verification
- Visually compare the generated top-down preview against photos `01, 04, 08, 14, 16, 17, 19, 21, 22, 28`.
