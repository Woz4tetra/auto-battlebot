# 2026-11-15 shared hazards file

## Notes moved from code comments (2026-11-15)

### `simulation/hazards.py`

One TOML file describes the hazards for both sides of a run: the kinematic sim reads it here
(``[sim] obstacles_file``) and the C++ field filter reads the same path
(``[field_filter] hazards_file``) to build its keep-out discs. A single file means the simulated
floor and the controller's model of it cannot drift apart, which two hand-synced copies would.

The path is written repo-relative ("config/hazards/one_hole.toml") because the two configs that
name it have no directory in common. Each side resolves it against the project root itself.

File format, all in field-frame metres:

[[hazards]]
kind = "hole"          # hole | wall_block
center = [0.3, -0.2]
radius = 0.25          # raw geometry, before any inflation

