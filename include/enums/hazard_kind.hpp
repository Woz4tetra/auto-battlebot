#pragma once

namespace auto_battlebot {
/** What a hazard disc is. HOLE and WALL_BLOCK come from the arena geometry file, which writes them
 * in lowercase ("hole", "wall_block") because the sim reads the same file. ROBOT is a live tracked
 * robot such as the house bot. RlPolicyNavigation's observation one-hot follows this order. */
enum class HazardKind { HOLE, WALL_BLOCK, ROBOT };
}  // namespace auto_battlebot
