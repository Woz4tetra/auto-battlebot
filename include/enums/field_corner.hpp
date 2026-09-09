#pragma once

namespace auto_battlebot {
/** Which field corner a fiducial board marks, named by the signs of its field-frame coordinates.
 *  That reads directly next to the corner-to-centre translation, which is exactly those two signs
 *  times half the field extent. Compass names would not. */
enum class FieldCorner { NEG_X_NEG_Y, NEG_X_POS_Y, POS_X_NEG_Y, POS_X_POS_Y };
}  // namespace auto_battlebot
