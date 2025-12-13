#include "enums/keypoint_label.hpp"
#include <stdexcept>

namespace auto_battlebot {

std::string keypoint_label_to_string(KeypointLabel label) {
    switch (label) {
        case KeypointLabel::MR_STABS_MK1_FRONT: return "MR_STABS_MK1_FRONT";
        case KeypointLabel::MR_STABS_MK1_BACK: return "MR_STABS_MK1_BACK";
        case KeypointLabel::MR_STABS_MK2_FRONT: return "MR_STABS_MK2_FRONT";
        case KeypointLabel::MR_STABS_MK2_BACK: return "MR_STABS_MK2_BACK";
        case KeypointLabel::MRS_BUFF_MK1_FRONT: return "MRS_BUFF_MK1_FRONT";
        case KeypointLabel::MRS_BUFF_MK1_BACK: return "MRS_BUFF_MK1_BACK";
        case KeypointLabel::MRS_BUFF_MK2_FRONT: return "MRS_BUFF_MK2_FRONT";
        case KeypointLabel::MRS_BUFF_MK2_BACK: return "MRS_BUFF_MK2_BACK";
        case KeypointLabel::HOUSE_BOT_FRONT: return "HOUSE_BOT_FRONT";
        case KeypointLabel::HOUSE_BOT_BACK: return "HOUSE_BOT_BACK";
        default: throw std::invalid_argument("Unknown KeypointLabel");
    }
}

KeypointLabel string_to_keypoint_label(const std::string& str) {
    if (str == "MR_STABS_MK1_FRONT") return KeypointLabel::MR_STABS_MK1_FRONT;
    if (str == "MR_STABS_MK1_BACK") return KeypointLabel::MR_STABS_MK1_BACK;
    if (str == "MR_STABS_MK2_FRONT") return KeypointLabel::MR_STABS_MK2_FRONT;
    if (str == "MR_STABS_MK2_BACK") return KeypointLabel::MR_STABS_MK2_BACK;
    if (str == "MRS_BUFF_MK1_FRONT") return KeypointLabel::MRS_BUFF_MK1_FRONT;
    if (str == "MRS_BUFF_MK1_BACK") return KeypointLabel::MRS_BUFF_MK1_BACK;
    if (str == "MRS_BUFF_MK2_FRONT") return KeypointLabel::MRS_BUFF_MK2_FRONT;
    if (str == "MRS_BUFF_MK2_BACK") return KeypointLabel::MRS_BUFF_MK2_BACK;
    if (str == "HOUSE_BOT_FRONT") return KeypointLabel::HOUSE_BOT_FRONT;
    if (str == "HOUSE_BOT_BACK") return KeypointLabel::HOUSE_BOT_BACK;
    throw std::invalid_argument("Unknown KeypointLabel string: " + str);
}

}  // namespace auto_battlebot
