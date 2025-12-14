#pragma once

#include <string>

namespace auto_battlebot
{
    enum class KeypointLabel
    {
        MR_STABS_MK1_FRONT,
        MR_STABS_MK1_BACK,
        MR_STABS_MK2_FRONT,
        MR_STABS_MK2_BACK,
        MRS_BUFF_MK1_FRONT,
        MRS_BUFF_MK1_BACK,
        MRS_BUFF_MK2_FRONT,
        MRS_BUFF_MK2_BACK,
        HOUSE_BOT_FRONT,
        HOUSE_BOT_BACK
    };

    std::string keypoint_label_to_string(KeypointLabel label);
    KeypointLabel string_to_keypoint_label(const std::string &str);

} // namespace auto_battlebot
