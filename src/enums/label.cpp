#include "enums/label.hpp"
#include <stdexcept>

namespace auto_battlebot {

std::string label_to_string(Label label) {
    switch (label) {
        case Label::MR_STABS_MK1: return "MR_STABS_MK1";
        case Label::MR_STABS_MK2: return "MR_STABS_MK2";
        case Label::MRS_BUFF_MK1: return "MRS_BUFF_MK1";
        case Label::MRS_BUFF_MK2: return "MRS_BUFF_MK2";
        case Label::HOUSE_BOT: return "HOUSE_BOT";
        case Label::OPPONENT: return "OPPONENT";
        default: throw std::invalid_argument("Unknown Label");
    }
}

Label string_to_label(const std::string& str) {
    if (str == "MR_STABS_MK1") return Label::MR_STABS_MK1;
    if (str == "MR_STABS_MK2") return Label::MR_STABS_MK2;
    if (str == "MRS_BUFF_MK1") return Label::MRS_BUFF_MK1;
    if (str == "MRS_BUFF_MK2") return Label::MRS_BUFF_MK2;
    if (str == "HOUSE_BOT") return Label::HOUSE_BOT;
    if (str == "OPPONENT") return Label::OPPONENT;
    throw std::invalid_argument("Unknown Label string: " + str);
}

}  // namespace auto_battlebot
