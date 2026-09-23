#include <gtest/gtest.h>
#include <toml++/toml.h>

#include "ui/config.hpp"

namespace auto_battlebot {
namespace {

std::unique_ptr<UiConfiguration> load(const char *text) {
    toml::table table = toml::parse(text);
    std::vector<std::string> sections;
    return load_ui_from_toml(table, sections);
}

TEST(UiConfigTest, ReadsLabelColorsLowercased) {
    auto config = load(R"(
[ui.label_colors]
MR_STABS_MK2 = "#ea2e2e"
opponent = "#22C55E"
)");
    ASSERT_TRUE(config);
    EXPECT_FALSE(config->enable);
    ASSERT_EQ(config->label_colors.size(), 2u);
    std::map<std::string, std::string> colors(config->label_colors.begin(),
                                              config->label_colors.end());
    EXPECT_EQ(colors["mr_stabs_mk2"], "#ea2e2e");
    EXPECT_EQ(colors["opponent"], "#22C55E");
}

TEST(UiConfigTest, RejectsAColorThatIsNotHex) {
    EXPECT_THROW(load("[ui.label_colors]\nopponent = \"green\"\n"), ConfigValidationError);
    EXPECT_THROW(load("[ui.label_colors]\nopponent = \"#22c55\"\n"), ConfigValidationError);
    EXPECT_THROW(load("[ui.label_colors]\nopponent = 5\n"), ConfigValidationError);
}

}  // namespace
}  // namespace auto_battlebot
