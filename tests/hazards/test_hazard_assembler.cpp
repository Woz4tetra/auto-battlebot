
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>

#include "enums/frame_id.hpp"
#include "hazards/hazard_assembler.hpp"
#include "hazards/hazard_config.hpp"
#include "time/manual_clock.hpp"

namespace auto_battlebot {
namespace {

constexpr double kOurHalfX = 0.15;
constexpr double kOurHalfY = 0.10;
const double kOurHalfDiagonal = std::hypot(kOurHalfX, kOurHalfY);
constexpr bool kStale = true;

RobotDescription our_robot(double half_x = kOurHalfX, double half_y = kOurHalfY) {
    RobotDescription robot;
    robot.frame_id = FrameId::OUR_ROBOT_1;
    robot.group = Group::OURS;
    robot.size = Size{2.0 * half_x, 2.0 * half_y, 0.1};
    return robot;
}

RobotDescription house_bot(double x, double y, bool stale = false) {
    RobotDescription robot;
    robot.frame_id = FrameId::NEUTRAL_ROBOT_1;
    robot.group = Group::NEUTRAL;
    robot.label = Label::HOUSE_BOT;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.rotation.w = 1.0;
    robot.size = Size{0.3, 0.3, 0.1};
    robot.velocity.vx_mps = 0.5;
    robot.is_stale = stale;
    return robot;
}

RobotDescriptionsStamped robots_with(std::vector<RobotDescription> descriptions) {
    RobotDescriptionsStamped robots;
    robots.descriptions = std::move(descriptions);
    return robots;
}

double expected_half_diagonal(double size_x, double size_y) {
    return 0.5 * std::sqrt(size_x * size_x + size_y * size_y);
}

struct Fixture {
    std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>();
    HazardAssemblerConfig config;
};

TEST(HazardAssemblerTest, StaticHazardIsInflatedOnceByOurSizePlusMargin) {
    Fixture fixture;
    fixture.config.static_margin_m = 0.10;
    StaticHazardConfig hole;
    hole.center_x = 0.3;
    hole.center_y = -0.2;
    hole.raw_radius_m = 0.25;
    HazardAssembler assembler(fixture.config, {hole}, fixture.clock);

    FieldDescription field;
    assembler.assemble(robots_with({our_robot()}), field);

    ASSERT_EQ(field.hazards.size(), 1u);
    EXPECT_EQ(field.hazards[0].source, HazardSource::STATIC);
    EXPECT_NEAR(field.hazards[0].center.x, 0.3, 1e-9);
    EXPECT_NEAR(field.hazards[0].center.y, -0.2, 1e-9);
    EXPECT_NEAR(field.hazards[0].inflated_radius_m, 0.25 + kOurHalfDiagonal + 0.10, 1e-9);
    EXPECT_NEAR(field.hazards[0].hard_radius, 0.25 + kOurHalfDiagonal + 0.02, 1e-9);
    EXPECT_LT(field.hazards[0].hard_radius, field.hazards[0].inflated_radius_m);
}

TEST(HazardAssemblerTest, OurSizeFallsBackRatherThanShrinkingOnADropout) {
    Fixture fixture;
    HazardAssembler assembler(fixture.config, {StaticHazardConfig{HazardKind::HOLE, 0.0, 0.0, 0.2}},
                              fixture.clock);

    FieldDescription seen;
    assembler.assemble(robots_with({our_robot()}), seen);
    const double with_track = seen.hazards[0].inflated_radius_m;

    FieldDescription dropped;
    assembler.assemble(robots_with({}), dropped);
    EXPECT_NEAR(dropped.hazards[0].inflated_radius_m, with_track, 1e-9);
}

TEST(HazardAssemblerTest, NeutralTrackBecomesATrackedHazardCarryingItsVelocity) {
    Fixture fixture;
    fixture.config.tracked_margin_m = 0.05;
    HazardAssembler assembler(fixture.config, {}, fixture.clock);

    FieldDescription field;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1)}), field);

    ASSERT_EQ(field.hazards.size(), 1u);
    EXPECT_EQ(field.hazards[0].source, HazardSource::TRACKED);
    EXPECT_NEAR(field.hazards[0].center.x, 0.4, 1e-9);
    EXPECT_NEAR(field.hazards[0].velocity.vx_mps, 0.5, 1e-9);
    EXPECT_NEAR(field.hazards[0].inflated_radius_m,
                expected_half_diagonal(0.3, 0.3) + kOurHalfDiagonal + 0.05, 1e-9);
    EXPECT_NEAR(field.hazards[0].hard_radius,
                expected_half_diagonal(0.3, 0.3) + kOurHalfDiagonal + 0.02, 1e-9);
}

TEST(HazardAssemblerTest, TrackedHazardSitsOnTheFootprintCentre) {
    Fixture fixture;
    HazardAssembler assembler(fixture.config, {}, fixture.clock);
    RobotDescription bot = house_bot(0.4, 0.1);
    constexpr double kFacingPlusY = M_PI / 2.0;
    const Position2D kOffset{0.05, 0.02};
    bot.pose.rotation.w = std::cos(kFacingPlusY / 2.0);
    bot.pose.rotation.z = std::sin(kFacingPlusY / 2.0);
    bot.footprint_center_offset = kOffset;
    const double expected_x = bot.pose.position.x + std::cos(kFacingPlusY) * kOffset.x -
                              std::sin(kFacingPlusY) * kOffset.y;
    const double expected_y = bot.pose.position.y + std::sin(kFacingPlusY) * kOffset.x +
                              std::cos(kFacingPlusY) * kOffset.y;

    FieldDescription field;
    assembler.assemble(robots_with({our_robot(), bot}), field);

    ASSERT_EQ(field.hazards.size(), 1u);
    EXPECT_NEAR(field.hazards[0].center.x, expected_x, 1e-9);
    EXPECT_NEAR(field.hazards[0].center.y, expected_y, 1e-9);
}

TEST(HazardAssemblerTest, OpponentsAreNotHazards) {
    Fixture fixture;
    HazardAssembler assembler(fixture.config, {}, fixture.clock);

    RobotDescription opponent;
    opponent.frame_id = FrameId::THEIR_ROBOT_1;
    opponent.group = Group::THEIRS;
    opponent.size = Size{0.3, 0.3, 0.1};

    FieldDescription field;
    assembler.assemble(robots_with({our_robot(), opponent}), field);
    EXPECT_TRUE(field.hazards.empty());
}

TEST(HazardAssemblerTest, StaleNeutralTrackIsHeldThenDropped) {
    Fixture fixture;
    fixture.config.tracked_hold_s = 0.5;
    HazardAssembler assembler(fixture.config, {}, fixture.clock);

    fixture.clock->set(10.0);
    FieldDescription live;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1)}), live);
    ASSERT_EQ(live.hazards.size(), 1u);

    fixture.clock->set(10.3);
    FieldDescription held;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1, kStale)}), held);
    ASSERT_EQ(held.hazards.size(), 1u) << "still inside the hold window";
    EXPECT_NEAR(held.hazards[0].center.x, 0.4, 1e-9);

    fixture.clock->set(10.8);
    FieldDescription expired;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1, kStale)}), expired);
    EXPECT_TRUE(expired.hazards.empty());
}

TEST(HazardAssemblerTest, ADetectorOutageDoesNotDeleteTheStaticHole) {
    Fixture fixture;
    fixture.config.tracked_hold_s = 0.1;
    HazardAssembler assembler(fixture.config, {StaticHazardConfig{HazardKind::HOLE, 0.0, 0.0, 0.2}},
                              fixture.clock);

    fixture.clock->set(1.0);
    FieldDescription with_track;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1)}), with_track);
    EXPECT_EQ(with_track.hazards.size(), 2u);

    fixture.clock->set(5.0);
    FieldDescription outage;
    assembler.assemble(robots_with({}), outage);
    ASSERT_EQ(outage.hazards.size(), 1u);
    EXPECT_EQ(outage.hazards[0].source, HazardSource::STATIC);
}

std::string write_temp_hazards(const std::string &contents, const std::string &name) {
    const std::string path =
        std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/" + name + ".toml";
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
}

TEST(HazardConfigTest, ParsesTheSharedGeometryFile) {
    const std::string path = write_temp_hazards(R"(
[[hazards]]
kind = "hole"
center = [0.3, -0.2]
radius = 0.25

[[hazards]]
kind = "wall_block"
center = [-0.4, 0.1]
radius = 0.18
)",
                                                "hazards_ok");
    const auto hazards = load_static_hazards(path);
    ASSERT_EQ(hazards.size(), 2u);
    EXPECT_EQ(hazards[0].kind, HazardKind::HOLE);
    EXPECT_NEAR(hazards[0].center_x, 0.3, 1e-9);
    EXPECT_NEAR(hazards[0].raw_radius_m, 0.25, 1e-9);
    EXPECT_EQ(hazards[1].kind, HazardKind::WALL_BLOCK);
    EXPECT_NEAR(hazards[1].center_y, 0.1, 1e-9);
}

TEST(HazardConfigTest, AMissingFileIsAnErrorNotAnEmptyList) {
    EXPECT_THROW(load_static_hazards("/nonexistent/hazards.toml"), ConfigValidationError);
}

TEST(HazardConfigTest, RejectsAnUnknownKind) {
    const std::string path = write_temp_hazards(
        "[[hazards]]\nkind = \"lava\"\ncenter = [0, 0]\nradius = 0.2\n", "hazards_bad_kind");
    EXPECT_THROW(load_static_hazards(path), ConfigValidationError);
}

TEST(HazardConfigTest, AnUnknownKindNamesTheValidValues) {
    const std::string path = write_temp_hazards(
        "[[hazards]]\nkind = \"lava\"\ncenter = [0, 0]\nradius = 0.2\n", "hazards_bad_kind_msg");
    try {
        load_static_hazards(path);
        FAIL() << "expected ConfigValidationError";
    } catch (const ConfigValidationError &e) {
        const std::string message = e.what();
        EXPECT_NE(message.find("'lava'"), std::string::npos) << message;
        EXPECT_NE(message.find("hole, wall_block"), std::string::npos) << message;
    }
}

TEST(HazardConfigTest, RobotIsNotAFileKind) {
    const std::string path = write_temp_hazards(
        "[[hazards]]\nkind = \"robot\"\ncenter = [0, 0]\nradius = 0.2\n", "hazards_robot_kind");
    EXPECT_THROW(load_static_hazards(path), ConfigValidationError);
}

TEST(HazardConfigTest, RejectsANonPositiveRadius) {
    const std::string path = write_temp_hazards(
        "[[hazards]]\nkind = \"hole\"\ncenter = [0, 0]\nradius = 0.0\n", "hazards_bad_radius");
    EXPECT_THROW(load_static_hazards(path), ConfigValidationError);
}

TEST(HazardConfigTest, RejectsAnUnexpectedTopLevelKey) {
    const std::string path = write_temp_hazards("obstacles = []\n", "hazards_wrong_key");
    EXPECT_THROW(load_static_hazards(path), ConfigValidationError);
}

}  // namespace

TEST(HazardAssemblerTest, ObjectRadiusStaysTheHazardRadiusNotTheInflatedOne) {
    Fixture fixture;
    fixture.config.static_margin_m = 0.10;
    fixture.config.tracked_margin_m = 0.05;

    StaticHazardConfig hole;
    hole.kind = HazardKind::HOLE;
    hole.center_x = 0.2;
    hole.center_y = -0.1;
    hole.raw_radius_m = 0.2828;
    HazardAssembler assembler(fixture.config, {hole}, fixture.clock);

    FieldDescription field;
    field.size.size = Size{2.4, 2.4, 0.0};
    assembler.assemble(robots_with({our_robot(), house_bot(0.6, 0.6)}), field);
    ASSERT_EQ(field.hazards.size(), 2u);

    const double ours = kOurHalfDiagonal;
    const double their_own = expected_half_diagonal(0.30, 0.30);
    for (const auto &hazard : field.hazards) {
        if (hazard.source == HazardSource::STATIC) {
            EXPECT_NEAR(hazard.object_radius, 0.2828, 1e-9);
            EXPECT_NEAR(hazard.inflated_radius_m, 0.2828 + ours + 0.10, 1e-9);
        } else {
            EXPECT_NEAR(hazard.object_radius, their_own, 1e-9);
            EXPECT_NEAR(hazard.inflated_radius_m, their_own + ours + 0.05, 1e-9);
        }
        EXPECT_GT(hazard.inflated_radius_m, hazard.object_radius);
    }
}

TEST(HazardAssemblerTest, CarriesTheKindOfEachHazard) {
    Fixture fixture;
    StaticHazardConfig hole{HazardKind::HOLE, 0.5, 0.0, 0.2};
    StaticHazardConfig block{HazardKind::WALL_BLOCK, -0.5, 0.0, 0.2};
    HazardAssembler assembler(fixture.config, {hole, block}, fixture.clock);

    FieldDescription field;
    field.size.size = Size{2.4, 2.4, 0.0};
    assembler.assemble(robots_with({our_robot(), house_bot(0.6, 0.6)}), field);
    ASSERT_EQ(field.hazards.size(), 3u);
    EXPECT_EQ(field.hazards[0].kind, HazardKind::HOLE);
    EXPECT_EQ(field.hazards[1].kind, HazardKind::WALL_BLOCK);
    EXPECT_EQ(field.hazards[2].source, HazardSource::TRACKED);
    EXPECT_EQ(field.hazards[2].kind, HazardKind::ROBOT);
}

}  // namespace auto_battlebot
