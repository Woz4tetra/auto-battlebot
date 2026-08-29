// The assembler is where inflation happens, once, and where the two hazard sources get their
// deliberately different decay: config geometry never expires, a neutral track is held past
// going stale and then dropped.

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

/** Our robot as the filter reports it: a 0.3 x 0.2 m footprint, half-diagonal 0.1803. */
RobotDescription our_robot(double half_x = 0.15, double half_y = 0.10) {
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
    robot.velocity.vx = 0.5;
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
    hole.radius = 0.25;
    HazardAssembler assembler(fixture.config, {hole}, fixture.clock);

    FieldDescription field;
    assembler.assemble(robots_with({our_robot()}), field);

    ASSERT_EQ(field.hazards.size(), 1u);
    EXPECT_EQ(field.hazards[0].source, HazardSource::STATIC);
    EXPECT_NEAR(field.hazards[0].center.x, 0.3, 1e-9);
    EXPECT_NEAR(field.hazards[0].center.y, -0.2, 1e-9);
    EXPECT_NEAR(field.hazards[0].inflated_radius, 0.25 + expected_half_diagonal(0.3, 0.2) + 0.10,
                1e-9);
    // The loss boundary carries only the small hard margin: steering plans around the fat disc,
    // the velocity barrier gets strict at the thin one.
    EXPECT_NEAR(field.hazards[0].hard_radius, 0.25 + expected_half_diagonal(0.3, 0.2) + 0.02, 1e-9);
    EXPECT_LT(field.hazards[0].hard_radius, field.hazards[0].inflated_radius);
}

TEST(HazardAssemblerTest, OurSizeFallsBackRatherThanShrinkingOnADropout) {
    // Without a fallback, a frame with no our-robot track would briefly shrink every keep-out
    // disc, which is the wrong direction to be wrong in.
    Fixture fixture;
    HazardAssembler assembler(fixture.config, {StaticHazardConfig{"hole", 0.0, 0.0, 0.2}},
                              fixture.clock);

    FieldDescription seen;
    assembler.assemble(robots_with({our_robot()}), seen);
    const double with_track = seen.hazards[0].inflated_radius;

    FieldDescription dropped;
    assembler.assemble(robots_with({}), dropped);
    EXPECT_NEAR(dropped.hazards[0].inflated_radius, with_track, 1e-9);
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
    EXPECT_NEAR(field.hazards[0].velocity.vx, 0.5, 1e-9);
    EXPECT_NEAR(field.hazards[0].inflated_radius,
                expected_half_diagonal(0.3, 0.3) + expected_half_diagonal(0.3, 0.2) + 0.05, 1e-9);
    EXPECT_NEAR(field.hazards[0].hard_radius,
                expected_half_diagonal(0.3, 0.3) + expected_half_diagonal(0.3, 0.2) + 0.02, 1e-9);
}

TEST(HazardAssemblerTest, OpponentsAreNotHazards) {
    // Opponents are what the solver already avoids as point sites. Turning them into keep-out
    // discs here would double-count them and make the field unreachable.
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
    // Forgetting a hazard is worse than holding a slightly wrong one, so the disc outlives the
    // track -- but only for the configured window, or a house bot that left the arena would keep
    // a chunk of the field fenced off for the rest of the match.
    Fixture fixture;
    fixture.config.tracked_hold_s = 0.5;
    HazardAssembler assembler(fixture.config, {}, fixture.clock);

    fixture.clock->set(10.0);
    FieldDescription live;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1)}), live);
    ASSERT_EQ(live.hazards.size(), 1u);

    fixture.clock->set(10.3);
    FieldDescription held;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1, /*stale=*/true)}), held);
    ASSERT_EQ(held.hazards.size(), 1u) << "still inside the hold window";
    EXPECT_NEAR(held.hazards[0].center.x, 0.4, 1e-9);

    fixture.clock->set(10.8);
    FieldDescription expired;
    assembler.assemble(robots_with({our_robot(), house_bot(0.4, 0.1, /*stale=*/true)}), expired);
    EXPECT_TRUE(expired.hazards.empty());
}

TEST(HazardAssemblerTest, ADetectorOutageDoesNotDeleteTheStaticHole) {
    // The hole does not move during a match, so config is the source of truth for it. This is
    // the asymmetry between the two sources, stated as a test.
    Fixture fixture;
    fixture.config.tracked_hold_s = 0.1;
    HazardAssembler assembler(fixture.config, {StaticHazardConfig{"hole", 0.0, 0.0, 0.2}},
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

// --- shared geometry file ---------------------------------------------------------------------

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
    EXPECT_EQ(hazards[0].kind, "hole");
    EXPECT_NEAR(hazards[0].center_x, 0.3, 1e-9);
    EXPECT_NEAR(hazards[0].radius, 0.25, 1e-9);
    EXPECT_EQ(hazards[1].kind, "wall_block");
    EXPECT_NEAR(hazards[1].center_y, 0.1, 1e-9);
}

TEST(HazardConfigTest, ResolvesARepoRelativePathAgainstTheProjectRoot) {
    // The sim config and the C++ config both name this file and share no directory, so the path
    // is written repo-relative and each side resolves it. If this regresses, the controller loses
    // its keep-out disc while the sim still has a hole in the floor.
    const auto hazards = load_static_hazards("config/hazards/one_hole.toml");
    ASSERT_EQ(hazards.size(), 1u);
    EXPECT_EQ(hazards[0].kind, "hole");
    EXPECT_NEAR(hazards[0].center_x, -0.35, 1e-9);
    EXPECT_NEAR(hazards[0].center_y, 0.2, 1e-9);
    EXPECT_NEAR(hazards[0].radius, 0.22, 1e-9);
}

TEST(HazardConfigTest, AMissingFileIsAnErrorNotAnEmptyList) {
    // A path typo must not silently remove the hole from the controller's model while the sim
    // still has one in the floor.
    EXPECT_THROW(load_static_hazards("/nonexistent/hazards.toml"), ConfigValidationError);
}

TEST(HazardConfigTest, RejectsAnUnknownKind) {
    const std::string path = write_temp_hazards(
        "[[hazards]]\nkind = \"lava\"\ncenter = [0, 0]\nradius = 0.2\n", "hazards_bad_kind");
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

TEST(HazardAssemblerTest, CarriesTheGeometryAlongsideTheKeepOut) {
    // object_radius is display-only, but it has to stay the hazard itself: a ring drawn at the
    // inflated radius alone reads as the hole being twice its size, the inflation invisible.
    Fixture fixture;
    fixture.config.static_margin_m = 0.10;
    fixture.config.tracked_margin_m = 0.05;

    StaticHazardConfig hole;
    hole.kind = "hole";
    hole.center_x = 0.2;
    hole.center_y = -0.1;
    hole.radius = 0.2828;
    HazardAssembler assembler(fixture.config, {hole}, fixture.clock);

    FieldDescription field;
    field.size.size = Size{2.4, 2.4, 0.0};
    assembler.assemble(robots_with({our_robot(), house_bot(0.6, 0.6)}), field);
    ASSERT_EQ(field.hazards.size(), 2u);

    const double ours = expected_half_diagonal(0.30, 0.20);
    const double their_own = expected_half_diagonal(0.30, 0.30);
    for (const auto &hazard : field.hazards) {
        if (hazard.source == HazardSource::STATIC) {
            EXPECT_NEAR(hazard.object_radius, 0.2828, 1e-9);
            EXPECT_NEAR(hazard.inflated_radius, 0.2828 + ours + 0.10, 1e-9);
        } else {
            EXPECT_NEAR(hazard.object_radius, their_own, 1e-9);
            EXPECT_NEAR(hazard.inflated_radius, their_own + ours + 0.05, 1e-9);
        }
        // The gap between the two rings is exactly the inflation, which is the point of drawing
        // both.
        EXPECT_GT(hazard.inflated_radius, hazard.object_radius);
    }
}

}  // namespace auto_battlebot
