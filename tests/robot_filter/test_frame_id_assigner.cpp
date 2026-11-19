#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "data_structures/measurement.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/frame_id_assigner.hpp"

using namespace auto_battlebot;

namespace {

MeasurementWithConfidence measurement_at(double x) {
    MeasurementWithConfidence measurement;
    measurement.confidence = 1.0;
    measurement.description.pose.position = Position{x, 0.0, 0.0};
    return measurement;
}

FrameId assigned_frame_at(const std::vector<RobotDescription> &result, double x) {
    for (const auto &description : result) {
        if (description.pose.position.x == x) return description.frame_id;
    }
    return FrameId::EMPTY;
}

FrameIdAssigner make_assigner() {
    FrameIdAssigner assigner(10.0, 5);
    assigner.set_last_position(FrameId::THEIR_ROBOT_1, Position{0.0, 0.0, 0.0});
    assigner.set_last_position(FrameId::THEIR_ROBOT_2, Position{1.0, 0.0, 0.0});
    return assigner;
}

}  // namespace

TEST(FrameIdAssignerTest, PooledAssignmentIndependentOfMeasurementOrder) {
    const std::vector<FrameId> frame_ids = {FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2};
    const std::vector<FrameId> either = {FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2};
    const std::vector<FrameId> first_only = {FrameId::THEIR_ROBOT_1};
    auto logger = std::make_shared<DiagnosticsModuleLogger>("test_frame_id_assigner");

    FrameIdAssigner forward = make_assigner();
    std::vector<MeasurementWithConfidence> forward_measurements = {measurement_at(0.9),
                                                                   measurement_at(0.95)};
    const auto forward_result =
        forward.assign(forward_measurements, frame_ids, logger, {either, first_only});

    FrameIdAssigner reversed = make_assigner();
    std::vector<MeasurementWithConfidence> reversed_measurements = {measurement_at(0.95),
                                                                    measurement_at(0.9)};
    const auto reversed_result =
        reversed.assign(reversed_measurements, frame_ids, logger, {first_only, either});

    ASSERT_EQ(forward_result.size(), 2u);
    ASSERT_EQ(reversed_result.size(), 2u);
    EXPECT_EQ(assigned_frame_at(forward_result, 0.95), FrameId::THEIR_ROBOT_1);
    EXPECT_EQ(assigned_frame_at(forward_result, 0.9), FrameId::THEIR_ROBOT_2);
    EXPECT_EQ(assigned_frame_at(reversed_result, 0.95), FrameId::THEIR_ROBOT_1);
    EXPECT_EQ(assigned_frame_at(reversed_result, 0.9), FrameId::THEIR_ROBOT_2);
}
