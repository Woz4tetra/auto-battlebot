#pragma once

#include "publisher/publisher_interface.hpp"

namespace auto_battlebot
{
    class NoopPublisher : public PublisherInterface
    {
    public:
        void publish_camera_data([[maybe_unused]] const CameraData &data) override {}
        void publish_field_mask([[maybe_unused]] const FieldMaskStamped &field_mask, [[maybe_unused]] const RgbImage &image) override {}
        void publish_initial_field_description([[maybe_unused]] const FieldDescriptionWithInlierPoints &field) override {}
        void publish_field_description([[maybe_unused]] const FieldDescription &field_description, [[maybe_unused]] const FieldDescriptionWithInlierPoints &initial_field_description) override {};
        void publish_keypoints([[maybe_unused]] const KeypointsStamped &keypoints) override {}
        void publish_robots([[maybe_unused]] const RobotDescriptionsStamped &robots) override {}
    };

} // namespace auto_battlebot
