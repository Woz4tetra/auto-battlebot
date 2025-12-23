#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class PublisherInterface
    {
    public:
        virtual ~PublisherInterface() = default;
        virtual void publish_camera_data(const CameraData &data) = 0;
        virtual void publish_field_mask(const FieldMaskStamped &field_mask, const RgbImage &image) = 0;
        virtual void publish_initial_field_description(const FieldDescriptionWithInlierPoints &field) = 0;
        virtual void publish_field_description(const FieldDescription &field_description, const FieldDescriptionWithInlierPoints &initial_field_description) = 0;
        virtual void publish_robots(const RobotDescriptionsStamped &robots) = 0;
    };

} // namespace auto_battlebot
