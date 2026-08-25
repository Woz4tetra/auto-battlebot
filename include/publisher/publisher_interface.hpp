#pragma once

#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"

namespace auto_battlebot {
class PublisherInterface {
   public:
    virtual ~PublisherInterface() = default;
    virtual void publish_camera_data(const CameraData &data) = 0;
    virtual void publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                                    const CameraInfo &camera_info) = 0;
    virtual void publish_initial_field_description(
        const FieldDescriptionWithInlierPoints &field) = 0;
    virtual void publish_field_description(
        const FieldDescription &field_description,
        const FieldDescriptionWithInlierPoints &initial_field_description) = 0;
    /** Keep-out discs for this cycle, at the inflated radius the controller steers on. Published
     * every cycle (unlike the field border, which is published once) because tracked hazards move
     * and a replay has to be able to say what the controller knew. */
    virtual void publish_hazards(const FieldDescription &field_description) = 0;
    virtual void publish_robots(const RobotDescriptionsStamped &robots) = 0;
    virtual void publish_blob_detections(const DetectionsStamped &detections) = 0;
    virtual void publish_keypoint_detections(const DetectionsStamped &detections) = 0;
    virtual void publish_navigation(const NavigationVisualization &nav) = 0;
};

}  // namespace auto_battlebot
