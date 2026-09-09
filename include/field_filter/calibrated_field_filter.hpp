#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/cage_calibration.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
/**
 * @brief Publishes a cage that was measured once, and checks the fixture still sits where it did.
 *
 * The match-day path. Fitting the mat outline at match time depends on segmenting a floor that
 * has a robot on it, and on the near corners being in frame, which at a cage mount they may not
 * be. A cage measured beforehand has neither problem.
 *
 * The line fit still runs at every startup, against the same frame, purely as a seating check.
 * Nothing else can notice that the fixture did not seat or that the cage got bumped: a clamped
 * camera has no visual odometry to disagree with. Above `max_seating_error_px` it warns and
 * publishes the stored calibration anyway, which is the answer we trust.
 */
class CalibratedFieldFilter : public CameraWorldFieldFilter {
   public:
    explicit CalibratedFieldFilter(const CalibratedFieldFilterConfiguration &config);
    ~CalibratedFieldFilter() override = default;

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;

   protected:
    const char *name() const override { return "CalibratedFieldFilter"; }

   private:
    void run_seating_check(const CameraData &camera_data, const MaskStamped &field_mask);

    CalibratedFieldFilterConfiguration config_;
    CageCalibration calibration_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
