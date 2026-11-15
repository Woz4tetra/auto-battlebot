#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/cage_calibration.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
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
