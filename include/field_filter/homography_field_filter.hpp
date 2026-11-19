#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
class HomographyFieldFilter : public CameraWorldFieldFilter {
   public:
    explicit HomographyFieldFilter(const HomographyFieldFilterConfiguration &config);
    ~HomographyFieldFilter() override = default;

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;

   protected:
    const char *filter_name_for_log() const override { return "HomographyFieldFilter"; }

   private:
    HomographyFieldFilterConfiguration config_;
    int consecutive_fit_failures_ = 0;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
