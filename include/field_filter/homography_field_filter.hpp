#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
/**
 * @brief Field pose from the mask outline in RGB and a known mat size. No depth.
 *
 * The depth path measures the field and needs a point cloud; this one assumes the size and needs
 * only the outline and the intrinsics. On the two well-framed 2024-10-26 tripod recordings the
 * two agree to 5.0 cm and 5.2 cm in camera range, with plane tilt inside 0.53 degrees.
 *
 * The trade is that the assumed size enters linearly: get the mat wrong by 4% and every range is
 * wrong by 4%. `field_size_x` and `field_size_y` are the floor mat, measured per venue, not the
 * cage. Nominal 8 ft runs about 6% long against measured mats of 2.30 to 2.40 m.
 */
class HomographyFieldFilter : public CameraWorldFieldFilter {
   public:
    explicit HomographyFieldFilter(const HomographyFieldFilterConfiguration &config);
    ~HomographyFieldFilter() override = default;

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;

   protected:
    const char *name() const override { return "HomographyFieldFilter"; }

   private:
    HomographyFieldFilterConfiguration config_;
    /** Failures since the last successful fit, so a retried init reports its diagnosis once. */
    int consecutive_failures_ = 0;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
