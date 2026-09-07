#pragma once

#include <map>
#include <memory>
#include <string>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {
/**
 * Diagnostics backend that publishes one JSON channel per module, `/diagnostics/<module>`, to
 * the live relay and the recorder. Values are typed (numbers stay numbers) and the payload is
 * keyed by subsection so a Foxglove plot path cannot silently retarget when a module first logs
 * mid-run. Layout in docs/foxglove_recording_format.md.
 */
class FoxgloveDiagnosticsBackend : public DiagnosticsBackend {
   public:
    FoxgloveDiagnosticsBackend(std::shared_ptr<VizSink> sink,
                               std::shared_ptr<McapRecorder> mcap_recorder);
    void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) override;

   private:
    OutputChannel &channel_for(const std::string &module);

    std::shared_ptr<VizSink> sink_;
    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::map<std::string, std::unique_ptr<OutputChannel>> channels_;
};

/** Build the `/diagnostics/<module>` JSON payload from the snapshots of one module. */
std::string diagnostics_module_json(const std::string &module,
                                    const std::vector<const DiagnosticStatusSnapshot *> &snapshots);
}  // namespace auto_battlebot
