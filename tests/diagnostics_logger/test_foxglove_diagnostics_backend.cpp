#include <gtest/gtest.h>

#include <cmath>

#include "diagnostics_logger/foxglove_diagnostics_backend.hpp"

namespace auto_battlebot {

TEST(FoxgloveDiagnosticsBackendTest, ModuleJsonKeepsTypesAndKeysBySubsection) {
    DiagnosticStatusSnapshot top;
    top.name = "runner";
    top.subsection = "";
    top.level = DiagnosticLevel::OK;
    top.values = {{"rate", 59.7}, {"count", 3}, {"mode", std::string("auto")}};

    DiagnosticStatusSnapshot tick;
    tick.name = "runner";
    tick.subsection = "tick";
    tick.level = DiagnosticLevel::WARN;
    tick.message = "slow \"tick\"";
    tick.values = {{"elapsed_ms", 12.5}, {"nan_value", std::nan("")}};

    const auto json = diagnostics_module_json("runner", {&top, &tick});
    EXPECT_EQ(json,
              "{\"runner\":{\"level\":0,\"message\":\"\",\"values\":{\"count\":3,\"mode\":\"auto\","
              "\"rate\":59.7}},"
              "\"tick\":{\"level\":1,\"message\":\"slow \\\"tick\\\"\",\"values\":{\"elapsed_ms\":"
              "12.5,\"nan_value\":null}}}");
}

TEST(FoxgloveDiagnosticsBackendTest, NoSinksIsANoop) {
    FoxgloveDiagnosticsBackend backend(nullptr, nullptr);
    DiagnosticStatusSnapshot snap;
    snap.name = "runner";
    snap.values = {{"rate", 1}};
    EXPECT_NO_THROW(backend.receive({snap}));
    EXPECT_NO_THROW(backend.receive({}));
}

}  // namespace auto_battlebot
