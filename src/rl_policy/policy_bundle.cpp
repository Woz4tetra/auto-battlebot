#include "rl_policy/policy_bundle.hpp"

#include <toml++/toml.h>

#include <cmath>
#include <initializer_list>
#include <magic_enum.hpp>
#include <string_view>

namespace auto_battlebot::rl_policy {
namespace {

namespace fs = std::filesystem;

/** Every action a policy emits is [linear, angular], the two normalized VelocityCommand
 * fields. */
constexpr size_t kActionSize = 2;

class ManifestReader {
   public:
    explicit ManifestReader(fs::path directory) : directory_(std::move(directory)) {}

    [[noreturn]] void fail(const std::string &message) const {
        throw PolicyBundleError("policy bundle " + directory_.string() + ": " + message);
    }

    const fs::path &directory() const { return directory_; }

    /** Rejects keys outside `allowed`, so a misspelled field fails instead of being ignored. */
    void check_keys(const toml::table &table, std::initializer_list<std::string_view> allowed,
                    const std::string &where) const {
        for (const auto &[key, value] : table) {
            bool known = false;
            for (std::string_view name : allowed) known = known || key.str() == name;
            if (!known) {
                std::string valid;
                for (std::string_view name : allowed) {
                    if (!valid.empty()) valid += ", ";
                    valid += name;
                }
                fail(where + "unknown key '" + std::string(key.str()) + "'; valid keys: " + valid);
            }
        }
    }

    std::string require_string(const toml::table &table, const std::string &key,
                               const std::string &where) const {
        const auto value = table[key].value<std::string>();
        if (!value) fail(where + key + " is missing or not a string");
        return *value;
    }

    int64_t require_int(const toml::table &table, const std::string &key,
                        const std::string &where) const {
        const toml::node *node = table.get(key);
        if (node == nullptr || !node->is_integer())
            fail(where + key + " is missing or not an integer");
        return node->as_integer()->get();
    }

    double require_number(const toml::table &table, const std::string &key,
                          const std::string &where) const {
        const auto value = table[key].value<double>();
        if (!value) fail(where + key + " is missing or not a number");
        return *value;
    }

    const toml::table &require_table(const toml::table &table, const std::string &key,
                                     const std::string &where) const {
        const toml::table *sub = table[key].as_table();
        if (sub == nullptr) fail(where + "[" + key + "] is missing or not a table");
        return *sub;
    }

    template <typename E>
    E require_enum(const toml::table &table, const std::string &key,
                   const std::string &where) const {
        const std::string text = require_string(table, key, where);
        const auto value = magic_enum::enum_cast<E>(text);
        if (!value.has_value()) {
            std::string valid;
            for (std::string_view name : magic_enum::enum_names<E>()) {
                if (!valid.empty()) valid += ", ";
                valid += name;
            }
            fail(where + key + " '" + text +
                 "' is not a value this build knows; valid values: " + valid);
        }
        return *value;
    }

    std::vector<double> require_numbers(const toml::table &table, const std::string &key,
                                        const std::string &where) const {
        const toml::array *array = table[key].as_array();
        if (array == nullptr || array->empty()) fail(where + key + " is missing or empty");
        std::vector<double> values;
        for (size_t i = 0; i < array->size(); ++i) {
            const auto value = (*array)[i].value<double>();
            if (!value || !std::isfinite(*value)) {
                fail(where + key + "[" + std::to_string(i) + "] is not a finite number");
            }
            values.push_back(*value);
        }
        return values;
    }

    fs::path require_file(const toml::table &table, const std::string &key,
                          const std::string &where) const {
        const fs::path path = directory_ / require_string(table, key, where);
        if (!fs::is_regular_file(path))
            fail(where + key + " names a missing file: " + path.string());
        return path;
    }

    std::vector<TensorSpec> require_tensors(const toml::table &table, const std::string &key,
                                            const std::string &where) const {
        const toml::array *array = table[key].as_array();
        if (array == nullptr || array->empty()) fail(where + key + " is missing or empty");
        std::vector<TensorSpec> specs;
        for (size_t i = 0; i < array->size(); ++i) {
            const std::string at = where + key + "[" + std::to_string(i) + "].";
            const toml::table *entry = (*array)[i].as_table();
            if (entry == nullptr) fail(at + " is not a table");
            check_keys(*entry, {"name", "shape"}, at);
            TensorSpec spec;
            spec.name = require_string(*entry, "name", at);
            const toml::array *shape = (*entry)["shape"].as_array();
            if (shape == nullptr || shape->empty()) fail(at + "shape is missing or empty");
            for (size_t d = 0; d < shape->size(); ++d) {
                const auto dim = (*shape)[d].value<int64_t>();
                if (!dim || *dim <= 0) fail(at + "shape has a non-positive or non-integer size");
                spec.shape.push_back(static_cast<int>(*dim));
            }
            specs.push_back(std::move(spec));
        }
        return specs;
    }

   private:
    fs::path directory_;
};

void parse_network_bundle(const ManifestReader &reader, const toml::table &manifest,
                          PolicyBundle &bundle) {
    if (manifest.contains("constant")) {
        reader.fail("[constant] is for CONSTANT bundles; this one is NETWORK");
    }

    const toml::table &decoder = reader.require_table(manifest, "decoder", "");
    reader.check_keys(decoder, {"kind", "scale"}, "decoder.");
    DecoderSpec decoder_spec;
    decoder_spec.kind = reader.require_enum<ActionDecoderKind>(decoder, "kind", "decoder.");
    decoder_spec.scale = reader.require_numbers(decoder, "scale", "decoder.");
    bundle.decoder = decoder_spec;

    const toml::array *networks = manifest["networks"].as_array();
    if (networks == nullptr || networks->empty()) {
        reader.fail("[[networks]] is missing; a NETWORK bundle needs at least the actor");
    }
    for (size_t i = 0; i < networks->size(); ++i) {
        const std::string at = "networks[" + std::to_string(i) + "].";
        const toml::table *entry = (*networks)[i].as_table();
        if (entry == nullptr) reader.fail(at + " is not a table");
        reader.check_keys(*entry, {"name", "runtime", "file", "inputs", "outputs"}, at);
        NetworkSpec network;
        network.name = reader.require_string(*entry, "name", at);
        network.runtime = reader.require_enum<NetworkRuntimeKind>(*entry, "runtime", at);
        network.file = reader.require_file(*entry, "file", at);
        network.inputs = reader.require_tensors(*entry, "inputs", at);
        network.outputs = reader.require_tensors(*entry, "outputs", at);
        bundle.networks.push_back(std::move(network));
    }

    const NetworkSpec &actor = bundle.networks.front();
    if (actor.outputs.size() != 1) {
        reader.fail("the actor (" + actor.name + ") has outputs " + specs_to_string(actor.outputs) +
                    "; it must have exactly one, the action");
    }
    if (actor.outputs.front().size() != kActionSize) {
        reader.fail("the actor's output " + actor.outputs.front().to_string() +
                    " is not [linear, angular]: the action must have 2 values");
    }
    if (decoder_spec.scale.size() != actor.outputs.front().size()) {
        reader.fail("decoder.scale has " + std::to_string(decoder_spec.scale.size()) +
                    " values but the actor's output " + actor.outputs.front().to_string() +
                    " has " + std::to_string(actor.outputs.front().size()));
    }

    const toml::table &selftest = reader.require_table(manifest, "selftest", "");
    reader.check_keys(selftest, {"file", "tolerance"}, "selftest.");
    SelftestSpec selftest_spec;
    selftest_spec.file = reader.require_file(selftest, "file", "selftest.");
    selftest_spec.tolerance = reader.require_number(selftest, "tolerance", "selftest.");
    if (!(selftest_spec.tolerance > 0.0)) reader.fail("selftest.tolerance must be > 0");
    bundle.selftest = selftest_spec;
}

void parse_constant_bundle(const ManifestReader &reader, const toml::table &manifest,
                           PolicyBundle &bundle) {
    for (const char *key : {"decoder", "networks", "selftest"}) {
        if (manifest.contains(key)) {
            reader.fail(std::string(key) +
                        " is for NETWORK bundles; a CONSTANT policy runs no "
                        "network");
        }
    }
    const toml::table &constant = reader.require_table(manifest, "constant", "");
    reader.check_keys(constant, {"action"}, "constant.");
    std::vector<double> action = reader.require_numbers(constant, "action", "constant.");
    if (action.size() != kActionSize) {
        reader.fail("constant.action has " + std::to_string(action.size()) +
                    " values; it must be [linear, angular]");
    }
    for (double value : action) {
        if (value < -1.0 || value > 1.0) {
            reader.fail("constant.action values must be normalized commands in [-1, 1]");
        }
    }
    bundle.constant_action = std::move(action);
}

}  // namespace

PolicyBundle load_policy_bundle(const std::filesystem::path &directory) {
    const ManifestReader reader(directory);
    const fs::path manifest_path = directory / "manifest.toml";
    if (!fs::is_regular_file(manifest_path)) reader.fail("no manifest.toml");

    toml::table manifest;
    try {
        manifest = toml::parse_file(manifest_path.string());
    } catch (const toml::parse_error &e) {
        reader.fail("manifest.toml does not parse: " + std::string(e.description()));
    }

    PolicyBundle bundle;
    bundle.directory = directory;
    // First, so a newer layout fails on its version rather than on whichever key changed.
    bundle.format_version = static_cast<int>(reader.require_int(manifest, "format_version", ""));
    if (bundle.format_version != kPolicyBundleFormatVersion) {
        reader.fail("format_version " + std::to_string(bundle.format_version) +
                    ", this build reads " + std::to_string(kPolicyBundleFormatVersion));
    }
    reader.check_keys(
        manifest,
        {"format_version", "run_id", "git_hash", "policy_kind", "obs_spec_version", "execution",
         "policy_period_ticks", "decoder", "networks", "selftest", "constant"},
        "");
    bundle.run_id = reader.require_string(manifest, "run_id", "");
    bundle.git_hash = manifest["git_hash"].value_or(std::string());
    bundle.policy_kind = reader.require_enum<PolicyKind>(manifest, "policy_kind", "");
    bundle.obs_spec_version =
        static_cast<int>(reader.require_int(manifest, "obs_spec_version", ""));
    bundle.execution = reader.require_enum<PolicyExecution>(manifest, "execution", "");
    bundle.policy_period_ticks =
        static_cast<int>(reader.require_int(manifest, "policy_period_ticks", ""));
    if (bundle.policy_period_ticks < 1) reader.fail("policy_period_ticks must be >= 1");

    switch (bundle.policy_kind) {
        case PolicyKind::NETWORK:
            parse_network_bundle(reader, manifest, bundle);
            break;
        case PolicyKind::CONSTANT:
            parse_constant_bundle(reader, manifest, bundle);
            break;
    }
    return bundle;
}

}  // namespace auto_battlebot::rl_policy
