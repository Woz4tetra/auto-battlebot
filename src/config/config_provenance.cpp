#include "config/config_provenance.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <regex>
#include <sstream>
#include <unordered_map>

namespace auto_battlebot {
namespace {
thread_local ConfigDefaultsRecorder *active_recorder = nullptr;

bool is_bare_key(const std::string &key) {
    return !key.empty() && std::all_of(key.begin(), key.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
    });
}

std::string quote_key(const std::string &key) {
    if (is_bare_key(key)) {
        return key;
    }
    std::ostringstream os;
    os << toml::value<std::string>(key);
    return os.str();
}

std::string join_section(const std::string &parent, const std::string &key) {
    return parent.empty() ? quote_key(key) : parent + "." + quote_key(key);
}

bool is_array_of_tables(const toml::array &array) {
    return !array.empty() && std::all_of(array.begin(), array.end(),
                                         [](const toml::node &node) { return node.is_table(); });
}

std::string section_header(const std::string &section) {
    static const std::regex index_suffix(R"(\[\d+\])");
    const bool array_element = !section.empty() && section.back() == ']';
    const std::string bare = std::regex_replace(section, index_suffix, "");
    return array_element ? "[[" + bare + "]]" : "[" + bare + "]";
}

class Resolver {
   public:
    explicit Resolver(const std::filesystem::path &config_root)
        : config_root_(std::filesystem::weakly_canonical(config_root)) {}

    void walk(const toml::table &table, const std::string &name) {
        const size_t index = add_section(name, &table);
        for (auto &&[key, node] : table) {
            const std::string key_str(key.str());
            if (const toml::table *sub = node.as_table()) {
                walk(*sub, join_section(name, key_str));
                continue;
            }
            const toml::array *array = node.as_array();
            if (array && is_array_of_tables(*array)) {
                for (size_t i = 0; i < array->size(); ++i) {
                    walk(*array->get(i)->as_table(),
                         join_section(name, key_str) + "[" + std::to_string(i) + "]");
                }
                continue;
            }
            sections_[index].fields.push_back(
                {name, key_str, format_toml_value(node), source_of(node)});
        }
    }

    void add_default(const toml::table *table, const std::string &section_name,
                     const std::string &key, const std::string &value) {
        size_t index = 0;
        if (auto it = by_table_.find(table); it != by_table_.end()) {
            index = it->second;
        } else if (auto named = by_name_.find(section_name); named != by_name_.end()) {
            index = named->second;
        } else {
            index = add_section(section_name, nullptr);
        }
        auto &fields = sections_[index].fields;
        const bool present = std::any_of(fields.begin(), fields.end(),
                                         [&](const ResolvedField &f) { return f.key == key; });
        if (!present) {
            fields.push_back({sections_[index].name, key, value, kDefaultSource});
        }
    }

    ResolvedConfig finish() {
        ResolvedConfig resolved;
        for (auto &section : sections_) {
            std::stable_sort(section.fields.begin(), section.fields.end(),
                             [](const ResolvedField &a, const ResolvedField &b) {
                                 if ((a.key == "type") != (b.key == "type")) {
                                     return a.key == "type";
                                 }
                                 return a.key < b.key;
                             });
            for (auto &field : section.fields) {
                resolved.fields.push_back(std::move(field));
            }
        }
        return resolved;
    }

   private:
    struct Section {
        std::string name;
        std::vector<ResolvedField> fields;
    };

    size_t add_section(const std::string &name, const toml::table *table) {
        if (auto it = by_name_.find(name); it != by_name_.end()) {
            if (table) {
                by_table_[table] = it->second;
            }
            return it->second;
        }
        sections_.push_back({name, {}});
        const size_t index = sections_.size() - 1;
        by_name_[name] = index;
        if (table) {
            by_table_[table] = index;
        }
        return index;
    }

    std::string source_of(const toml::node &node) const {
        const toml::source_region &source = node.source();
        if (!source.path) {
            return "unknown";
        }
        if (!std::filesystem::is_regular_file(*source.path)) {
            return *source.path;
        }
        const std::filesystem::path file = std::filesystem::weakly_canonical(*source.path);
        std::filesystem::path shown = file.lexically_relative(config_root_);
        if (shown.empty() || *shown.begin() == "..") {
            shown = file;
        }
        return shown.generic_string() + ":" + std::to_string(source.begin.line);
    }

    std::filesystem::path config_root_;
    std::vector<Section> sections_;
    std::unordered_map<std::string, size_t> by_name_;
    std::unordered_map<const toml::table *, size_t> by_table_;
};
}  // namespace

const ResolvedField *ResolvedConfig::find(const std::string &section,
                                          const std::string &key) const {
    for (const auto &field : fields) {
        if (field.section == section && field.key == key) {
            return &field;
        }
    }
    return nullptr;
}

std::string format_double(double value) {
    if (!std::isfinite(value)) {
        return std::isnan(value) ? "nan" : (value > 0 ? "inf" : "-inf");
    }
    std::array<char, 32> buffer{};
    auto [end, ec] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    std::string text(buffer.data(), end);
    if (text.find_first_of(".eE") == std::string::npos) {
        text += ".0";
    }
    return text;
}

std::string format_toml_value(const toml::node &node) {
    if (const auto *number = node.as_floating_point()) {
        return format_double(number->get());
    }
    if (const toml::array *array = node.as_array()) {
        std::string text = "[";
        for (size_t i = 0; i < array->size(); ++i) {
            text += (i == 0 ? "" : ", ") + format_toml_value(*array->get(i));
        }
        return text + "]";
    }
    std::ostringstream os;
    node.visit([&](auto &&typed) { os << typed; });
    return os.str();
}

std::string format_resolved_toml(const ResolvedConfig &resolved) {
    std::ostringstream os;
    size_t i = 0;
    const auto &fields = resolved.fields;
    while (i < fields.size()) {
        const std::string &section = fields[i].section;
        size_t end = i;
        size_t width = 0;
        while (end < fields.size() && fields[end].section == section) {
            width =
                std::max(width, quote_key(fields[end].key).size() + 3 + fields[end].value.size());
            ++end;
        }
        width = std::min<size_t>(width, 60);
        if (!section.empty()) {
            if (os.tellp() > 0) {
                os << "\n";
            }
            os << section_header(section) << "\n";
        }
        for (; i < end; ++i) {
            std::string line = fields[i].value == kUnsetValue
                                   ? "# " + quote_key(fields[i].key) + " unset"
                                   : quote_key(fields[i].key) + " = " + fields[i].value;
            if (line.size() < width) {
                line.append(width - line.size(), ' ');
            }
            os << line << "  # " << fields[i].source << "\n";
        }
    }
    return os.str();
}

ConfigDefaultsRecorder::ConfigDefaultsRecorder() : previous_(active_recorder) {
    active_recorder = this;
}

ConfigDefaultsRecorder::~ConfigDefaultsRecorder() { active_recorder = previous_; }

void ConfigDefaultsRecorder::record(const toml::table &table, const std::string &section_name,
                                    const std::string &key, std::string value) {
    defaults_.push_back({&table, section_name, key, std::move(value)});
}

ResolvedConfig ConfigDefaultsRecorder::resolve(const toml::table &root,
                                               const std::filesystem::path &config_root) const {
    Resolver resolver(config_root);
    resolver.walk(root, "");
    for (const auto &recorded : defaults_) {
        resolver.add_default(recorded.table, recorded.section_name, recorded.key, recorded.value);
    }
    return resolver.finish();
}

void record_config_default(const toml::table &table, const std::string &section_name,
                           const std::string &key, std::string value) {
    if (active_recorder) {
        active_recorder->record(table, section_name, key, std::move(value));
    }
}
}  // namespace auto_battlebot
