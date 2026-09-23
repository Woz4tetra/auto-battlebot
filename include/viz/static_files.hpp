#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

// Static file serving for the web dashboard in viz_relay. Header-only because viz_relay does not
// link auto_battlebot_lib and the tests do.
namespace auto_battlebot::viz {

/** Decodes %XX escapes. Returns nullopt on a malformed escape or an encoded NUL. */
inline std::optional<std::string> percent_decode(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] != '%') {
            out.push_back(text[i]);
            continue;
        }
        if (i + 2 >= text.size()) return std::nullopt;
        const int hi = hex(text[i + 1]);
        const int lo = hex(text[i + 2]);
        if (hi < 0 || lo < 0) return std::nullopt;
        const char c = static_cast<char>(hi * 16 + lo);
        if (c == '\0') return std::nullopt;
        out.push_back(c);
        i += 2;
    }
    return out;
}

/**
 * Maps a URL path to a regular file under `root`, or nullopt. `/` maps to `index.html`.
 *
 * Rejects `..` and `.` segments (also when percent-encoded), backslashes, empty segments such as
 * `//etc/passwd`, and anything whose real path lands outside the root, which catches symlinks
 * that point out of it. A query string must already be stripped.
 */
inline std::optional<std::filesystem::path> resolve_static_path(const std::filesystem::path &root,
                                                                std::string_view url_path) {
    auto decoded = percent_decode(url_path);
    if (!decoded) return std::nullopt;
    std::string_view rel = *decoded;
    if (!rel.starts_with('/')) return std::nullopt;
    rel.remove_prefix(1);
    if (rel.empty()) rel = "index.html";
    if (rel.find('\\') != std::string_view::npos) return std::nullopt;

    std::filesystem::path candidate = root;
    size_t start = 0;
    while (start <= rel.size()) {
        const size_t end = rel.find('/', start);
        const std::string_view segment =
            rel.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
        if (segment.empty() || segment == "." || segment == "..") return std::nullopt;
        candidate /= std::string(segment);
        if (end == std::string_view::npos) break;
        start = end + 1;
    }

    std::error_code ec;
    const auto real_root = std::filesystem::canonical(root, ec);
    if (ec) return std::nullopt;
    const auto real = std::filesystem::canonical(candidate, ec);
    if (ec) return std::nullopt;
    auto r = real_root.begin();
    auto c = real.begin();
    for (; r != real_root.end(); ++r, ++c) {
        if (c == real.end() || *c != *r) return std::nullopt;
    }
    if (!std::filesystem::is_regular_file(real, ec)) return std::nullopt;
    return real;
}

/** Content-Type for the files Vite emits. */
inline std::string_view content_type_for(const std::filesystem::path &path) {
    const std::string ext = path.extension().string();
    if (ext == ".html") return "text/html; charset=utf-8";
    if (ext == ".js" || ext == ".mjs") return "text/javascript; charset=utf-8";
    if (ext == ".css") return "text/css; charset=utf-8";
    if (ext == ".json") return "application/json";
    if (ext == ".webmanifest") return "application/manifest+json";
    if (ext == ".svg") return "image/svg+xml";
    if (ext == ".png") return "image/png";
    if (ext == ".ico") return "image/x-icon";
    if (ext == ".woff2") return "font/woff2";
    if (ext == ".woff") return "font/woff";
    if (ext == ".ttf") return "font/ttf";
    if (ext == ".txt") return "text/plain; charset=utf-8";
    return "application/octet-stream";
}

}  // namespace auto_battlebot::viz
