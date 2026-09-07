#pragma once

#include <foxglove/schema.hpp>
#include <string>

namespace auto_battlebot {

/** Schema attached to a channel, owned as strings so it can be sent over the relay socket and
 *  handed to the MCAP writer alike. `data` is the serialized schema: a protobuf
 *  FileDescriptorSet for `protobuf`, jsonschema text for `jsonschema`. */
struct VizSchema {
    std::string name;
    std::string encoding;
    std::string data;

    static VizSchema from_sdk(const foxglove::Schema& schema) {
        return VizSchema{schema.name, schema.encoding,
                         std::string(reinterpret_cast<const char*>(schema.data), schema.data_len)};
    }
    static VizSchema jsonschema(std::string name, std::string text) {
        return VizSchema{std::move(name), "jsonschema", std::move(text)};
    }
    foxglove::Schema to_sdk() const {
        return foxglove::Schema{name, encoding, reinterpret_cast<const std::byte*>(data.data()),
                                data.size()};
    }
};

}  // namespace auto_battlebot
