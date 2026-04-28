#pragma once

#include <miniros/node_handle.h>
#include <miniros/publisher.h>

#include <string>

namespace auto_battlebot {

/** Type-safe wrapper around miniros::Publisher.
 *  Locks the message type at compile time so publish() only accepts M. */
template <typename M>
class TypedPublisher {
   public:
    TypedPublisher() = default;
    explicit TypedPublisher(miniros::Publisher pub) : pub_(std::move(pub)) {}

    static TypedPublisher advertise(miniros::NodeHandle& nh, const std::string& topic,
                                    uint32_t queue_size, bool latch = false) {
        return TypedPublisher(nh.advertise<M>(topic, queue_size, latch));
    }

    void publish(const M& msg) const { pub_.publish(msg); }

    bool is_valid() const { return pub_.isValid(); }
    explicit operator bool() const { return is_valid(); }
    std::string topic() const { return pub_.getTopic(); }

   private:
    miniros::Publisher pub_;
};

}  // namespace auto_battlebot
