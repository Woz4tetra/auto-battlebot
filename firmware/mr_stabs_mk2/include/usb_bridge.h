#pragma once

// Serves the diagnostics page over the USB serial port, for when WiFi is not reachable.
// scripts/usb_dashboard.py on the laptop listens on localhost and sends each browser request down
// the serial line; this task replays it against the robot's own web server on 127.0.0.1 and sends
// the raw HTTP response back. Log lines keep flowing on the same port between frames.
//
// Frames are single lines. Each data frame carries a CRC32 of its bytes, and the laptop retries a
// GET whose response arrives damaged.
//   laptop -> robot   "\x1eQ <id> <base64 request>\n"   open a connection and send the request
//                     "\x1eC <id>\n"                    the browser went away, close it
//   robot -> laptop   "\x1eD <id> <base64 bytes> <crc32>\n"  response bytes, in order
//                     "\x1eE <id>\n"                    the connection closed
namespace usb_bridge {

// Starts the bridge task. Call after Serial.begin().
void begin();

}  // namespace usb_bridge
