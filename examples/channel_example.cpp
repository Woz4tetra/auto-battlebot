#include <core/channel.h>
#include <core/async_channel.h>
#include <iostream>
#include <thread>
#include <chrono>

// Example message types
struct ImageFrame {
    uint64_t timestamp;
    int width;
    int height;
    std::vector<uint8_t> data;
};

struct Detection {
    std::string label;
    float confidence;
    int x, y, w, h;
};

// Example Camera Module
class CameraModule {
public:
    // Camera owns the output channel
    Channel<ImageFrame> frame_output;

    void start() {
        running_ = true;
        capture_thread_ = std::thread([this] { captureLoop(); });
    }

    void stop() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    void captureLoop() {
        int frame_count = 0;
        while (running_) {
            // Simulate camera capture
            ImageFrame frame;
            frame.timestamp = frame_count++;
            frame.width = 1920;
            frame.height = 1080;
            frame.data.resize(100);  // Dummy data
            
            // Send frame through channel
            bool delivered = frame_output.send(frame);
            if (!delivered) {
                std::cout << "Warning: No receiver connected for frame " 
                         << frame.timestamp << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 fps
        }
    }

    std::thread capture_thread_;
    std::atomic<bool> running_{false};
};

// Example Vision Module using sync channel (fast processing)
class VisionModuleSync {
public:
    void connect_camera(Channel<ImageFrame>& camera_output) {
        camera_output.connect([this](const ImageFrame& frame) {
            processFrame(frame);
        });
    }

    Channel<Detection> detection_output;

private:
    void processFrame(const ImageFrame& frame) {
        // Fast processing happens in camera's thread
        std::cout << "Processing frame " << frame.timestamp 
                  << " (sync, in camera thread)" << std::endl;
        
        // Create a detection
        Detection det{"robot", 0.95f, 100, 100, 50, 50};
        detection_output.send(det);
    }
};

// Example Vision Module using async channel (heavy processing)
class VisionModuleAsync {
public:
    void connect_camera(Channel<ImageFrame>& camera_output) {
        // Use internal async channel to avoid blocking camera
        async_input_.connect([this](const ImageFrame& frame) {
            processFrame(frame);
        });
        
        // Connect camera to our async input
        camera_output.connect([this](const ImageFrame& frame) {
            async_input_.send(frame);
        });
    }

    Channel<Detection> detection_output;

private:
    void processFrame(const ImageFrame& frame) {
        // Heavy processing in dedicated thread
        std::cout << "Processing frame " << frame.timestamp 
                  << " (async, in vision thread)" << std::endl;
        
        // Simulate heavy processing
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        Detection det{"robot", 0.95f, 100, 100, 50, 50};
        detection_output.send(det);
    }

    AsyncChannel<ImageFrame> async_input_;
};

// Example Control Module
class ControlModule {
public:
    void connect_vision(Channel<Detection>& vision_output) {
        vision_output.connect([this](const Detection& det) {
            handleDetection(det);
        });
    }

private:
    void handleDetection(const Detection& det) {
        std::cout << "Received detection: " << det.label 
                  << " (" << det.confidence << ")" << std::endl;
    }
};

int main() {
    std::cout << "=== Sync Channel Example ===" << std::endl;
    {
        CameraModule camera;
        VisionModuleSync vision;
        ControlModule control;
        
        // Wire up the modules using channels
        vision.connect_camera(camera.frame_output);
        control.connect_vision(vision.detection_output);
        
        camera.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        camera.stop();
    }
    
    std::cout << "\n=== Async Channel Example ===" << std::endl;
    {
        CameraModule camera;
        VisionModuleAsync vision;
        ControlModule control;
        
        // Wire up the modules using channels
        vision.connect_camera(camera.frame_output);
        control.connect_vision(vision.detection_output);
        
        camera.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        camera.stop();
    }
    
    return 0;
}
