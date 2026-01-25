// Auto-Battlebot Simulation System
// Frame header structure for shared memory communication

using System.Runtime.InteropServices;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Header structure for shared memory frame data.
    /// Total size: 128 bytes (aligned for optimal memory access)
    /// Ensure the data layout matches include/shared_memory/simulation/simulation_frame_header.hpp
    /// 
    /// This struct must match the C++ side exactly for binary compatibility.
    /// 
    /// Camera model follows OpenCV/Brown-Conrady convention:
    /// - Intrinsics: fx, fy, cx, cy (pinhole model)
    /// - Distortion: k1, k2, p1, p2, k3 (radial + tangential)
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct FrameHeader
    {
        /// <summary>
        /// Incrementing frame identifier for synchronization.
        /// C++ side uses this to detect new frames.
        /// </summary>
        public ulong FrameId;           // 8 bytes (offset 0)

        /// <summary>
        /// Timestamp in seconds since simulation start (Unity Time.time).
        /// </summary>
        public double Timestamp;         // 8 bytes (offset 8)

        /// <summary>
        /// Image width in pixels.
        /// </summary>
        public int Width;                // 4 bytes (offset 16)

        /// <summary>
        /// Image height in pixels.
        /// </summary>
        public int Height;               // 4 bytes (offset 20)

        /// <summary>
        /// Byte offset from start of shared memory to RGB data.
        /// </summary>
        public int RgbOffset;            // 4 bytes (offset 24)

        /// <summary>
        /// Byte offset from start of shared memory to depth data.
        /// </summary>
        public int DepthOffset;          // 4 bytes (offset 28)

        /// <summary>
        /// Byte offset from start of shared memory to pose data.
        /// </summary>
        public int PoseOffset;           // 4 bytes (offset 32)

        /// <summary>
        /// Which buffer (0 or 1) is currently being written.
        /// Used for double-buffering synchronization.
        /// </summary>
        public int ActiveBuffer;         // 4 bytes (offset 36)

        /// <summary>
        /// Camera intrinsic: focal length in X (pixels).
        /// </summary>
        public double Fx;                // 8 bytes (offset 40)

        /// <summary>
        /// Camera intrinsic: focal length in Y (pixels).
        /// </summary>
        public double Fy;                // 8 bytes (offset 48)

        /// <summary>
        /// Camera intrinsic: principal point X (pixels).
        /// </summary>
        public double Cx;                // 8 bytes (offset 56)

        /// <summary>
        /// Camera intrinsic: principal point Y (pixels).
        /// </summary>
        public double Cy;                // 8 bytes (offset 64)

        /// <summary>
        /// Radial distortion coefficient k1.
        /// </summary>
        public double K1;                // 8 bytes (offset 72)

        /// <summary>
        /// Radial distortion coefficient k2.
        /// </summary>
        public double K2;                // 8 bytes (offset 80)

        /// <summary>
        /// Tangential distortion coefficient p1.
        /// </summary>
        public double P1;                // 8 bytes (offset 88)

        /// <summary>
        /// Tangential distortion coefficient p2.
        /// </summary>
        public double P2;                // 8 bytes (offset 96)

        /// <summary>
        /// Radial distortion coefficient k3.
        /// </summary>
        public double K3;                // 8 bytes (offset 104)

        /// <summary>
        /// Reserved for future use and padding to 128 bytes.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public byte[] Reserved;          // 16 bytes (offset 112)

        /// <summary>
        /// Total header size in bytes.
        /// </summary>
        public const int SIZE = 128;

        /// <summary>
        /// Creates a new FrameHeader with default values (no distortion).
        /// Uses default camera intrinsics based on image dimensions.
        /// </summary>
        public static FrameHeader Create(int width, int height)
        {
            // Default intrinsics: assume ~90° horizontal FOV, no distortion
            double fx = width / 2.0;
            double fy = width / 2.0;  // Assume square pixels
            double cx = width / 2.0;
            double cy = height / 2.0;

            return Create(width, height, fx, fy, cx, cy, 0, 0, 0, 0, 0);
        }

        /// <summary>
        /// Creates a new FrameHeader with specified camera intrinsics (no distortion).
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="fx">Focal length X in pixels.</param>
        /// <param name="fy">Focal length Y in pixels.</param>
        /// <param name="cx">Principal point X in pixels.</param>
        /// <param name="cy">Principal point Y in pixels.</param>
        public static FrameHeader Create(int width, int height, double fx, double fy, double cx, double cy)
        {
            return Create(width, height, fx, fy, cx, cy, 0, 0, 0, 0, 0);
        }

        /// <summary>
        /// Creates a new FrameHeader with full camera model (intrinsics + distortion).
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="fx">Focal length X in pixels.</param>
        /// <param name="fy">Focal length Y in pixels.</param>
        /// <param name="cx">Principal point X in pixels.</param>
        /// <param name="cy">Principal point Y in pixels.</param>
        /// <param name="k1">Radial distortion coefficient k1.</param>
        /// <param name="k2">Radial distortion coefficient k2.</param>
        /// <param name="p1">Tangential distortion coefficient p1.</param>
        /// <param name="p2">Tangential distortion coefficient p2.</param>
        /// <param name="k3">Radial distortion coefficient k3.</param>
        public static FrameHeader Create(int width, int height,
            double fx, double fy, double cx, double cy,
            double k1, double k2, double p1, double p2, double k3)
        {
            var header = new FrameHeader
            {
                FrameId = 0,
                Timestamp = 0.0,
                Width = width,
                Height = height,
                RgbOffset = SIZE,
                ActiveBuffer = 0,
                Fx = fx,
                Fy = fy,
                Cx = cx,
                Cy = cy,
                K1 = k1,
                K2 = k2,
                P1 = p1,
                P2 = p2,
                K3 = k3,
                Reserved = new byte[16]
            };

            // Calculate offsets
            int rgbSize = width * height * 3;    // BGR, 3 bytes per pixel
            int depthSize = width * height * 4;  // float32, 4 bytes per pixel

            header.DepthOffset = header.RgbOffset + rgbSize;
            header.PoseOffset = header.DepthOffset + depthSize;

            return header;
        }

        /// <summary>
        /// Creates intrinsics from Unity camera field of view (no distortion).
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="verticalFovDegrees">Vertical field of view in degrees (Unity Camera.fieldOfView).</param>
        public static FrameHeader CreateFromFov(int width, int height, float verticalFovDegrees)
        {
            // Calculate focal length from vertical FOV
            // fy = height / (2 * tan(fov/2))
            double fovRadians = verticalFovDegrees * System.Math.PI / 180.0;
            double fy = height / (2.0 * System.Math.Tan(fovRadians / 2.0));
            double fx = fy;  // Assume square pixels
            double cx = width / 2.0;
            double cy = height / 2.0;

            return Create(width, height, fx, fy, cx, cy, 0, 0, 0, 0, 0);
        }

        /// <summary>
        /// Pose matrix size in bytes (4x4 matrix of float64).
        /// </summary>
        public const int POSE_SIZE = 128;  // 16 doubles × 8 bytes = 128 bytes

        /// <summary>
        /// Calculates total frame size including header.
        /// </summary>
        public static int CalculateTotalSize(int width, int height)
        {
            int rgbSize = width * height * 3;    // BGR format
            int depthSize = width * height * 4;  // float32

            return SIZE + rgbSize + depthSize + POSE_SIZE;
        }
    }
}
