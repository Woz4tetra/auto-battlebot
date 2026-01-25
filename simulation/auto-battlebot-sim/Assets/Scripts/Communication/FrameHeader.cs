// Auto-Battlebot Simulation System
// Frame header structure for shared memory communication

using System.Runtime.InteropServices;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Header structure for shared memory frame data.
    /// Total size: 64 bytes (aligned for optimal memory access)
    /// 
    /// This struct must match the C++ side exactly for binary compatibility.
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
        /// Reserved for future use and padding to 64 bytes.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 24)]
        public byte[] Reserved;          // 24 bytes (offset 40)

        /// <summary>
        /// Total header size in bytes.
        /// </summary>
        public const int SIZE = 64;

        /// <summary>
        /// Creates a new FrameHeader with default values.
        /// </summary>
        public static FrameHeader Create(int width, int height)
        {
            var header = new FrameHeader
            {
                FrameId = 0,
                Timestamp = 0.0,
                Width = width,
                Height = height,
                RgbOffset = SIZE,
                ActiveBuffer = 0,
                Reserved = new byte[24]
            };

            // Calculate offsets
            int rgbSize = width * height * 3;    // BGR, 3 bytes per pixel
            int depthSize = width * height * 4;  // float32, 4 bytes per pixel

            header.DepthOffset = header.RgbOffset + rgbSize;
            header.PoseOffset = header.DepthOffset + depthSize;

            return header;
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
