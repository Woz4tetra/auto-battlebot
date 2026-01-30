// Auto-Battlebot Simulation System
// TCP message types and protocol definitions for CUDA Interop communication
//
// Protocol Overview:
// - All messages are fixed-size for efficient parsing
// - Little-endian byte order throughout
// - Unity acts as TCP server, C++ application connects as client
// - Camera intrinsics sent once on connection
// - Frame data (pose + sync) sent each frame
// - Velocity commands received asynchronously

using System;
using System.Runtime.InteropServices;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Message types for TCP communication protocol.
    /// First byte of each message indicates the type.
    /// </summary>
    public enum TcpMessageType : byte
    {
        /// <summary>
        /// Frame ready with pose data (Unity → C++).
        /// Payload: frame_id (8) + timestamp_ns (8) + pose (128) = 144 bytes
        /// </summary>
        FrameReady = 0x01,

        /// <summary>
        /// Velocity command (C++ → Unity).
        /// Payload: command_id (8) + linear_x (8) + linear_y (8) + angular_z (8) = 32 bytes
        /// </summary>
        VelocityCommand = 0x02,

        /// <summary>
        /// Camera intrinsics (Unity → C++).
        /// Payload: width (4) + height (4) + fx,fy,cx,cy (32) + distortion (40) = 80 bytes
        /// </summary>
        CameraIntrinsics = 0x03,

        /// <summary>
        /// Frame processed acknowledgment (C++ → Unity).
        /// Payload: frame_id (8) = 8 bytes
        /// </summary>
        FrameProcessed = 0x04,

        /// <summary>
        /// Ping for connection check.
        /// Payload: none
        /// </summary>
        Ping = 0x05,

        /// <summary>
        /// Pong response to ping.
        /// Payload: none
        /// </summary>
        Pong = 0x06,

        /// <summary>
        /// Frame ready without depth (Unity → C++).
        /// Same payload as FrameReady.
        /// </summary>
        FrameReadyNoDepth = 0x07,

        /// <summary>
        /// Request frame from Unity (C++ → Unity).
        /// Payload: with_depth (1) = 1 byte
        /// </summary>
        RequestFrame = 0x08,

        /// <summary>
        /// Shutdown notification.
        /// Payload: none
        /// </summary>
        Shutdown = 0xFF,
    }

    /// <summary>
    /// Message sizes for each type (including the 1-byte type header).
    /// </summary>
    public static class TcpMessageSizes
    {
        public const int TypeHeader = 1;
        public const int FrameReady = TypeHeader + 8 + 8 + 128;  // 145 bytes
        public const int VelocityCommand = TypeHeader + 8 + 8 + 8 + 8;  // 33 bytes
        public const int CameraIntrinsics = TypeHeader + 4 + 4 + 32 + 40;  // 81 bytes
        public const int FrameProcessed = TypeHeader + 8;  // 9 bytes
        public const int Ping = TypeHeader;  // 1 byte
        public const int Pong = TypeHeader;  // 1 byte
        public const int RequestFrame = TypeHeader + 1;  // 2 bytes
        public const int Shutdown = TypeHeader;  // 1 byte
    }

    /// <summary>
    /// Frame ready message structure.
    /// Sent from Unity to C++ after each frame renders.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct FrameReadyMessage
    {
        /// <summary>
        /// Incrementing frame identifier.
        /// </summary>
        public ulong FrameId;

        /// <summary>
        /// Timestamp in nanoseconds since simulation start.
        /// </summary>
        public ulong TimestampNs;

        /// <summary>
        /// Camera pose as 4x4 matrix (row-major, 16 doubles).
        /// Transform from camera to world coordinates.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public double[] Pose;

        /// <summary>
        /// Total size in bytes (excluding type header).
        /// </summary>
        public const int PayloadSize = 8 + 8 + 128;

        /// <summary>
        /// Creates a new FrameReadyMessage.
        /// </summary>
        public static FrameReadyMessage Create(ulong frameId, double timestampSeconds, double[] pose)
        {
            if (pose == null || pose.Length != 16)
            {
                throw new ArgumentException("Pose must be a 16-element array (4x4 matrix)");
            }

            return new FrameReadyMessage
            {
                FrameId = frameId,
                TimestampNs = (ulong)(timestampSeconds * 1_000_000_000),
                Pose = pose
            };
        }

        /// <summary>
        /// Creates a new FrameReadyMessage from Unity Matrix4x4.
        /// </summary>
        public static FrameReadyMessage Create(ulong frameId, double timestampSeconds, UnityEngine.Matrix4x4 pose)
        {
            // Convert Unity Matrix4x4 to row-major double array
            var poseArray = new double[16];
            for (int row = 0; row < 4; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    poseArray[row * 4 + col] = pose[row, col];
                }
            }

            return Create(frameId, timestampSeconds, poseArray);
        }
    }

    /// <summary>
    /// Frame processed acknowledgment structure.
    /// Sent from C++ to Unity after processing a frame.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct FrameProcessedMessage
    {
        /// <summary>
        /// Frame ID that was processed.
        /// </summary>
        public ulong FrameId;

        /// <summary>
        /// Payload size in bytes.
        /// </summary>
        public const int PayloadSize = 8;
    }

    /// <summary>
    /// Request frame message structure.
    /// Sent from C++ to Unity to request a new frame.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct RequestFrameMessage
    {
        /// <summary>
        /// Whether to include depth in the frame.
        /// </summary>
        [MarshalAs(UnmanagedType.U1)]
        public bool WithDepth;

        /// <summary>
        /// Payload size in bytes.
        /// </summary>
        public const int PayloadSize = 1;
    }
}
