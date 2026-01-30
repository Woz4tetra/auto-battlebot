// Auto-Battlebot Simulation System
// Camera intrinsics structure for TCP communication
//
// Follows OpenCV/Brown-Conrady camera model convention:
// - Intrinsics: fx, fy, cx, cy (pinhole model)
// - Distortion: k1, k2, p1, p2, k3 (radial + tangential)

using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Camera intrinsic parameters for the simulated camera.
    /// Sent once via TCP when C++ application connects.
    /// 
    /// This struct must match the C++ side exactly for binary compatibility.
    /// Total payload size: 80 bytes (excluding message type header).
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct CameraIntrinsics
    {
        /// <summary>
        /// Image width in pixels.
        /// </summary>
        public int Width;              // 4 bytes (offset 0)

        /// <summary>
        /// Image height in pixels.
        /// </summary>
        public int Height;             // 4 bytes (offset 4)

        /// <summary>
        /// Focal length in X direction (pixels).
        /// </summary>
        public double Fx;              // 8 bytes (offset 8)

        /// <summary>
        /// Focal length in Y direction (pixels).
        /// </summary>
        public double Fy;              // 8 bytes (offset 16)

        /// <summary>
        /// Principal point X coordinate (pixels).
        /// </summary>
        public double Cx;              // 8 bytes (offset 24)

        /// <summary>
        /// Principal point Y coordinate (pixels).
        /// </summary>
        public double Cy;              // 8 bytes (offset 32)

        /// <summary>
        /// Radial distortion coefficient k1.
        /// </summary>
        public double K1;              // 8 bytes (offset 40)

        /// <summary>
        /// Radial distortion coefficient k2.
        /// </summary>
        public double K2;              // 8 bytes (offset 48)

        /// <summary>
        /// Tangential distortion coefficient p1.
        /// </summary>
        public double P1;              // 8 bytes (offset 56)

        /// <summary>
        /// Tangential distortion coefficient p2.
        /// </summary>
        public double P2;              // 8 bytes (offset 64)

        /// <summary>
        /// Radial distortion coefficient k3.
        /// </summary>
        public double K3;              // 8 bytes (offset 72)

        /// <summary>
        /// Total payload size in bytes.
        /// </summary>
        public const int PayloadSize = 80;

        /// <summary>
        /// Creates default intrinsics from image dimensions.
        /// Assumes ~90° horizontal FOV and no distortion.
        /// </summary>
        public static CameraIntrinsics CreateDefault(int width, int height)
        {
            double fx = width / 2.0;
            double fy = width / 2.0;  // Square pixels
            double cx = width / 2.0;
            double cy = height / 2.0;

            return new CameraIntrinsics
            {
                Width = width,
                Height = height,
                Fx = fx,
                Fy = fy,
                Cx = cx,
                Cy = cy,
                K1 = 0, K2 = 0, P1 = 0, P2 = 0, K3 = 0
            };
        }

        /// <summary>
        /// Creates intrinsics from a Unity camera's field of view.
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="verticalFovDegrees">Vertical field of view in degrees.</param>
        public static CameraIntrinsics CreateFromFov(int width, int height, float verticalFovDegrees)
        {
            // fy = height / (2 * tan(fov/2))
            double fovRadians = verticalFovDegrees * Math.PI / 180.0;
            double fy = height / (2.0 * Math.Tan(fovRadians / 2.0));
            double fx = fy;  // Square pixels

            return new CameraIntrinsics
            {
                Width = width,
                Height = height,
                Fx = fx,
                Fy = fy,
                Cx = width / 2.0,
                Cy = height / 2.0,
                K1 = 0, K2 = 0, P1 = 0, P2 = 0, K3 = 0
            };
        }

        /// <summary>
        /// Creates intrinsics from a Unity Camera component.
        /// </summary>
        public static CameraIntrinsics CreateFromCamera(Camera camera, int width, int height)
        {
            return CreateFromFov(width, height, camera.fieldOfView);
        }

        /// <summary>
        /// Creates intrinsics with full parameters.
        /// </summary>
        public static CameraIntrinsics Create(
            int width, int height,
            double fx, double fy, double cx, double cy,
            double k1 = 0, double k2 = 0, double p1 = 0, double p2 = 0, double k3 = 0)
        {
            return new CameraIntrinsics
            {
                Width = width,
                Height = height,
                Fx = fx,
                Fy = fy,
                Cx = cx,
                Cy = cy,
                K1 = k1,
                K2 = k2,
                P1 = p1,
                P2 = p2,
                K3 = k3
            };
        }

        /// <summary>
        /// Validates that intrinsics are reasonable.
        /// </summary>
        public bool IsValid()
        {
            // Check dimensions
            if (Width <= 0 || Height <= 0)
                return false;

            // Check focal lengths (should be positive and reasonable)
            if (Fx <= 0 || Fy <= 0)
                return false;

            // Check principal point (should be within image bounds, approximately)
            if (Cx < 0 || Cx > Width || Cy < 0 || Cy > Height)
                return false;

            // Check for NaN/Infinity
            if (double.IsNaN(Fx) || double.IsInfinity(Fx) ||
                double.IsNaN(Fy) || double.IsInfinity(Fy) ||
                double.IsNaN(Cx) || double.IsInfinity(Cx) ||
                double.IsNaN(Cy) || double.IsInfinity(Cy))
                return false;

            return true;
        }

        /// <summary>
        /// Returns a string representation of the intrinsics.
        /// </summary>
        public override string ToString()
        {
            return $"CameraIntrinsics({Width}x{Height}, fx={Fx:F2}, fy={Fy:F2}, cx={Cx:F2}, cy={Cy:F2})";
        }
    }
}
