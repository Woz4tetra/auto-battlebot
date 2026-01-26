// Auto-Battlebot Simulation System
// High-performance shared memory writer for IPC with C++ application

using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// High-performance shared memory writer that writes image data (RGB and depth)
    /// to a memory-mapped file accessible by the C++ application.
    /// 
    /// Features:
    /// - Double-buffering to prevent tearing/partial reads
    /// - Platform abstraction for Linux (/dev/shm/) and Windows (named memory)
    /// - Performance metrics tracking
    /// - IDisposable pattern for proper cleanup
    /// 
    /// Note on Double Buffering:
    /// Double buffering prevents the C++ reader from seeing partially-written frames
    /// by maintaining two separate buffers and swapping between them. However, when
    /// using the SyncSocket (SIM-004) in lockstep mode, double buffering becomes
    /// redundant because C++ waits for a "frame ready" signal before reading.
    /// 
    /// Recommendations:
    /// - Lockstep mode with SyncSocket: Can disable double buffering to save ~6MB RAM
    /// - Free-running mode (no sync): Keep double buffering enabled
    /// - Development/testing: Keep enabled as a safety fallback
    /// </summary>
    public class SharedMemoryWriter : IDisposable
    {
        #region Constants

        /// <summary>
        /// Default shared memory name for the frame buffer.
        /// </summary>
        public const string DEFAULT_MEMORY_NAME = "auto_battlebot_frame";

        /// <summary>
        /// Linux shared memory directory.
        /// </summary>
        private const string LINUX_SHM_PATH = "/dev/shm/";

        #endregion

        #region Private Fields

        private readonly int _width;
        private readonly int _height;
        private readonly string _memoryName;
        private readonly bool _useDoubleBuffering;

        private MemoryMappedFile _memoryMappedFile;
        private MemoryMappedViewAccessor _accessor;
        private FileStream _linuxFileStream;  // Keep reference to prevent GC on Linux

        private ulong _frameId = 0;
        private int _activeBuffer = 0;

        private bool _isDisposed = false;
        private bool _isInitialized = false;

        // Performance metrics
        private readonly Stopwatch _writeStopwatch = new Stopwatch();
        private long _totalWriteTimeMs = 0;
        private long _writeCount = 0;
        private long _maxWriteTimeMs = 0;

        // Pre-calculated sizes and offsets
        private readonly int _totalSize;
        private readonly int _singleBufferSize;
        private readonly int _rgbSize;
        private readonly int _depthSize;
        private readonly int _poseSize;

        // Cached header for writing
        private FrameHeader _header;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the shared memory has been successfully initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Current frame ID (increments with each write).
        /// </summary>
        public ulong CurrentFrameId => _frameId;

        /// <summary>
        /// Average write time in milliseconds.
        /// </summary>
        public double AverageWriteTimeMs => _writeCount > 0 ? (double)_totalWriteTimeMs / _writeCount : 0;

        /// <summary>
        /// Maximum write time observed in milliseconds.
        /// </summary>
        public long MaxWriteTimeMs => _maxWriteTimeMs;

        /// <summary>
        /// Total number of frames written.
        /// </summary>
        public long WriteCount => _writeCount;

        /// <summary>
        /// Estimated throughput in MB/s based on recent writes.
        /// </summary>
        public double ThroughputMBps
        {
            get
            {
                if (AverageWriteTimeMs <= 0) return 0;
                double bytesPerSecond = _singleBufferSize / (AverageWriteTimeMs / 1000.0);
                return bytesPerSecond / (1024 * 1024);
            }
        }

        #endregion

        #region Constructor

        /// <summary>
        /// Creates a new SharedMemoryWriter.
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="memoryName">Name for the shared memory region.</param>
        /// <param name="useDoubleBuffering">
        /// Enable double-buffering to prevent tearing. Can be disabled when using
        /// SyncSocket in lockstep mode, as synchronization prevents race conditions.
        /// </param>
        /// <summary>
        /// Creates a new SharedMemoryWriter with default camera intrinsics.
        /// </summary>
        public SharedMemoryWriter(int width, int height, string memoryName = null, bool useDoubleBuffering = true)
            : this(width, height, 0, 0, 0, 0, memoryName, useDoubleBuffering, useDefaultIntrinsics: true)
        {
        }

        /// <summary>
        /// Creates a new SharedMemoryWriter with specified camera intrinsics.
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="fx">Focal length X in pixels.</param>
        /// <param name="fy">Focal length Y in pixels.</param>
        /// <param name="cx">Principal point X in pixels.</param>
        /// <param name="cy">Principal point Y in pixels.</param>
        /// <param name="memoryName">Name for the shared memory region.</param>
        /// <param name="useDoubleBuffering">Enable double-buffering to prevent tearing.</param>
        public SharedMemoryWriter(int width, int height, double fx, double fy, double cx, double cy,
            string memoryName = null, bool useDoubleBuffering = true)
            : this(width, height, fx, fy, cx, cy, memoryName, useDoubleBuffering, useDefaultIntrinsics: false)
        {
        }

        private SharedMemoryWriter(int width, int height, double fx, double fy, double cx, double cy,
            string memoryName, bool useDoubleBuffering, bool useDefaultIntrinsics)
        {
            _width = width;
            _height = height;
            _memoryName = memoryName ?? DEFAULT_MEMORY_NAME;
            _useDoubleBuffering = useDoubleBuffering;

            // Calculate sizes
            _rgbSize = width * height * 3;      // BGR format
            _depthSize = width * height * 4;    // float32
            _poseSize = FrameHeader.POSE_SIZE;  // 4x4 matrix of float64 (128 bytes)

            _singleBufferSize = FrameHeader.SIZE + _rgbSize + _depthSize + _poseSize;
            _totalSize = _useDoubleBuffering ? _singleBufferSize * 2 : _singleBufferSize;

            // Initialize header template with intrinsics
            if (useDefaultIntrinsics)
            {
                _header = FrameHeader.Create(width, height);
            }
            else
            {
                _header = FrameHeader.Create(width, height, fx, fy, cx, cy);
            }
        }

        #endregion

        #region Factory Methods

        /// <summary>
        /// Creates a SharedMemoryWriter with camera intrinsics calculated from Unity camera FOV.
        /// </summary>
        /// <param name="width">Image width in pixels.</param>
        /// <param name="height">Image height in pixels.</param>
        /// <param name="verticalFovDegrees">Vertical field of view in degrees (Unity Camera.fieldOfView).</param>
        /// <param name="memoryName">Name for the shared memory region.</param>
        /// <param name="useDoubleBuffering">Enable double-buffering to prevent tearing.</param>
        public static SharedMemoryWriter CreateFromFov(int width, int height, float verticalFovDegrees,
            string memoryName = null, bool useDoubleBuffering = true)
        {
            // Calculate focal length from vertical FOV
            // fy = height / (2 * tan(fov/2))
            double fovRadians = verticalFovDegrees * System.Math.PI / 180.0;
            double fy = height / (2.0 * System.Math.Tan(fovRadians / 2.0));
            double fx = fy;  // Assume square pixels
            double cx = width / 2.0;
            double cy = height / 2.0;

            return new SharedMemoryWriter(width, height, fx, fy, cx, cy, memoryName, useDoubleBuffering);
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Updates the camera intrinsics (no distortion). Call before Initialize() or the intrinsics
        /// will be written with the next frame.
        /// </summary>
        /// <param name="fx">Focal length X in pixels.</param>
        /// <param name="fy">Focal length Y in pixels.</param>
        /// <param name="cx">Principal point X in pixels.</param>
        /// <param name="cy">Principal point Y in pixels.</param>
        public void SetIntrinsics(double fx, double fy, double cx, double cy)
        {
            _header.Fx = fx;
            _header.Fy = fy;
            _header.Cx = cx;
            _header.Cy = cy;
        }

        /// <summary>
        /// Updates the camera intrinsics from Unity camera FOV.
        /// </summary>
        /// <param name="verticalFovDegrees">Vertical field of view in degrees.</param>
        public void SetIntrinsicsFromFov(float verticalFovDegrees)
        {
            double fovRadians = verticalFovDegrees * System.Math.PI / 180.0;
            double fy = _height / (2.0 * System.Math.Tan(fovRadians / 2.0));
            SetIntrinsics(fy, fy, _width / 2.0, _height / 2.0);
        }

        /// <summary>
        /// Gets the current camera intrinsics.
        /// </summary>
        public (double fx, double fy, double cx, double cy) GetIntrinsics()
        {
            return (_header.Fx, _header.Fy, _header.Cx, _header.Cy);
        }

        /// <summary>
        /// Sets the distortion coefficients (Brown-Conrady model).
        /// </summary>
        /// <param name="k1">Radial distortion coefficient k1.</param>
        /// <param name="k2">Radial distortion coefficient k2.</param>
        /// <param name="p1">Tangential distortion coefficient p1.</param>
        /// <param name="p2">Tangential distortion coefficient p2.</param>
        /// <param name="k3">Radial distortion coefficient k3.</param>
        public void SetDistortion(double k1, double k2, double p1, double p2, double k3)
        {
            _header.K1 = k1;
            _header.K2 = k2;
            _header.P1 = p1;
            _header.P2 = p2;
            _header.K3 = k3;
        }

        /// <summary>
        /// Gets the current distortion coefficients.
        /// </summary>
        public (double k1, double k2, double p1, double p2, double k3) GetDistortion()
        {
            return (_header.K1, _header.K2, _header.P1, _header.P2, _header.K3);
        }

        /// <summary>
        /// Sets the full camera model (intrinsics + distortion).
        /// </summary>
        public void SetCameraModel(double fx, double fy, double cx, double cy,
            double k1, double k2, double p1, double p2, double k3)
        {
            SetIntrinsics(fx, fy, cx, cy);
            SetDistortion(k1, k2, p1, p2, k3);
            Debug.Log($"[SharedMemoryWriter] Camera model set: fx={fx:F2}, fy={fy:F2}, cx={cx:F2}, cy={cy:F2}");
        }

        /// <summary>
        /// Initializes the shared memory region.
        /// </summary>
        /// <returns>True if initialization was successful.</returns>
        public bool Initialize()
        {
            if (_isInitialized)
            {
                Debug.LogWarning("[SharedMemoryWriter] Already initialized");
                return true;
            }

            try
            {
                CreateMemoryMappedFile();
                _isInitialized = true;

                Debug.Log($"[SharedMemoryWriter] Initialized: {_width}x{_height}, " +
                    $"buffer size: {_singleBufferSize / 1024.0:F1}KB, " +
                    $"total size: {_totalSize / 1024.0:F1}KB, " +
                    $"double-buffering: {_useDoubleBuffering}");

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SharedMemoryWriter] Failed to initialize: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Writes a complete frame to shared memory.
        /// </summary>
        /// <param name="rgbData">RGB image data in BGR format (width * height * 3 bytes).</param>
        /// <param name="depthData">Depth image data as float array (width * height floats).</param>
        /// <param name="poseMatrix">4x4 pose matrix as double array (16 doubles, row-major).</param>
        /// <param name="timestamp">Frame timestamp in seconds.</param>
        /// <returns>True if write was successful.</returns>
        public bool WriteFrame(byte[] rgbData, float[] depthData, double[] poseMatrix, double timestamp)
        {
            if (!_isInitialized)
            {
                Debug.LogError("[SharedMemoryWriter] Not initialized");
                return false;
            }

            if (_isDisposed)
            {
                Debug.LogError("[SharedMemoryWriter] Already disposed");
                return false;
            }

            // Validate input sizes
            if (rgbData == null || rgbData.Length != _rgbSize)
            {
                Debug.LogError($"[SharedMemoryWriter] Invalid RGB data size. Expected {_rgbSize}, got {rgbData?.Length ?? 0}");
                return false;
            }

            if (depthData == null || depthData.Length != _width * _height)
            {
                Debug.LogError($"[SharedMemoryWriter] Invalid depth data size. Expected {_width * _height}, got {depthData?.Length ?? 0}");
                return false;
            }

            if (poseMatrix == null || poseMatrix.Length != 16)
            {
                Debug.LogError($"[SharedMemoryWriter] Invalid pose matrix size. Expected 16, got {poseMatrix?.Length ?? 0}");
                return false;
            }

            _writeStopwatch.Restart();

            try
            {
                // Calculate buffer offset for double-buffering
                int bufferOffset = _useDoubleBuffering ? (_activeBuffer * _singleBufferSize) : 0;

                // Update header
                _frameId++;
                _header.FrameId = _frameId;
                _header.Timestamp = timestamp;
                _header.ActiveBuffer = _activeBuffer;

                // Write header
                WriteHeader(bufferOffset);

                // Write RGB data
                int rgbOffset = bufferOffset + _header.RgbOffset;
                _accessor.WriteArray(rgbOffset, rgbData, 0, rgbData.Length);

                // Write depth data (convert float[] to bytes)
                int depthOffset = bufferOffset + _header.DepthOffset;
                WriteFloatArray(depthOffset, depthData);

                // Write pose matrix (convert double[] to bytes)
                int poseOffset = bufferOffset + _header.PoseOffset;
                WriteDoubleArray(poseOffset, poseMatrix);

                // Swap buffers for next write
                if (_useDoubleBuffering)
                {
                    _activeBuffer = 1 - _activeBuffer;
                }

                _writeStopwatch.Stop();

                // Update metrics
                long writeTimeMs = _writeStopwatch.ElapsedMilliseconds;
                _totalWriteTimeMs += writeTimeMs;
                _writeCount++;
                if (writeTimeMs > _maxWriteTimeMs)
                {
                    _maxWriteTimeMs = writeTimeMs;
                }

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SharedMemoryWriter] Write failed: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Writes a complete frame using Unity texture data directly.
        /// </summary>
        /// <param name="rgbTexture">RGB texture (will be read as BGR).</param>
        /// <param name="depthTexture">Depth texture (R32 float format).</param>
        /// <param name="poseMatrix">4x4 pose matrix.</param>
        /// <param name="timestamp">Frame timestamp.</param>
        /// <returns>True if write was successful.</returns>
        public bool WriteFrame(Texture2D rgbTexture, Texture2D depthTexture, Matrix4x4 poseMatrix, double timestamp)
        {
            if (rgbTexture == null || depthTexture == null)
            {
                Debug.LogError("[SharedMemoryWriter] Null texture provided");
                return false;
            }

            // Convert RGB texture to BGR byte array
            byte[] rgbData = ConvertTextureToBGR(rgbTexture);

            // Get depth data as float array
            float[] depthData = ConvertDepthTexture(depthTexture);

            // Convert Unity Matrix4x4 to double array (row-major)
            double[] poseArray = ConvertMatrix4x4ToDoubleArray(poseMatrix);

            return WriteFrame(rgbData, depthData, poseArray, timestamp);
        }

        /// <summary>
        /// Gets performance metrics as a formatted string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[SharedMemoryWriter] Performance Report:\n" +
                   $"  Frames written: {_writeCount}\n" +
                   $"  Avg write time: {AverageWriteTimeMs:F2}ms\n" +
                   $"  Max write time: {_maxWriteTimeMs}ms\n" +
                   $"  Throughput: {ThroughputMBps:F1} MB/s";
        }

        /// <summary>
        /// Resets performance metrics.
        /// </summary>
        public void ResetMetrics()
        {
            _totalWriteTimeMs = 0;
            _writeCount = 0;
            _maxWriteTimeMs = 0;
        }

        #endregion

        #region Private Methods

        private void CreateMemoryMappedFile()
        {
            if (IsLinux())
            {
                CreateLinuxMemoryMappedFile();
            }
            else
            {
                CreateWindowsMemoryMappedFile();
            }

            _accessor = _memoryMappedFile.CreateViewAccessor(0, _totalSize);

            // Zero out the memory
            byte[] zeros = new byte[Math.Min(4096, _totalSize)];
            for (int offset = 0; offset < _totalSize; offset += zeros.Length)
            {
                int bytesToWrite = Math.Min(zeros.Length, _totalSize - offset);
                _accessor.WriteArray(offset, zeros, 0, bytesToWrite);
            }
        }

        private void CreateLinuxMemoryMappedFile()
        {
            // On Linux, create a file in /dev/shm/ which is a tmpfs mount
            string filePath = Path.Combine(LINUX_SHM_PATH, _memoryName);

            // Create or open the file and set its size
            _linuxFileStream = new FileStream(filePath, FileMode.Create, FileAccess.ReadWrite, FileShare.ReadWrite);
            _linuxFileStream.SetLength(_totalSize);

            // Memory-map the file (leaveOpen: true so we manage the stream ourselves)
            _memoryMappedFile = MemoryMappedFile.CreateFromFile(
                _linuxFileStream,
                null,
                _totalSize,
                MemoryMappedFileAccess.ReadWrite,
                HandleInheritability.None,
                true);  // leaveOpen = true, we'll close the stream in Dispose

            Debug.Log($"[SharedMemoryWriter] Created Linux shared memory at: {filePath}");
        }

        private void CreateWindowsMemoryMappedFile()
        {
            // On Windows, create a named memory-mapped file
            _memoryMappedFile = MemoryMappedFile.CreateOrOpen(
                _memoryName,
                _totalSize,
                MemoryMappedFileAccess.ReadWrite);

            Debug.Log($"[SharedMemoryWriter] Created Windows shared memory: {_memoryName}");
        }

        private void WriteHeader(int offset)
        {
            // Write header fields individually for reliability
            // Must match FrameHeader struct layout exactly
            _accessor.Write(offset + 0, _header.FrameId);
            _accessor.Write(offset + 8, _header.Timestamp);
            _accessor.Write(offset + 16, _header.Width);
            _accessor.Write(offset + 20, _header.Height);
            _accessor.Write(offset + 24, _header.RgbOffset);
            _accessor.Write(offset + 28, _header.DepthOffset);
            _accessor.Write(offset + 32, _header.PoseOffset);
            _accessor.Write(offset + 36, _header.ActiveBuffer);

            // Camera intrinsics
            _accessor.Write(offset + 40, _header.Fx);
            _accessor.Write(offset + 48, _header.Fy);
            _accessor.Write(offset + 56, _header.Cx);
            _accessor.Write(offset + 64, _header.Cy);

            // Distortion coefficients
            _accessor.Write(offset + 72, _header.K1);
            _accessor.Write(offset + 80, _header.K2);
            _accessor.Write(offset + 88, _header.P1);
            _accessor.Write(offset + 96, _header.P2);
            _accessor.Write(offset + 104, _header.K3);

            // Reserved bytes (offset 112-127) are left as zeros
        }

        private void WriteFloatArray(int offset, float[] data)
        {
            // Write float array directly using WriteArray for performance
            _accessor.WriteArray(offset, data, 0, data.Length);
        }

        private void WriteDoubleArray(int offset, double[] data)
        {
            // Write double array directly using WriteArray for performance
            _accessor.WriteArray(offset, data, 0, data.Length);
        }

        private byte[] ConvertTextureToBGR(Texture2D texture)
        {
            Color32[] pixels = texture.GetPixels32();
            byte[] bgrData = new byte[pixels.Length * 3];

            for (int i = 0; i < pixels.Length; i++)
            {
                int idx = i * 3;
                bgrData[idx + 0] = pixels[i].b;  // Blue
                bgrData[idx + 1] = pixels[i].g;  // Green
                bgrData[idx + 2] = pixels[i].r;  // Red
            }

            return bgrData;
        }

        private float[] ConvertDepthTexture(Texture2D depthTexture)
        {
            // Assuming R32_SFloat format or similar
            Color[] pixels = depthTexture.GetPixels();
            float[] depthData = new float[pixels.Length];

            for (int i = 0; i < pixels.Length; i++)
            {
                depthData[i] = pixels[i].r;  // Depth stored in red channel
            }

            return depthData;
        }

        private double[] ConvertMatrix4x4ToDoubleArray(Matrix4x4 matrix)
        {
            // Convert Unity Matrix4x4 to row-major double array
            // Unity matrices are column-major, so we transpose during conversion
            double[] result = new double[16];

            for (int row = 0; row < 4; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    result[row * 4 + col] = matrix[row, col];
                }
            }

            return result;
        }

        private static bool IsLinux()
        {
            return Application.platform == RuntimePlatform.LinuxPlayer ||
                   Application.platform == RuntimePlatform.LinuxEditor ||
                   Application.platform == RuntimePlatform.LinuxServer;
        }

        #endregion

        #region IDisposable Implementation

        /// <summary>
        /// Releases all resources used by the SharedMemoryWriter.
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Releases resources.
        /// </summary>
        /// <param name="disposing">True if called from Dispose(), false if from finalizer.</param>
        protected virtual void Dispose(bool disposing)
        {
            if (_isDisposed)
            {
                return;
            }

            if (disposing)
            {
                // Dispose managed resources in reverse order of creation
                _accessor?.Dispose();
                _memoryMappedFile?.Dispose();
                _linuxFileStream?.Dispose();  // Close the file stream after memory-mapped file

                // On Linux, clean up the shared memory file
                if (IsLinux())
                {
                    try
                    {
                        string filePath = Path.Combine(LINUX_SHM_PATH, _memoryName);
                        if (File.Exists(filePath))
                        {
                            File.Delete(filePath);
                            Debug.Log($"[SharedMemoryWriter] Cleaned up shared memory file: {filePath}");
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"[SharedMemoryWriter] Failed to clean up shared memory file: {ex.Message}");
                    }
                }
            }

            _isDisposed = true;
            _isInitialized = false;

            Debug.Log($"[SharedMemoryWriter] Disposed. {GetPerformanceReport()}");
        }

        /// <summary>
        /// Finalizer.
        /// </summary>
        ~SharedMemoryWriter()
        {
            Dispose(false);
        }

        #endregion
    }
}
