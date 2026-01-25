// Auto-Battlebot Simulation System
// Unit tests for SharedMemoryWriter - verifies write/read consistency

using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AutoBattlebot.Communication;

namespace AutoBattlebot.Tests.Editor
{
    /// <summary>
    /// Unit tests for SharedMemoryWriter.
    /// Tests write/read consistency to verify data integrity.
    /// </summary>
    [TestFixture]
    public class SharedMemoryWriterTests
    {
        private const string TEST_MEMORY_NAME = "auto_battlebot_test_frame";
        private const int TEST_WIDTH = 64;   // Small size for fast tests
        private const int TEST_HEIGHT = 48;

        private SharedMemoryWriter _writer;

        [SetUp]
        public void SetUp()
        {
            // Clean up any leftover test files
            CleanupTestMemory();
        }

        [TearDown]
        public void TearDown()
        {
            _writer?.Dispose();
            _writer = null;
            CleanupTestMemory();
        }

        private void CleanupTestMemory()
        {
            // Clean up Linux shared memory file if it exists
            string linuxPath = Path.Combine("/dev/shm/", TEST_MEMORY_NAME);
            if (File.Exists(linuxPath))
            {
                try { File.Delete(linuxPath); } catch { }
            }
        }

        #region Initialization Tests

        [Test]
        public void Initialize_CreatesSharedMemory()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);

            // Act
            bool result = _writer.Initialize();

            // Assert
            Assert.IsTrue(result, "Initialize should return true");
            Assert.IsTrue(_writer.IsInitialized, "IsInitialized should be true after initialization");
        }

        [Test]
        public void Initialize_CalledTwice_ReturnsTrue()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            // Act
            bool result = _writer.Initialize();

            // Assert
            Assert.IsTrue(result, "Second Initialize call should return true");
        }

        [Test]
        public void Constructor_CalculatesCorrectSizes()
        {
            // Arrange & Act
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: false);

            // Assert - verify through successful initialization
            Assert.IsTrue(_writer.Initialize());

            // Expected sizes:
            // Header: 128 bytes
            // RGB: 64 * 48 * 3 = 9,216 bytes
            // Depth: 64 * 48 * 4 = 12,288 bytes
            // Pose: 128 bytes (16 doubles × 8 bytes)
            // Total single buffer: 128 + 9,216 + 12,288 + 128 = 21,760 bytes
            int expectedSize = FrameHeader.CalculateTotalSize(TEST_WIDTH, TEST_HEIGHT);
            Assert.AreEqual(21760, expectedSize, "Total frame size calculation should match expected");
        }

        #endregion

        #region Write/Read Consistency Tests

        [Test]
        public void WriteFrame_RgbData_CanBeReadBack()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: false);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act
            bool writeResult = _writer.WriteFrame(rgbData, depthData, poseMatrix, 1.5);

            // Assert
            Assert.IsTrue(writeResult, "WriteFrame should return true");

            // Read back and verify
            using (var reader = OpenMemoryForReading())
            {
                // Read header
                var header = ReadHeader(reader);
                Assert.AreEqual(1UL, header.FrameId, "FrameId should be 1");
                Assert.AreEqual(1.5, header.Timestamp, 0.001, "Timestamp should match");
                Assert.AreEqual(TEST_WIDTH, header.Width, "Width should match");
                Assert.AreEqual(TEST_HEIGHT, header.Height, "Height should match");

                // Read RGB data
                byte[] readRgb = new byte[rgbData.Length];
                reader.ReadArray(header.RgbOffset, readRgb, 0, readRgb.Length);
                CollectionAssert.AreEqual(rgbData, readRgb, "RGB data should match what was written");
            }
        }

        [Test]
        public void WriteFrame_DepthData_CanBeReadBack()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: false);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 2.0);

            // Assert - Read back depth data
            using (var reader = OpenMemoryForReading())
            {
                var header = ReadHeader(reader);

                float[] readDepth = new float[depthData.Length];
                reader.ReadArray(header.DepthOffset, readDepth, 0, readDepth.Length);

                for (int i = 0; i < depthData.Length; i++)
                {
                    Assert.AreEqual(depthData[i], readDepth[i], 0.0001f,
                        $"Depth value at index {i} should match");
                }
            }
        }

        [Test]
        public void WriteFrame_PoseMatrix_CanBeReadBack()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: false);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 3.0);

            // Assert - Read back pose matrix
            using (var reader = OpenMemoryForReading())
            {
                var header = ReadHeader(reader);

                double[] readPose = new double[16];
                reader.ReadArray(header.PoseOffset, readPose, 0, readPose.Length);

                for (int i = 0; i < 16; i++)
                {
                    Assert.AreEqual(poseMatrix[i], readPose[i], 0.0000001,
                        $"Pose matrix value at index {i} should match");
                }
            }
        }

        [Test]
        public void WriteFrame_MultipleFrames_FrameIdIncrements()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: false);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act - Write 3 frames
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 1.0);
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 2.0);
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 3.0);

            // Assert
            Assert.AreEqual(3UL, _writer.CurrentFrameId, "CurrentFrameId should be 3 after 3 writes");

            using (var reader = OpenMemoryForReading())
            {
                var header = ReadHeader(reader);
                Assert.AreEqual(3UL, header.FrameId, "FrameId in memory should be 3");
                Assert.AreEqual(3.0, header.Timestamp, 0.001, "Timestamp should be from last frame");
            }
        }

        #endregion

        #region Double Buffering Tests

        [Test]
        public void WriteFrame_WithDoubleBuffering_AlternatesBuffers()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME, useDoubleBuffering: true);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act - Write first frame (goes to buffer 0)
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 1.0);

            int singleBufferSize = FrameHeader.CalculateTotalSize(TEST_WIDTH, TEST_HEIGHT);

            using (var reader = OpenMemoryForReading())
            {
                // Check buffer 0 (where first frame was written)
                var header0 = ReadHeaderAtOffset(reader, 0);
                Assert.AreEqual(1UL, header0.FrameId, "Buffer 0 should have frame 1");
            }

            // Write second frame (goes to buffer 1)
            _writer.WriteFrame(rgbData, depthData, poseMatrix, 2.0);

            using (var reader = OpenMemoryForReading())
            {
                // Check buffer 1
                var header1 = ReadHeaderAtOffset(reader, singleBufferSize);
                Assert.AreEqual(2UL, header1.FrameId, "Buffer 1 should have frame 2");
            }
        }

        #endregion

        #region Validation Tests

        [Test]
        public void WriteFrame_NullRgbData_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Expect the error log
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("Invalid RGB data size"));

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(null, depthData, poseMatrix, 1.0));
        }

        [Test]
        public void WriteFrame_WrongRgbSize_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            byte[] wrongSizeRgb = new byte[100]; // Wrong size
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Expect the error log
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("Invalid RGB data size"));

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(wrongSizeRgb, depthData, poseMatrix, 1.0));
        }

        [Test]
        public void WriteFrame_NullDepthData_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Expect the error log
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("Invalid depth data size"));

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(rgbData, null, poseMatrix, 1.0));
        }

        [Test]
        public void WriteFrame_NullPoseMatrix_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);

            // Expect the error log
            LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("Invalid pose matrix size"));

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(rgbData, depthData, null, 1.0));
        }

        [Test]
        public void WriteFrame_BeforeInitialization_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            // Note: NOT calling Initialize()

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Expect the error log
            LogAssert.Expect(LogType.Error, "[SharedMemoryWriter] Not initialized");

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(rgbData, depthData, poseMatrix, 1.0));
        }

        [Test]
        public void WriteFrame_AfterDispose_ReturnsFalse()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();
            _writer.Dispose();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Expect the error log (disposed sets _isInitialized = false)
            LogAssert.Expect(LogType.Error, "[SharedMemoryWriter] Not initialized");

            // Act & Assert
            Assert.IsFalse(_writer.WriteFrame(rgbData, depthData, poseMatrix, 1.0));

            // Prevent TearDown from double-disposing
            _writer = null;
        }

        #endregion

        #region Performance Metrics Tests

        [Test]
        public void PerformanceMetrics_TracksWriteCount()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            // Act
            for (int i = 0; i < 5; i++)
            {
                _writer.WriteFrame(rgbData, depthData, poseMatrix, i);
            }

            // Assert
            Assert.AreEqual(5, _writer.WriteCount, "WriteCount should be 5");
        }

        [Test]
        public void ResetMetrics_ClearsAllMetrics()
        {
            // Arrange
            _writer = new SharedMemoryWriter(TEST_WIDTH, TEST_HEIGHT, TEST_MEMORY_NAME);
            _writer.Initialize();

            byte[] rgbData = CreateTestRgbData(TEST_WIDTH, TEST_HEIGHT);
            float[] depthData = CreateTestDepthData(TEST_WIDTH, TEST_HEIGHT);
            double[] poseMatrix = CreateTestPoseMatrix();

            _writer.WriteFrame(rgbData, depthData, poseMatrix, 1.0);

            // Act
            _writer.ResetMetrics();

            // Assert
            Assert.AreEqual(0, _writer.WriteCount, "WriteCount should be 0 after reset");
            Assert.AreEqual(0, _writer.MaxWriteTimeMs, "MaxWriteTimeMs should be 0 after reset");
        }

        #endregion

        #region FrameHeader Tests

        [Test]
        public void FrameHeader_Size_Is128Bytes()
        {
            Assert.AreEqual(128, FrameHeader.SIZE, "FrameHeader.SIZE should be 128 bytes");
        }

        [Test]
        public void FrameHeader_PoseSize_Is128Bytes()
        {
            Assert.AreEqual(128, FrameHeader.POSE_SIZE, "FrameHeader.POSE_SIZE should be 128 bytes (16 doubles)");
        }

        [Test]
        public void FrameHeader_Create_SetsDefaultIntrinsics()
        {
            // Arrange & Act
            var header = FrameHeader.Create(1280, 720);

            // Assert - default intrinsics based on ~90° FOV
            Assert.AreEqual(640.0, header.Fx, 0.001, "Default Fx should be width/2");
            Assert.AreEqual(640.0, header.Fy, 0.001, "Default Fy should be width/2");
            Assert.AreEqual(640.0, header.Cx, 0.001, "Default Cx should be width/2");
            Assert.AreEqual(360.0, header.Cy, 0.001, "Default Cy should be height/2");
        }

        [Test]
        public void FrameHeader_Create_WithIntrinsics_SetsCorrectValues()
        {
            // Arrange & Act
            var header = FrameHeader.Create(1280, 720, 500.0, 500.0, 640.0, 360.0);

            // Assert
            Assert.AreEqual(500.0, header.Fx, 0.001);
            Assert.AreEqual(500.0, header.Fy, 0.001);
            Assert.AreEqual(640.0, header.Cx, 0.001);
            Assert.AreEqual(360.0, header.Cy, 0.001);
        }

        [Test]
        public void FrameHeader_CreateFromFov_CalculatesCorrectIntrinsics()
        {
            // Arrange - 60° vertical FOV on 1080p
            int width = 1920;
            int height = 1080;
            float fov = 60.0f;

            // Act
            var header = FrameHeader.CreateFromFov(width, height, fov);

            // Assert
            // fy = height / (2 * tan(fov/2)) = 1080 / (2 * tan(30°)) = 1080 / 1.1547 ≈ 935.3
            double expectedFy = height / (2.0 * System.Math.Tan(fov * System.Math.PI / 360.0));
            Assert.AreEqual(expectedFy, header.Fy, 0.1, "Fy should be calculated from FOV");
            Assert.AreEqual(expectedFy, header.Fx, 0.1, "Fx should equal Fy (square pixels)");
            Assert.AreEqual(width / 2.0, header.Cx, 0.001, "Cx should be image center");
            Assert.AreEqual(height / 2.0, header.Cy, 0.001, "Cy should be image center");
        }

        [Test]
        public void FrameHeader_Create_SetsZeroDistortionByDefault()
        {
            // Arrange & Act
            var header = FrameHeader.Create(1280, 720);

            // Assert - default should have no distortion
            Assert.AreEqual(0.0, header.K1, 0.0001, "K1 should be 0 by default");
            Assert.AreEqual(0.0, header.K2, 0.0001, "K2 should be 0 by default");
            Assert.AreEqual(0.0, header.P1, 0.0001, "P1 should be 0 by default");
            Assert.AreEqual(0.0, header.P2, 0.0001, "P2 should be 0 by default");
            Assert.AreEqual(0.0, header.K3, 0.0001, "K3 should be 0 by default");
        }

        [Test]
        public void FrameHeader_Create_WithDistortion_SetsCorrectValues()
        {
            // Arrange - typical ZED camera distortion coefficients
            double k1 = -0.0412;
            double k2 = 0.0135;
            double p1 = 0.0001;
            double p2 = -0.0002;
            double k3 = -0.0053;

            // Act
            var header = FrameHeader.Create(1280, 720, 700.0, 700.0, 640.0, 360.0, k1, k2, p1, p2, k3);

            // Assert
            Assert.AreEqual(k1, header.K1, 0.0001);
            Assert.AreEqual(k2, header.K2, 0.0001);
            Assert.AreEqual(p1, header.P1, 0.0001);
            Assert.AreEqual(p2, header.P2, 0.0001);
            Assert.AreEqual(k3, header.K3, 0.0001);
        }

        [Test]
        public void FrameHeader_Create_CalculatesCorrectOffsets()
        {
            // Arrange & Act
            var header = FrameHeader.Create(1280, 720);

            // Assert
            Assert.AreEqual(1280, header.Width);
            Assert.AreEqual(720, header.Height);
            Assert.AreEqual(128, header.RgbOffset, "RGB should start after header");

            int expectedRgbSize = 1280 * 720 * 3;
            Assert.AreEqual(128 + expectedRgbSize, header.DepthOffset, "Depth should start after RGB");

            int expectedDepthSize = 1280 * 720 * 4;
            Assert.AreEqual(128 + expectedRgbSize + expectedDepthSize, header.PoseOffset, "Pose should start after depth");
        }

        [Test]
        public void FrameHeader_CalculateTotalSize_ReturnsCorrectValue()
        {
            // Arrange
            int width = 1280;
            int height = 720;

            // Act
            int totalSize = FrameHeader.CalculateTotalSize(width, height);

            // Assert
            // Header: 128
            // RGB: 1280 * 720 * 3 = 2,764,800
            // Depth: 1280 * 720 * 4 = 3,686,400
            // Pose: 128 (16 doubles × 8 bytes)
            // Total: 6,451,456
            int expected = 128 + (1280 * 720 * 3) + (1280 * 720 * 4) + 128;
            Assert.AreEqual(expected, totalSize);
        }

        #endregion

        #region Helper Methods

        private byte[] CreateTestRgbData(int width, int height)
        {
            byte[] data = new byte[width * height * 3];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = (byte)(i % 256);
            }
            return data;
        }

        private float[] CreateTestDepthData(int width, int height)
        {
            float[] data = new float[width * height];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = (i % 1000) / 100.0f; // Values from 0.0 to 9.99
            }
            return data;
        }

        private double[] CreateTestPoseMatrix()
        {
            // Create an identity matrix with some modifications for testing
            return new double[]
            {
                1.0, 0.0, 0.0, 1.5,   // Row 0: identity with X translation
                0.0, 1.0, 0.0, 2.5,   // Row 1: identity with Y translation
                0.0, 0.0, 1.0, 3.5,   // Row 2: identity with Z translation
                0.0, 0.0, 0.0, 1.0    // Row 3: homogeneous coordinate
            };
        }

        private MemoryMappedViewAccessor OpenMemoryForReading()
        {
            string filePath = Path.Combine("/dev/shm/", TEST_MEMORY_NAME);

            if (File.Exists(filePath))
            {
                // Linux path
                var fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
                var mmf = MemoryMappedFile.CreateFromFile(
                    fileStream,
                    null,
                    0,
                    MemoryMappedFileAccess.Read,
                    HandleInheritability.None,
                    false);
                return mmf.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read);
            }
            else
            {
                // Windows path
                var mmf = MemoryMappedFile.OpenExisting(TEST_MEMORY_NAME, MemoryMappedFileRights.Read);
                return mmf.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read);
            }
        }

        private FrameHeader ReadHeader(MemoryMappedViewAccessor accessor)
        {
            return ReadHeaderAtOffset(accessor, 0);
        }

        private FrameHeader ReadHeaderAtOffset(MemoryMappedViewAccessor accessor, int offset)
        {
            var header = new FrameHeader
            {
                FrameId = accessor.ReadUInt64(offset + 0),
                Timestamp = accessor.ReadDouble(offset + 8),
                Width = accessor.ReadInt32(offset + 16),
                Height = accessor.ReadInt32(offset + 20),
                RgbOffset = accessor.ReadInt32(offset + 24),
                DepthOffset = accessor.ReadInt32(offset + 28),
                PoseOffset = accessor.ReadInt32(offset + 32),
                ActiveBuffer = accessor.ReadInt32(offset + 36)
            };
            return header;
        }

        #endregion
    }
}
