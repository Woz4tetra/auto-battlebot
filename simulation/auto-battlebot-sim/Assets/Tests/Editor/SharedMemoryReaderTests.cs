// Auto-Battlebot Simulation System
// Unit tests for SharedMemoryReader - verifies command read functionality

using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AutoBattlebot.Communication;

namespace AutoBattlebot.Tests.Editor
{
    /// <summary>
    /// Unit tests for SharedMemoryReader.
    /// Tests command reading, validation, and new command detection.
    /// </summary>
    [TestFixture]
    public class SharedMemoryReaderTests
    {
        private const string TEST_MEMORY_NAME = "auto_battlebot_test_command";

        private SharedMemoryReader _reader;

        [SetUp]
        public void SetUp()
        {
            CleanupTestMemory();
        }

        [TearDown]
        public void TearDown()
        {
            _reader?.Dispose();
            _reader = null;
            CleanupTestMemory();
        }

        private void CleanupTestMemory()
        {
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
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);

            // Act
            bool result = _reader.Initialize(createIfNotExists: true);

            // Assert
            Assert.IsTrue(result, "Initialize should return true");
            Assert.IsTrue(_reader.IsInitialized, "IsInitialized should be true");
        }

        [Test]
        public void Initialize_CalledTwice_ReturnsTrue()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Act
            bool result = _reader.Initialize();

            // Assert
            Assert.IsTrue(result, "Second Initialize should return true");
        }

        #endregion

        #region Read Tests

        [Test]
        public void TryReadCommand_ReadsZeroCommandInitially()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Act
            bool result = _reader.TryReadCommand(out var command, out bool isNew);

            // Assert
            Assert.IsTrue(result, "TryReadCommand should succeed");
            Assert.AreEqual(0UL, command.CommandId, "Initial command ID should be 0");
            Assert.AreEqual(0.0, command.LinearX, 0.0001, "Initial LinearX should be 0");
            Assert.AreEqual(0.0, command.LinearY, 0.0001, "Initial LinearY should be 0");
            Assert.AreEqual(0.0, command.AngularZ, 0.0001, "Initial AngularZ should be 0");
        }

        [Test]
        public void TryReadCommand_DetectsNewCommand()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Read initial state
            _reader.TryReadCommand(out _, out _);

            // Write a new command
            var newCommand = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 1.5,
                LinearY = 0.5,
                AngularZ = 0.3
            };
            SharedMemoryTestUtils.WriteCommand(_reader, newCommand);

            // Act
            bool result = _reader.TryReadCommand(out var readCommand, out bool isNew);

            // Assert
            Assert.IsTrue(result, "TryReadCommand should succeed");
            Assert.IsTrue(isNew, "Should detect new command");
            Assert.AreEqual(1UL, readCommand.CommandId, "CommandId should match");
            Assert.AreEqual(1.5, readCommand.LinearX, 0.0001, "LinearX should match");
            Assert.AreEqual(0.5, readCommand.LinearY, 0.0001, "LinearY should match");
            Assert.AreEqual(0.3, readCommand.AngularZ, 0.0001, "AngularZ should match");
        }

        [Test]
        public void TryReadCommand_SameCommandIdNotNew()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            var command = new VelocityCommand
            {
                CommandId = 5,
                LinearX = 1.0,
                LinearY = 0.0,
                AngularZ = 0.5
            };
            SharedMemoryTestUtils.WriteCommand(_reader, command);

            // First read
            _reader.TryReadCommand(out _, out bool firstIsNew);
            Assert.IsTrue(firstIsNew, "First read should be new");

            // Act - Second read with same command_id
            bool result = _reader.TryReadCommand(out var readCommand, out bool secondIsNew);

            // Assert
            Assert.IsTrue(result, "TryReadCommand should succeed");
            Assert.IsFalse(secondIsNew, "Second read should not be new");
            Assert.AreEqual(5UL, readCommand.CommandId, "CommandId should still be readable");
        }

        [Test]
        public void TryReadCommand_TracksLastCommand()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            var command = new VelocityCommand
            {
                CommandId = 10,
                LinearX = 2.0,
                LinearY = -1.0,
                AngularZ = 1.5
            };
            SharedMemoryTestUtils.WriteCommand(_reader, command);

            // Act
            _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.AreEqual(10UL, _reader.LastCommandId, "LastCommandId should be updated");
            Assert.AreEqual(2.0, _reader.LastCommand.LinearX, 0.0001, "LastCommand should be updated");
        }

        #endregion

        #region Validation Tests

        [Test]
        public void TryReadCommand_RejectsExcessiveLinearVelocity()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME, maxLinearVelocity: 5.0);
            _reader.Initialize();

            // Read initial to set baseline
            _reader.TryReadCommand(out _, out _);

            var invalidCommand = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 100.0,  // Exceeds max
                LinearY = 0.0,
                AngularZ = 0.0
            };
            SharedMemoryTestUtils.WriteCommand(_reader, invalidCommand);

            // Expect warning log
            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("Invalid command"));

            // Act
            bool result = _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.IsFalse(result, "Should reject invalid command");
            Assert.AreEqual(1, _reader.InvalidCommandCount, "InvalidCommandCount should increment");
        }

        [Test]
        public void TryReadCommand_RejectsExcessiveAngularVelocity()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME, maxAngularVelocity: 10.0);
            _reader.Initialize();

            // Read initial
            _reader.TryReadCommand(out _, out _);

            var invalidCommand = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 0.0,
                LinearY = 0.0,
                AngularZ = 50.0  // Exceeds max
            };
            SharedMemoryTestUtils.WriteCommand(_reader, invalidCommand);

            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("Invalid command"));

            // Act
            bool result = _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.IsFalse(result, "Should reject invalid command");
        }

        [Test]
        public void TryReadCommand_RejectsNaNValues()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Read initial
            _reader.TryReadCommand(out _, out _);

            var invalidCommand = new VelocityCommand
            {
                CommandId = 1,
                LinearX = double.NaN,
                LinearY = 0.0,
                AngularZ = 0.0
            };
            SharedMemoryTestUtils.WriteCommand(_reader, invalidCommand);

            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("Invalid command"));

            // Act
            bool result = _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.IsFalse(result, "Should reject NaN values");
        }

        [Test]
        public void TryReadCommand_BeforeInitialization_ReturnsFalse()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            // NOT calling Initialize()

            LogAssert.Expect(LogType.Error, "[SharedMemoryReader] Not initialized");

            // Act
            bool result = _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.IsFalse(result, "Should fail before initialization");
        }

        [Test]
        public void TryReadCommand_AfterDispose_ReturnsFalse()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();
            _reader.Dispose();

            LogAssert.Expect(LogType.Error, "[SharedMemoryReader] Not initialized");

            // Act
            bool result = _reader.TryReadCommand(out _, out _);

            // Assert
            Assert.IsFalse(result, "Should fail after dispose");

            _reader = null;  // Prevent double dispose
        }

        #endregion

        #region Performance Metrics Tests

        [Test]
        public void PerformanceMetrics_TracksReadCount()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Act
            for (int i = 0; i < 10; i++)
            {
                _reader.TryReadCommand(out _, out _);
            }

            // Assert
            Assert.AreEqual(10, _reader.ReadCount, "ReadCount should be 10");
        }

        [Test]
        public void PerformanceMetrics_TracksNewCommandCount()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Initial read
            _reader.TryReadCommand(out _, out _);

            // Write 3 new commands
            for (ulong i = 1; i <= 3; i++)
            {
                SharedMemoryTestUtils.WriteCommand(_reader, new VelocityCommand { CommandId = i });
                _reader.TryReadCommand(out _, out _);
            }

            // Assert - initial read (id=0) is NOT new since _lastCommandId starts at 0
            // Only the 3 written commands (id=1,2,3) are detected as new
            Assert.AreEqual(3, _reader.NewCommandCount, "NewCommandCount should be 3");
        }

        [Test]
        public void ResetMetrics_ClearsAllMetrics()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();
            _reader.TryReadCommand(out _, out _);

            // Act
            _reader.ResetMetrics();

            // Assert
            Assert.AreEqual(0, _reader.ReadCount, "ReadCount should be 0");
            Assert.AreEqual(0, _reader.NewCommandCount, "NewCommandCount should be 0");
            Assert.AreEqual(0, _reader.InvalidCommandCount, "InvalidCommandCount should be 0");
        }

        [Test]
        public void ReadTime_IsUnder1ms()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Act - Perform multiple reads
            for (int i = 0; i < 100; i++)
            {
                _reader.TryReadCommand(out _, out _);
            }

            // Assert - Average read time should be well under 1ms (1000µs)
            Assert.Less(_reader.AverageReadTimeUs, 1000.0,
                $"Average read time ({_reader.AverageReadTimeUs}µs) should be under 1ms");
        }

        #endregion

        #region VelocityCommand Tests

        [Test]
        public void VelocityCommand_Size_Is32Bytes()
        {
            Assert.AreEqual(32, VelocityCommand.SIZE, "VelocityCommand.SIZE should be 32 bytes");
        }

        [Test]
        public void VelocityCommand_Zero_HasAllZeroValues()
        {
            var zero = VelocityCommand.Zero;

            Assert.AreEqual(0UL, zero.CommandId);
            Assert.AreEqual(0.0, zero.LinearX, 0.0001);
            Assert.AreEqual(0.0, zero.LinearY, 0.0001);
            Assert.AreEqual(0.0, zero.AngularZ, 0.0001);
        }

        [Test]
        public void VelocityCommand_IsValid_ReturnsTrueForValidCommand()
        {
            var command = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 2.0,
                LinearY = -1.5,
                AngularZ = 3.0
            };

            Assert.IsTrue(command.IsValid(), "Valid command should pass validation");
        }

        [Test]
        public void VelocityCommand_IsValid_ReturnsFalseForNaN()
        {
            var command = new VelocityCommand
            {
                CommandId = 1,
                LinearX = double.NaN,
                LinearY = 0.0,
                AngularZ = 0.0
            };

            Assert.IsFalse(command.IsValid(), "NaN should fail validation");
        }

        [Test]
        public void VelocityCommand_IsValid_ReturnsFalseForInfinity()
        {
            var command = new VelocityCommand
            {
                CommandId = 1,
                LinearX = double.PositiveInfinity,
                LinearY = 0.0,
                AngularZ = 0.0
            };

            Assert.IsFalse(command.IsValid(), "Infinity should fail validation");
        }

        [Test]
        public void VelocityCommand_IsValid_ReturnsFalseForExcessiveVelocity()
        {
            var command = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 15.0,  // Exceeds default max of 10
                LinearY = 0.0,
                AngularZ = 0.0
            };

            Assert.IsFalse(command.IsValid(maxLinearVelocity: 10.0),
                "Excessive velocity should fail validation");
        }

        [Test]
        public void VelocityCommand_ToString_FormatsCorrectly()
        {
            var command = new VelocityCommand
            {
                CommandId = 42,
                LinearX = 1.5,
                LinearY = -0.5,
                AngularZ = 2.0
            };

            string str = command.ToString();

            Assert.IsTrue(str.Contains("42"), "Should contain command ID");
            Assert.IsTrue(str.Contains("1.5") || str.Contains("1,5"), "Should contain LinearX");
        }

        #endregion

        #region WaitForNewCommand Tests

        [Test]
        public void WaitForNewCommand_ReturnsImmediatelyWhenNewCommandAvailable()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Read initial
            _reader.TryReadCommand(out _, out _);

            // Write new command
            var command = new VelocityCommand
            {
                CommandId = 1,
                LinearX = 1.0,
                LinearY = 0.0,
                AngularZ = 0.0
            };
            SharedMemoryTestUtils.WriteCommand(_reader, command);

            // Act
            var sw = System.Diagnostics.Stopwatch.StartNew();
            bool result = _reader.WaitForNewCommand(out var readCommand, timeoutMs: 1000);
            sw.Stop();

            // Assert
            Assert.IsTrue(result, "Should find new command");
            Assert.AreEqual(1UL, readCommand.CommandId);
            Assert.Less(sw.ElapsedMilliseconds, 100, "Should return quickly when command is available");
        }

        [Test]
        public void WaitForNewCommand_TimesOutWhenNoNewCommand()
        {
            // Arrange
            _reader = new SharedMemoryReader(TEST_MEMORY_NAME);
            _reader.Initialize();

            // Act
            var sw = System.Diagnostics.Stopwatch.StartNew();
            bool result = _reader.WaitForNewCommand(out _, timeoutMs: 50, pollIntervalMs: 10);
            sw.Stop();

            // Assert
            Assert.IsFalse(result, "Should timeout");
            Assert.GreaterOrEqual(sw.ElapsedMilliseconds, 40, "Should wait close to timeout");
        }

        #endregion
    }
}
