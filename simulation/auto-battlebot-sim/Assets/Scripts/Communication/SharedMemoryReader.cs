// Auto-Battlebot Simulation System
// Shared memory reader for receiving velocity commands from C++ application

using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Shared memory reader that reads velocity commands from the C++ application.
    /// Commands are small and fixed-size (32 bytes), making reads very fast.
    /// 
    /// Features:
    /// - Detects new commands via command_id comparison
    /// - Configurable polling with timeout
    /// - Command validation (velocity range checks)
    /// - Platform abstraction for Linux and Windows
    /// - IDisposable pattern for proper cleanup
    /// </summary>
    public class SharedMemoryReader : IDisposable
    {
        #region Constants

        /// <summary>
        /// Default shared memory name for commands.
        /// </summary>
        public const string DEFAULT_MEMORY_NAME = "auto_battlebot_command";

        /// <summary>
        /// Linux shared memory directory.
        /// </summary>
        private const string LINUX_SHM_PATH = "/dev/shm/";

        #endregion

        #region Private Fields

        private readonly string _memoryName;
        private readonly double _maxLinearVelocity;
        private readonly double _maxAngularVelocity;

        private MemoryMappedFile _memoryMappedFile;
        private MemoryMappedViewAccessor _accessor;
        private FileStream _linuxFileStream;  // Keep reference to prevent GC on Linux

        private ulong _lastCommandId = 0;
        private VelocityCommand _lastCommand = VelocityCommand.Zero;

        private bool _isDisposed = false;
        private bool _isInitialized = false;

        // Performance metrics
        private readonly Stopwatch _readStopwatch = new Stopwatch();
        private long _totalReadTimeUs = 0;  // Microseconds for precision
        private long _readCount = 0;
        private long _maxReadTimeUs = 0;
        private long _newCommandCount = 0;
        private long _invalidCommandCount = 0;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the shared memory has been successfully initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Last command ID that was successfully read.
        /// </summary>
        public ulong LastCommandId => _lastCommandId;

        /// <summary>
        /// Last valid command that was read.
        /// </summary>
        public VelocityCommand LastCommand => _lastCommand;

        /// <summary>
        /// Total number of read operations performed.
        /// </summary>
        public long ReadCount => _readCount;

        /// <summary>
        /// Number of new commands received (command_id changed).
        /// </summary>
        public long NewCommandCount => _newCommandCount;

        /// <summary>
        /// Number of invalid commands received (failed validation).
        /// </summary>
        public long InvalidCommandCount => _invalidCommandCount;

        /// <summary>
        /// Average read time in microseconds.
        /// </summary>
        public double AverageReadTimeUs => _readCount > 0 ? (double)_totalReadTimeUs / _readCount : 0;

        /// <summary>
        /// Maximum read time observed in microseconds.
        /// </summary>
        public long MaxReadTimeUs => _maxReadTimeUs;

        #endregion

        #region Constructor

        /// <summary>
        /// Creates a new SharedMemoryReader.
        /// </summary>
        /// <param name="memoryName">Name for the shared memory region.</param>
        /// <param name="maxLinearVelocity">Maximum allowed linear velocity for validation.</param>
        /// <param name="maxAngularVelocity">Maximum allowed angular velocity for validation.</param>
        public SharedMemoryReader(
            string memoryName = null,
            double maxLinearVelocity = VelocityCommand.DEFAULT_MAX_LINEAR_VELOCITY,
            double maxAngularVelocity = VelocityCommand.DEFAULT_MAX_ANGULAR_VELOCITY)
        {
            _memoryName = memoryName ?? DEFAULT_MEMORY_NAME;
            _maxLinearVelocity = maxLinearVelocity;
            _maxAngularVelocity = maxAngularVelocity;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Initializes the shared memory region for reading.
        /// Creates the memory region if it doesn't exist (for testing without C++).
        /// </summary>
        /// <param name="createIfNotExists">Create the shared memory if it doesn't exist.</param>
        /// <returns>True if initialization was successful.</returns>
        public bool Initialize(bool createIfNotExists = true)
        {
            if (_isInitialized)
            {
                Debug.LogWarning("[SharedMemoryReader] Already initialized");
                return true;
            }

            try
            {
                if (IsLinux())
                {
                    InitializeLinux(createIfNotExists);
                }
                else
                {
                    InitializeWindows(createIfNotExists);
                }

                _isInitialized = true;

                Debug.Log($"[SharedMemoryReader] Initialized: memory={_memoryName}, " +
                    $"maxLinear={_maxLinearVelocity}m/s, maxAngular={_maxAngularVelocity}rad/s");

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SharedMemoryReader] Failed to initialize: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Reads the current command from shared memory.
        /// </summary>
        /// <param name="command">The command that was read.</param>
        /// <param name="isNewCommand">True if the command_id changed since last read.</param>
        /// <returns>True if read was successful and command is valid.</returns>
        public bool TryReadCommand(out VelocityCommand command, out bool isNewCommand)
        {
            command = VelocityCommand.Zero;
            isNewCommand = false;

            if (!_isInitialized)
            {
                Debug.LogError("[SharedMemoryReader] Not initialized");
                return false;
            }

            if (_isDisposed)
            {
                Debug.LogError("[SharedMemoryReader] Already disposed");
                return false;
            }

            _readStopwatch.Restart();

            try
            {
                // Read command from shared memory
                command = ReadCommandFromMemory();

                _readStopwatch.Stop();

                // Update metrics
                long readTimeUs = _readStopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;
                _totalReadTimeUs += readTimeUs;
                _readCount++;
                if (readTimeUs > _maxReadTimeUs)
                {
                    _maxReadTimeUs = readTimeUs;
                }

                // Check if this is a new command
                isNewCommand = command.CommandId != _lastCommandId;

                if (isNewCommand)
                {
                    _newCommandCount++;

                    // Validate the command
                    if (!command.IsValid(_maxLinearVelocity, _maxAngularVelocity))
                    {
                        _invalidCommandCount++;
                        Debug.LogWarning($"[SharedMemoryReader] Invalid command received: {command}");
                        return false;
                    }

                    // Update last known command
                    _lastCommandId = command.CommandId;
                    _lastCommand = command;
                }

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SharedMemoryReader] Read failed: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Polls for a new command with timeout.
        /// </summary>
        /// <param name="command">The new command if one was received.</param>
        /// <param name="timeoutMs">Maximum time to wait in milliseconds (0 = no wait).</param>
        /// <param name="pollIntervalMs">Interval between polls in milliseconds.</param>
        /// <returns>True if a new command was received within the timeout.</returns>
        public bool WaitForNewCommand(out VelocityCommand command, int timeoutMs = 0, int pollIntervalMs = 1)
        {
            command = VelocityCommand.Zero;

            var timeout = Stopwatch.StartNew();

            do
            {
                if (TryReadCommand(out var readCommand, out bool isNew))
                {
                    if (isNew)
                    {
                        command = readCommand;
                        return true;
                    }
                }

                if (timeoutMs > 0 && pollIntervalMs > 0)
                {
                    System.Threading.Thread.Sleep(pollIntervalMs);
                }
            }
            while (timeoutMs > 0 && timeout.ElapsedMilliseconds < timeoutMs);

            return false;
        }

        /// <summary>
        /// Gets performance metrics as a formatted string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[SharedMemoryReader] Performance Report:\n" +
                   $"  Read operations: {_readCount}\n" +
                   $"  New commands: {_newCommandCount}\n" +
                   $"  Invalid commands: {_invalidCommandCount}\n" +
                   $"  Avg read time: {AverageReadTimeUs:F1}µs\n" +
                   $"  Max read time: {_maxReadTimeUs}µs";
        }

        /// <summary>
        /// Resets performance metrics.
        /// </summary>
        public void ResetMetrics()
        {
            _totalReadTimeUs = 0;
            _readCount = 0;
            _maxReadTimeUs = 0;
            _newCommandCount = 0;
            _invalidCommandCount = 0;
        }

        #endregion

        #region Private Methods

        private void InitializeLinux(bool createIfNotExists)
        {
            string filePath = Path.Combine(LINUX_SHM_PATH, _memoryName);

            if (createIfNotExists || File.Exists(filePath))
            {
                // Create or open the file
                _linuxFileStream = new FileStream(
                    filePath,
                    createIfNotExists ? FileMode.OpenOrCreate : FileMode.Open,
                    FileAccess.ReadWrite,
                    FileShare.ReadWrite);

                if (createIfNotExists && _linuxFileStream.Length < VelocityCommand.SIZE)
                {
                    _linuxFileStream.SetLength(VelocityCommand.SIZE);
                }

                // Use the actual file size (may be larger if C++ uses double buffering)
                // but ensure it's at least VelocityCommand.SIZE
                long mapSize = Math.Max(_linuxFileStream.Length, VelocityCommand.SIZE);

                _memoryMappedFile = MemoryMappedFile.CreateFromFile(
                    _linuxFileStream,
                    null,
                    mapSize,
                    MemoryMappedFileAccess.ReadWrite,
                    HandleInheritability.None,
                    true);  // leaveOpen = true

                // Only access the first command-sized region (we read from offset 0)
                _accessor = _memoryMappedFile.CreateViewAccessor(0, VelocityCommand.SIZE);

                Debug.Log($"[SharedMemoryReader] Opened Linux shared memory at: {filePath} (file size: {_linuxFileStream.Length} bytes)");
            }
            else
            {
                throw new FileNotFoundException($"Shared memory file not found: {filePath}");
            }
        }

        private void InitializeWindows(bool createIfNotExists)
        {
            try
            {
                if (createIfNotExists)
                {
                    _memoryMappedFile = MemoryMappedFile.CreateOrOpen(
                        _memoryName,
                        VelocityCommand.SIZE,
                        MemoryMappedFileAccess.ReadWrite);
                }
                else
                {
                    _memoryMappedFile = MemoryMappedFile.OpenExisting(
                        _memoryName,
                        MemoryMappedFileRights.Read);
                }

                _accessor = _memoryMappedFile.CreateViewAccessor(
                    0,
                    VelocityCommand.SIZE,
                    createIfNotExists ? MemoryMappedFileAccess.ReadWrite : MemoryMappedFileAccess.Read);

                Debug.Log($"[SharedMemoryReader] Opened Windows shared memory: {_memoryName}");
            }
            catch (FileNotFoundException)
            {
                throw new FileNotFoundException($"Shared memory not found: {_memoryName}");
            }
        }

        private VelocityCommand ReadCommandFromMemory()
        {
            var command = new VelocityCommand
            {
                CommandId = _accessor.ReadUInt64(0),
                LinearX = _accessor.ReadDouble(8),
                LinearY = _accessor.ReadDouble(16),
                AngularZ = _accessor.ReadDouble(24)
            };

            return command;
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
        /// Releases all resources used by the SharedMemoryReader.
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
                // Dispose managed resources
                _accessor?.Dispose();
                _memoryMappedFile?.Dispose();
                _linuxFileStream?.Dispose();

                // On Linux, clean up the shared memory file if we created it
                if (IsLinux())
                {
                    try
                    {
                        string filePath = Path.Combine(LINUX_SHM_PATH, _memoryName);
                        if (File.Exists(filePath))
                        {
                            File.Delete(filePath);
                            Debug.Log($"[SharedMemoryReader] Cleaned up shared memory file: {filePath}");
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"[SharedMemoryReader] Failed to clean up shared memory file: {ex.Message}");
                    }
                }
            }

            _isDisposed = true;
            _isInitialized = false;

            Debug.Log($"[SharedMemoryReader] Disposed. {GetPerformanceReport()}");
        }

        /// <summary>
        /// Finalizer.
        /// </summary>
        ~SharedMemoryReader()
        {
            Dispose(false);
        }

        #endregion
    }
}
