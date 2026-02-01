// Auto-Battlebot Simulation System
// TCP bridge for small data exchange with C++ application
//
// Handles:
// - Frame synchronization signals
// - Pose metadata transmission
// - Velocity command reception
// - Camera intrinsics transmission (on connection)
//
// Unity acts as TCP server, C++ application connects as client.
// This replaces shared memory for small data while CUDA Interop handles GPU textures.

using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;
using AutoBattlebot.Core;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// TCP bridge for bidirectional small data exchange with the C++ application.
    /// 
    /// Features:
    /// - Server mode: Unity listens for C++ client connections
    /// - Sends camera intrinsics on connection
    /// - Sends frame-ready signals with pose data
    /// - Receives velocity commands
    /// - Automatic reconnection support
    /// - Performance metrics tracking
    /// </summary>
    public class TcpBridge : IDisposable
    {
        #region Constants

        /// <summary>
        /// Default TCP port for the bridge.
        /// </summary>
        public const int DEFAULT_PORT = 18707;

        /// <summary>
        /// Default timeout for blocking operations in milliseconds.
        /// </summary>
        public const int DEFAULT_TIMEOUT_MS = 100;

        /// <summary>
        /// Maximum time to wait for client connection during initialization.
        /// </summary>
        private const int ACCEPT_POLL_INTERVAL_MS = 100;

        #endregion

        #region Private Fields

        private readonly int _port;
        private readonly int _timeoutMs;

        private Socket _serverSocket;
        private Socket _clientSocket;
        private NetworkStream _stream;

        private Thread _acceptThread;
        private volatile bool _stopAcceptThread = false;

        private bool _isDisposed = false;
        private bool _isListening = false;
        private bool _isConnected = false;
        private bool _intrinsicsSent = false;

        private CameraIntrinsics _cachedIntrinsics;

        // Buffers for reading/writing
        private readonly byte[] _readBuffer = new byte[256];
        private readonly byte[] _writeBuffer = new byte[256];
        private readonly object _writeLock = new object();
        private readonly object _readLock = new object();

        // Performance metrics
        private readonly Stopwatch _stopwatch = new Stopwatch();
        private long _totalSendTimeUs = 0;
        private long _totalReceiveTimeUs = 0;
        private long _sendCount = 0;
        private long _receiveCount = 0;
        private long _errorCount = 0;
        private long _reconnectCount = 0;

        // Pending command (received asynchronously)
        private VelocityCommand _pendingCommand = VelocityCommand.Zero;
        private bool _hasNewCommand = false;
        private ulong _lastCommandId = 0;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the server is listening for connections.
        /// </summary>
        public bool IsListening => _isListening;

        /// <summary>
        /// Whether a client is connected.
        /// </summary>
        public bool IsConnected => _isConnected && _clientSocket?.Connected == true;

        /// <summary>
        /// Whether camera intrinsics have been sent to the client.
        /// </summary>
        public bool IntrinsicsSent => _intrinsicsSent;

        /// <summary>
        /// The TCP port being used.
        /// </summary>
        public int Port => _port;

        /// <summary>
        /// Number of messages sent.
        /// </summary>
        public long SendCount => _sendCount;

        /// <summary>
        /// Number of messages received.
        /// </summary>
        public long ReceiveCount => _receiveCount;

        /// <summary>
        /// Number of errors encountered.
        /// </summary>
        public long ErrorCount => _errorCount;

        /// <summary>
        /// Average send latency in microseconds.
        /// </summary>
        public double AverageSendTimeUs => _sendCount > 0 ? (double)_totalSendTimeUs / _sendCount : 0;

        /// <summary>
        /// Average receive latency in microseconds.
        /// </summary>
        public double AverageReceiveTimeUs => _receiveCount > 0 ? (double)_totalReceiveTimeUs / _receiveCount : 0;

        #endregion

        #region Events

        /// <summary>
        /// Fired when a client connects.
        /// </summary>
        public event Action OnClientConnected;

        /// <summary>
        /// Fired when a client disconnects.
        /// </summary>
        public event Action OnClientDisconnected;

        /// <summary>
        /// Fired when a velocity command is received.
        /// </summary>
        public event Action<VelocityCommand> OnCommandReceived;

        /// <summary>
        /// Fired when a frame request is received (parameter: withDepth).
        /// </summary>
        public event Action<bool> OnFrameRequested;

        #endregion

        #region Constructor

        /// <summary>
        /// Creates a new TcpBridge.
        /// </summary>
        /// <param name="port">TCP port to listen on.</param>
        /// <param name="timeoutMs">Timeout for blocking operations.</param>
        public TcpBridge(int port = DEFAULT_PORT, int timeoutMs = DEFAULT_TIMEOUT_MS)
        {
            _port = port;
            _timeoutMs = timeoutMs;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Initializes the TCP server and starts listening for connections.
        /// </summary>
        /// <returns>True if initialization was successful.</returns>
        public bool Initialize()
        {
            if (_isDisposed)
            {
                Debug.LogError("[TcpBridge] Cannot initialize - already disposed");
                return false;
            }

            if (_isListening)
            {
                Debug.LogWarning("[TcpBridge] Already listening");
                return true;
            }

            try
            {
                _serverSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                _serverSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                _serverSocket.Bind(new IPEndPoint(IPAddress.Loopback, _port));
                _serverSocket.Listen(1);

                _isListening = true;

                // Start accept thread
                _stopAcceptThread = false;
                _acceptThread = new Thread(AcceptLoop)
                {
                    IsBackground = true,
                    Name = "TcpBridge-Accept"
                };
                _acceptThread.Start();

                Debug.Log($"[TcpBridge] Server listening on port {_port}");
                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[TcpBridge] Failed to initialize: {ex.Message}");
                _errorCount++;
                return false;
            }
        }

        /// <summary>
        /// Sends camera intrinsics to the connected client.
        /// Should be called once after connection is established.
        /// </summary>
        /// <param name="intrinsics">Camera intrinsics to send.</param>
        /// <returns>True if sent successfully.</returns>
        public bool SendIntrinsics(CameraIntrinsics intrinsics)
        {
            if (!IsConnected)
            {
                Debug.LogWarning("[TcpBridge] Cannot send intrinsics - not connected");
                return false;
            }

            if (!intrinsics.IsValid())
            {
                Debug.LogError("[TcpBridge] Invalid camera intrinsics");
                return false;
            }

            _stopwatch.Restart();

            try
            {
                lock (_writeLock)
                {
                    // Build message: type (1) + payload (80)
                    _writeBuffer[0] = (byte)TcpMessageType.CameraIntrinsics;

                    int offset = 1;
                    offset += WriteInt32(_writeBuffer, offset, intrinsics.Width);
                    offset += WriteInt32(_writeBuffer, offset, intrinsics.Height);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.Fx);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.Fy);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.Cx);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.Cy);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.K1);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.K2);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.P1);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.P2);
                    offset += WriteDouble(_writeBuffer, offset, intrinsics.K3);

                    _stream.Write(_writeBuffer, 0, TcpMessageSizes.CameraIntrinsics);
                }

                _cachedIntrinsics = intrinsics;
                _intrinsicsSent = true;
                _sendCount++;

                _stopwatch.Stop();
                _totalSendTimeUs += _stopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;

                Debug.Log($"[TcpBridge] Sent camera intrinsics: {intrinsics}");
                return true;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Sends a frame-ready message with pose data.
        /// </summary>
        /// <param name="frameId">Current frame ID.</param>
        /// <param name="timestampSeconds">Timestamp in seconds.</param>
        /// <param name="pose">Camera pose matrix (4x4, row-major).</param>
        /// <param name="hasDepth">Whether the frame includes depth data.</param>
        /// <returns>True if sent successfully.</returns>
        public bool SendFrameReady(ulong frameId, double timestampSeconds, Matrix4x4 pose, bool hasDepth = true)
        {
            if (!IsConnected)
            {
                return false;
            }

            _stopwatch.Restart();

            try
            {
                lock (_writeLock)
                {
                    // Message type
                    _writeBuffer[0] = hasDepth
                        ? (byte)TcpMessageType.FrameReady
                        : (byte)TcpMessageType.FrameReadyNoDepth;

                    int offset = 1;

                    // Frame ID (8 bytes)
                    offset += WriteUInt64(_writeBuffer, offset, frameId);

                    // Timestamp in nanoseconds (8 bytes)
                    ulong timestampNs = (ulong)(timestampSeconds * 1_000_000_000);
                    offset += WriteUInt64(_writeBuffer, offset, timestampNs);

                    // Pose matrix (128 bytes = 16 doubles)
                    for (int row = 0; row < 4; row++)
                    {
                        for (int col = 0; col < 4; col++)
                        {
                            offset += WriteDouble(_writeBuffer, offset, pose[row, col]);
                        }
                    }

                    _stream.Write(_writeBuffer, 0, TcpMessageSizes.FrameReady);
                }

                _sendCount++;

                _stopwatch.Stop();
                _totalSendTimeUs += _stopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;

                return true;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Sends a frame-ready message with raw image data (fallback mode).
        /// Used when CUDA interop is unavailable (e.g., Vulkan backend).
        /// </summary>
        /// <param name="frameId">Current frame ID.</param>
        /// <param name="timestampSeconds">Timestamp in seconds.</param>
        /// <param name="pose">Camera pose matrix (4x4, row-major).</param>
        /// <param name="rgbWidth">RGB image width.</param>
        /// <param name="rgbHeight">RGB image height.</param>
        /// <param name="rgbData">Raw RGB image data (RGBA format).</param>
        /// <param name="depthWidth">Depth image width (0 if no depth).</param>
        /// <param name="depthHeight">Depth image height (0 if no depth).</param>
        /// <param name="depthData">Raw depth image data (R32F format), or null.</param>
        /// <returns>True if sent successfully.</returns>
        public bool SendFrameReadyWithData(
            ulong frameId,
            double timestampSeconds,
            Matrix4x4 pose,
            int rgbWidth,
            int rgbHeight,
            byte[] rgbData,
            int depthWidth,
            int depthHeight,
            byte[] depthData)
        {
            if (!IsConnected)
            {
                return false;
            }

            if (rgbData == null || rgbData.Length == 0)
            {
                Debug.LogError("[TcpBridge] RGB data is null or empty");
                return false;
            }

            bool hasDepth = depthData != null && depthData.Length > 0;

            _stopwatch.Restart();

            try
            {
                lock (_writeLock)
                {
                    // Calculate total message size
                    int headerSize = TcpMessageSizes.FrameReadyWithDataHeader;
                    int totalSize = headerSize + rgbData.Length + (hasDepth ? depthData.Length : 0);

                    // Allocate buffer if needed (reuse if possible)
                    byte[] buffer = totalSize <= _writeBuffer.Length 
                        ? _writeBuffer 
                        : new byte[totalSize];

                    // Message type
                    buffer[0] = hasDepth
                        ? (byte)TcpMessageType.FrameReadyWithData
                        : (byte)TcpMessageType.FrameReadyWithDataNoDepth;

                    int offset = 1;

                    // Frame ID (8 bytes)
                    offset += WriteUInt64(buffer, offset, frameId);

                    // Timestamp in nanoseconds (8 bytes)
                    ulong timestampNs = (ulong)(timestampSeconds * 1_000_000_000);
                    offset += WriteUInt64(buffer, offset, timestampNs);

                    // Pose matrix (128 bytes = 16 doubles)
                    for (int row = 0; row < 4; row++)
                    {
                        for (int col = 0; col < 4; col++)
                        {
                            offset += WriteDouble(buffer, offset, pose[row, col]);
                        }
                    }

                    // Image dimensions
                    offset += WriteInt32(buffer, offset, rgbWidth);
                    offset += WriteInt32(buffer, offset, rgbHeight);
                    offset += WriteInt32(buffer, offset, hasDepth ? depthWidth : 0);
                    offset += WriteInt32(buffer, offset, hasDepth ? depthHeight : 0);

                    // Data sizes
                    offset += WriteInt32(buffer, offset, rgbData.Length);
                    offset += WriteInt32(buffer, offset, hasDepth ? depthData.Length : 0);

                    // RGB data
                    Buffer.BlockCopy(rgbData, 0, buffer, offset, rgbData.Length);
                    offset += rgbData.Length;

                    // Depth data (if present)
                    if (hasDepth)
                    {
                        Buffer.BlockCopy(depthData, 0, buffer, offset, depthData.Length);
                        offset += depthData.Length;
                    }

                    // Send the entire message
                    _stream.Write(buffer, 0, totalSize);
                }

                _sendCount++;

                _stopwatch.Stop();
                _totalSendTimeUs += _stopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;

                return true;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Tries to read a velocity command without blocking.
        /// </summary>
        /// <param name="command">The received command.</param>
        /// <param name="isNew">True if this is a new command (different command ID).</param>
        /// <returns>True if a command was available.</returns>
        public bool TryReceiveCommand(out VelocityCommand command, out bool isNew)
        {
            command = VelocityCommand.Zero;
            isNew = false;

            if (!IsConnected)
            {
                return false;
            }

            try
            {
                // Check if data is available
                if (_clientSocket.Available == 0)
                {
                    // Return cached command if we have one
                    if (_hasNewCommand)
                    {
                        command = _pendingCommand;
                        isNew = true;
                        _hasNewCommand = false;
                        return true;
                    }
                    return false;
                }

                // Read available messages
                while (_clientSocket.Available > 0)
                {
                    if (!TryReadMessage())
                    {
                        break;
                    }
                }

                // Return any pending command
                if (_hasNewCommand)
                {
                    command = _pendingCommand;
                    isNew = true;
                    _hasNewCommand = false;
                    return true;
                }

                return false;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Blocks until a velocity command is received or timeout occurs.
        /// </summary>
        /// <param name="timeoutMs">Timeout in milliseconds.</param>
        /// <param name="command">The received command.</param>
        /// <returns>True if a command was received.</returns>
        public bool WaitForCommand(int timeoutMs, out VelocityCommand command)
        {
            command = VelocityCommand.Zero;

            if (!IsConnected)
            {
                return false;
            }

            _stopwatch.Restart();

            try
            {
                lock (_readLock)
                {
                    _stream.ReadTimeout = timeoutMs;

                    while (_stopwatch.ElapsedMilliseconds < timeoutMs)
                    {
                        if (TryReadMessage())
                        {
                            if (_hasNewCommand)
                            {
                                command = _pendingCommand;
                                _hasNewCommand = false;

                                _stopwatch.Stop();
                                _totalReceiveTimeUs += _stopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;

                                return true;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                return false;
            }
            catch (IOException ex) when (ex.InnerException is SocketException se &&
                                         se.SocketErrorCode == SocketError.TimedOut)
            {
                return false;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Sends a ping and waits for pong response.
        /// </summary>
        /// <param name="timeoutMs">Timeout in milliseconds.</param>
        /// <returns>True if pong was received.</returns>
        public bool Ping(int timeoutMs = 100)
        {
            if (!IsConnected)
            {
                return false;
            }

            try
            {
                lock (_writeLock)
                {
                    _writeBuffer[0] = (byte)TcpMessageType.Ping;
                    _stream.Write(_writeBuffer, 0, 1);
                }

                // Wait for pong
                var sw = Stopwatch.StartNew();
                while (sw.ElapsedMilliseconds < timeoutMs)
                {
                    if (_clientSocket.Available > 0)
                    {
                        lock (_readLock)
                        {
                            int bytesRead = _stream.Read(_readBuffer, 0, 1);
                            if (bytesRead > 0 && _readBuffer[0] == (byte)TcpMessageType.Pong)
                            {
                                return true;
                            }
                        }
                    }
                    Thread.Sleep(1);
                }

                return false;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Attempts to reconnect after disconnection.
        /// </summary>
        /// <returns>True if reconnection infrastructure is ready.</returns>
        public bool TryReconnect()
        {
            if (_isDisposed)
            {
                return false;
            }

            // Close existing client connection
            CloseClientConnection();

            // Server is still listening, will accept new connection
            _reconnectCount++;

            if (!_isListening)
            {
                return Initialize();
            }

            return true;
        }

        /// <summary>
        /// Gets a performance report string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[TcpBridge] Performance Report:\n" +
                   $"  Port: {_port}\n" +
                   $"  Connected: {IsConnected}\n" +
                   $"  Sends: {_sendCount} (avg {AverageSendTimeUs:F1}µs)\n" +
                   $"  Receives: {_receiveCount} (avg {AverageReceiveTimeUs:F1}µs)\n" +
                   $"  Errors: {_errorCount}\n" +
                   $"  Reconnects: {_reconnectCount}";
        }

        /// <summary>
        /// Resets performance metrics.
        /// </summary>
        public void ResetMetrics()
        {
            _totalSendTimeUs = 0;
            _totalReceiveTimeUs = 0;
            _sendCount = 0;
            _receiveCount = 0;
            _errorCount = 0;
            _reconnectCount = 0;
        }

        #endregion

        #region Private Methods

        private void AcceptLoop()
        {
            while (!_stopAcceptThread)
            {
                try
                {
                    // Poll for incoming connections
                    if (!_serverSocket.Poll(ACCEPT_POLL_INTERVAL_MS * 1000, SelectMode.SelectRead))
                    {
                        continue;
                    }

                    Socket client = _serverSocket.Accept();

                    // Close existing client if any
                    CloseClientConnection();

                    _clientSocket = client;
                    _clientSocket.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.NoDelay, true);
                    _clientSocket.ReceiveTimeout = _timeoutMs;
                    _clientSocket.SendTimeout = _timeoutMs;

                    _stream = new NetworkStream(_clientSocket, ownsSocket: false);
                    _isConnected = true;
                    _intrinsicsSent = false;

                    Debug.Log("[TcpBridge] Client connected");
                    OnClientConnected?.Invoke();

                    // Send cached intrinsics if we have them and haven't sent yet
                    // (OnClientConnected handler may have already sent intrinsics)
                    if (!_intrinsicsSent && _cachedIntrinsics.IsValid())
                    {
                        SendIntrinsics(_cachedIntrinsics);
                    }
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.Interrupted ||
                                                  ex.SocketErrorCode == SocketError.OperationAborted)
                {
                    break;
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[TcpBridge] Accept error: {ex.Message}");
                    Thread.Sleep(100);
                }
            }
        }

        private bool TryReadMessage()
        {
            try
            {
                // Read message type
                int bytesRead = _stream.Read(_readBuffer, 0, 1);
                if (bytesRead == 0)
                {
                    HandleDisconnection(null);
                    return false;
                }

                var msgType = (TcpMessageType)_readBuffer[0];

                switch (msgType)
                {
                    case TcpMessageType.VelocityCommand:
                        return ReadVelocityCommand();

                    case TcpMessageType.FrameProcessed:
                        return ReadFrameProcessed();

                    case TcpMessageType.RequestFrame:
                        return ReadFrameRequest();

                    case TcpMessageType.Ping:
                        // Respond with pong
                        lock (_writeLock)
                        {
                            _writeBuffer[0] = (byte)TcpMessageType.Pong;
                            _stream.Write(_writeBuffer, 0, 1);
                        }
                        return true;

                    case TcpMessageType.Pong:
                        // Ignore pong (handled by Ping method)
                        return true;

                    case TcpMessageType.Shutdown:
                        Debug.Log("[TcpBridge] Received shutdown signal");
                        HandleDisconnection(null);
                        return false;

                    default:
                        Debug.LogWarning($"[TcpBridge] Unknown message type: 0x{(byte)msgType:X2}");
                        _errorCount++;
                        return false;
                }
            }
            catch (IOException ex) when (ex.InnerException is SocketException se &&
                                         (se.SocketErrorCode == SocketError.TimedOut ||
                                          se.SocketErrorCode == SocketError.WouldBlock))
            {
                return false;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        private bool ReadVelocityCommand()
        {
            // Read remaining payload (32 bytes)
            int payloadSize = TcpMessageSizes.VelocityCommand - 1;
            int bytesRead = ReadFully(_readBuffer, 0, payloadSize);
            if (bytesRead != payloadSize)
            {
                return false;
            }

            int offset = 0;
            ulong commandId = ReadUInt64(_readBuffer, offset); offset += 8;
            double linearX = ReadDouble(_readBuffer, offset); offset += 8;
            double linearY = ReadDouble(_readBuffer, offset); offset += 8;
            double angularZ = ReadDouble(_readBuffer, offset);

            var command = new VelocityCommand
            {
                CommandId = commandId,
                LinearX = linearX,
                LinearY = linearY,
                AngularZ = angularZ
            };

            if (command.CommandId != _lastCommandId)
            {
                _pendingCommand = command;
                _hasNewCommand = true;
                _lastCommandId = command.CommandId;
                _receiveCount++;
                OnCommandReceived?.Invoke(command);
            }

            return true;
        }

        private bool ReadFrameProcessed()
        {
            // Read frame_id (8 bytes)
            int bytesRead = ReadFully(_readBuffer, 0, 8);
            if (bytesRead != 8)
            {
                return false;
            }

            // ulong frameId = ReadUInt64(_readBuffer, 0);
            // Could track frame acknowledgments here if needed
            _receiveCount++;

            return true;
        }

        private bool ReadFrameRequest()
        {
            // Read with_depth (1 byte)
            int bytesRead = ReadFully(_readBuffer, 0, 1);
            if (bytesRead != 1)
            {
                return false;
            }

            bool withDepth = _readBuffer[0] != 0;
            _receiveCount++;
            OnFrameRequested?.Invoke(withDepth);

            return true;
        }

        private int ReadFully(byte[] buffer, int offset, int count)
        {
            int totalRead = 0;
            while (totalRead < count)
            {
                int bytesRead = _stream.Read(buffer, offset + totalRead, count - totalRead);
                if (bytesRead == 0)
                {
                    break;
                }
                totalRead += bytesRead;
            }
            return totalRead;
        }

        private void HandleDisconnection(Exception ex)
        {
            if (!_isConnected)
            {
                return;
            }

            _isConnected = false;

            if (ex != null)
            {
                Debug.LogWarning($"[TcpBridge] Client disconnected: {ex.Message}");
            }
            else
            {
                Debug.Log("[TcpBridge] Client disconnected");
            }

            OnClientDisconnected?.Invoke();
            CloseClientConnection();
        }

        private void CloseClientConnection()
        {
            _isConnected = false;
            _intrinsicsSent = false;

            try { _stream?.Close(); _stream?.Dispose(); } catch { }
            _stream = null;

            try { _clientSocket?.Shutdown(SocketShutdown.Both); } catch { }
            try { _clientSocket?.Close(); _clientSocket?.Dispose(); } catch { }
            _clientSocket = null;
        }

        #region Binary Helpers

        private static int WriteInt32(byte[] buffer, int offset, int value)
        {
            buffer[offset + 0] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
            buffer[offset + 2] = (byte)((value >> 16) & 0xFF);
            buffer[offset + 3] = (byte)((value >> 24) & 0xFF);
            return 4;
        }

        private static int WriteUInt64(byte[] buffer, int offset, ulong value)
        {
            buffer[offset + 0] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
            buffer[offset + 2] = (byte)((value >> 16) & 0xFF);
            buffer[offset + 3] = (byte)((value >> 24) & 0xFF);
            buffer[offset + 4] = (byte)((value >> 32) & 0xFF);
            buffer[offset + 5] = (byte)((value >> 40) & 0xFF);
            buffer[offset + 6] = (byte)((value >> 48) & 0xFF);
            buffer[offset + 7] = (byte)((value >> 56) & 0xFF);
            return 8;
        }

        private static int WriteDouble(byte[] buffer, int offset, double value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            if (!BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            Array.Copy(bytes, 0, buffer, offset, 8);
            return 8;
        }

        private static ulong ReadUInt64(byte[] buffer, int offset)
        {
            return (ulong)buffer[offset] |
                   ((ulong)buffer[offset + 1] << 8) |
                   ((ulong)buffer[offset + 2] << 16) |
                   ((ulong)buffer[offset + 3] << 24) |
                   ((ulong)buffer[offset + 4] << 32) |
                   ((ulong)buffer[offset + 5] << 40) |
                   ((ulong)buffer[offset + 6] << 48) |
                   ((ulong)buffer[offset + 7] << 56);
        }

        private static double ReadDouble(byte[] buffer, int offset)
        {
            if (BitConverter.IsLittleEndian)
            {
                return BitConverter.ToDouble(buffer, offset);
            }
            else
            {
                byte[] bytes = new byte[8];
                Array.Copy(buffer, offset, bytes, 0, 8);
                Array.Reverse(bytes);
                return BitConverter.ToDouble(bytes, 0);
            }
        }

        #endregion

        #endregion

        #region IDisposable

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_isDisposed)
            {
                return;
            }

            if (disposing)
            {
                _stopAcceptThread = true;

                CloseClientConnection();

                try { _serverSocket?.Close(); _serverSocket?.Dispose(); } catch { }
                _serverSocket = null;

                if (_acceptThread != null && _acceptThread.IsAlive)
                {
                    _acceptThread.Join(1000);
                }

                _isListening = false;
            }

            _isDisposed = true;
            Debug.Log($"[TcpBridge] Disposed. {GetPerformanceReport()}");
        }

        ~TcpBridge()
        {
            Dispose(false);
        }

        #endregion
    }
}
