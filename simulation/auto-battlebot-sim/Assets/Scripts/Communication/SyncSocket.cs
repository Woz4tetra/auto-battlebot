// Auto-Battlebot Simulation System
// Synchronization socket for frame timing coordination between Unity and C++
//
// Protocol:
// - Unity sends FRAME_READY (1 byte: 0x01) after writing frame to shared memory
// - C++ sends COMMAND_READY (1 byte: 0x02) after writing command to shared memory
// - In lockstep mode, each side blocks until receiving the other's signal
// - In free-running mode, signals are sent but not waited for

using System;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Synchronization socket for coordinating frame timing between Unity and C++.
    /// Uses Unix domain sockets on Linux and TCP loopback on Windows.
    /// 
    /// Features:
    /// - Frame-ready signal from Unity to C++
    /// - Command-ready signal from C++ to Unity
    /// - Blocking wait with configurable timeout
    /// - Graceful connection/disconnection handling
    /// - Automatic reconnection support
    /// </summary>
    public class SyncSocket : IDisposable
    {
        #region Constants

        /// <summary>
        /// Default socket path for Unix domain sockets.
        /// </summary>
        public const string DEFAULT_SOCKET_PATH = "/tmp/auto_battlebot_sync.sock";

        /// <summary>
        /// Default port for Windows TCP fallback.
        /// </summary>
        public const int DEFAULT_TCP_PORT = 47821;

        /// <summary>
        /// Signal byte indicating frame is ready (Unity → C++).
        /// </summary>
        public const byte SIGNAL_FRAME_READY = 0x01;

        /// <summary>
        /// Signal byte indicating command is ready (C++ → Unity).
        /// </summary>
        public const byte SIGNAL_COMMAND_READY = 0x02;

        /// <summary>
        /// Signal byte for ping/heartbeat.
        /// </summary>
        public const byte SIGNAL_PING = 0x03;

        /// <summary>
        /// Signal byte for pong/heartbeat response.
        /// </summary>
        public const byte SIGNAL_PONG = 0x04;

        /// <summary>
        /// Default timeout in milliseconds.
        /// </summary>
        public const int DEFAULT_TIMEOUT_MS = 1000;

        /// <summary>
        /// Default reconnection interval in milliseconds.
        /// </summary>
        private const int RECONNECT_INTERVAL_MS = 500;

        #endregion

        #region Private Fields

        private readonly string _socketPath;
        private readonly int _tcpPort;
        private readonly SyncMode _mode;
        private readonly int _timeoutMs;
        private readonly bool _isServer;

        private Socket _socket;
        private Socket _clientSocket;  // For server mode
        private NetworkStream _stream;

        private bool _isDisposed = false;
        private bool _isConnected = false;
        private bool _isListening = false;

        private Thread _acceptThread;
        private volatile bool _stopAcceptThread = false;

        // Performance metrics
        private readonly Stopwatch _signalStopwatch = new Stopwatch();
        private long _totalSignalTimeUs = 0;
        private long _signalCount = 0;
        private long _maxSignalTimeUs = 0;
        private long _timeoutCount = 0;
        private long _reconnectCount = 0;

        // Buffer for reading
        private readonly byte[] _readBuffer = new byte[1];
        private readonly byte[] _writeBuffer = new byte[1];

        #endregion

        #region Properties

        /// <summary>
        /// Whether the socket is connected.
        /// </summary>
        public bool IsConnected => _isConnected && (_isServer ? _clientSocket?.Connected == true : _socket?.Connected == true);

        /// <summary>
        /// Whether the server is listening for connections.
        /// </summary>
        public bool IsListening => _isListening;

        /// <summary>
        /// The current synchronization mode.
        /// </summary>
        public SyncMode Mode => _mode;

        /// <summary>
        /// Total number of signals sent/received.
        /// </summary>
        public long SignalCount => _signalCount;

        /// <summary>
        /// Number of timeouts that occurred.
        /// </summary>
        public long TimeoutCount => _timeoutCount;

        /// <summary>
        /// Number of reconnections that occurred.
        /// </summary>
        public long ReconnectCount => _reconnectCount;

        /// <summary>
        /// Average signal latency in microseconds.
        /// </summary>
        public double AverageSignalTimeUs => _signalCount > 0 ? (double)_totalSignalTimeUs / _signalCount : 0;

        /// <summary>
        /// Maximum signal latency observed in microseconds.
        /// </summary>
        public long MaxSignalTimeUs => _maxSignalTimeUs;

        #endregion

        #region Events

        /// <summary>
        /// Fired when connection is established.
        /// </summary>
        public event Action OnConnected;

        /// <summary>
        /// Fired when connection is lost.
        /// </summary>
        public event Action OnDisconnected;

        /// <summary>
        /// Fired when a command-ready signal is received from C++.
        /// </summary>
        public event Action OnCommandReady;

        #endregion

        #region Constructor

        /// <summary>
        /// Creates a new SyncSocket.
        /// </summary>
        /// <param name="isServer">True if Unity acts as server (recommended).</param>
        /// <param name="mode">Synchronization mode.</param>
        /// <param name="socketPath">Unix socket path (Linux only).</param>
        /// <param name="tcpPort">TCP port for Windows fallback.</param>
        /// <param name="timeoutMs">Timeout for blocking operations.</param>
        public SyncSocket(
            bool isServer = true,
            SyncMode mode = SyncMode.Lockstep,
            string socketPath = null,
            int tcpPort = DEFAULT_TCP_PORT,
            int timeoutMs = DEFAULT_TIMEOUT_MS)
        {
            _isServer = isServer;
            _mode = mode;
            _socketPath = socketPath ?? DEFAULT_SOCKET_PATH;
            _tcpPort = tcpPort;
            _timeoutMs = timeoutMs;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Initializes the socket. For server mode, starts listening.
        /// For client mode, attempts to connect.
        /// </summary>
        /// <returns>True if initialization was successful.</returns>
        public bool Initialize()
        {
            if (_isDisposed)
            {
                Debug.LogError("[SyncSocket] Cannot initialize - already disposed");
                return false;
            }

            try
            {
                if (_isServer)
                {
                    return InitializeServer();
                }
                else
                {
                    return InitializeClient();
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SyncSocket] Initialization failed: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Sends frame-ready signal to C++.
        /// In lockstep mode, blocks until command-ready is received.
        /// </summary>
        /// <param name="frameId">Frame ID for logging.</param>
        /// <returns>True if signal was sent (and ack received in lockstep mode).</returns>
        public bool SignalFrameReady(ulong frameId = 0)
        {
            if (!IsConnected)
            {
                Debug.LogWarning("[SyncSocket] Cannot signal - not connected");
                return false;
            }

            _signalStopwatch.Restart();

            try
            {
                // Send frame-ready signal
                _writeBuffer[0] = SIGNAL_FRAME_READY;
                GetActiveStream().Write(_writeBuffer, 0, 1);

                if (_mode == SyncMode.Lockstep)
                {
                    // Wait for command-ready response
                    if (!WaitForSignal(SIGNAL_COMMAND_READY, _timeoutMs))
                    {
                        _timeoutCount++;
                        Debug.LogWarning($"[SyncSocket] Timeout waiting for command-ready (frame {frameId})");
                        return false;
                    }
                }

                _signalStopwatch.Stop();
                UpdateSignalMetrics();

                return true;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Sends command-ready signal to Unity (typically called by C++ side).
        /// For testing purposes in Unity.
        /// </summary>
        /// <returns>True if signal was sent.</returns>
        public bool SignalCommandReady()
        {
            if (!IsConnected)
            {
                return false;
            }

            try
            {
                _writeBuffer[0] = SIGNAL_COMMAND_READY;
                GetActiveStream().Write(_writeBuffer, 0, 1);
                return true;
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Waits for a specific signal with timeout.
        /// </summary>
        /// <param name="expectedSignal">The signal byte to wait for.</param>
        /// <param name="timeoutMs">Timeout in milliseconds.</param>
        /// <returns>True if signal was received.</returns>
        public bool WaitForSignal(byte expectedSignal, int timeoutMs)
        {
            if (!IsConnected)
            {
                return false;
            }

            try
            {
                var stream = GetActiveStream();
                stream.ReadTimeout = timeoutMs;

                int bytesRead = stream.Read(_readBuffer, 0, 1);
                if (bytesRead == 0)
                {
                    HandleDisconnection(null);
                    return false;
                }

                byte signal = _readBuffer[0];

                if (signal == expectedSignal)
                {
                    _signalCount++;

                    if (signal == SIGNAL_COMMAND_READY)
                    {
                        OnCommandReady?.Invoke();
                    }

                    return true;
                }
                else if (signal == SIGNAL_PING)
                {
                    // Respond to ping
                    _writeBuffer[0] = SIGNAL_PONG;
                    stream.Write(_writeBuffer, 0, 1);
                    // Recursively wait for actual signal
                    return WaitForSignal(expectedSignal, timeoutMs);
                }
                else
                {
                    Debug.LogWarning($"[SyncSocket] Unexpected signal: 0x{signal:X2}, expected 0x{expectedSignal:X2}");
                    return false;
                }
            }
            catch (IOException ex) when (ex.InnerException is SocketException se &&
                                         (se.SocketErrorCode == SocketError.TimedOut ||
                                          se.SocketErrorCode == SocketError.WouldBlock))
            {
                return false;  // Timeout or would-block (non-blocking socket)
            }
            catch (SocketException se) when (se.SocketErrorCode == SocketError.TimedOut ||
                                              se.SocketErrorCode == SocketError.WouldBlock)
            {
                return false;  // Timeout or would-block
            }
            catch (Exception ex)
            {
                HandleDisconnection(ex);
                return false;
            }
        }

        /// <summary>
        /// Attempts to reconnect after disconnection.
        /// </summary>
        /// <returns>True if reconnection was successful.</returns>
        public bool TryReconnect()
        {
            if (_isDisposed)
            {
                return false;
            }

            Debug.Log("[SyncSocket] Attempting reconnection...");

            // Clean up existing connection
            CloseClientConnection();

            if (_isServer)
            {
                // Server just waits for new client
                _reconnectCount++;
                return true;  // Accept thread will handle new connection
            }
            else
            {
                // Client attempts to reconnect
                if (InitializeClient())
                {
                    _reconnectCount++;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Sends a ping and waits for pong to check connection.
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
                _writeBuffer[0] = SIGNAL_PING;
                GetActiveStream().Write(_writeBuffer, 0, 1);
                return WaitForSignal(SIGNAL_PONG, timeoutMs);
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Gets performance metrics as a formatted string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[SyncSocket] Performance Report:\n" +
                   $"  Mode: {_mode}\n" +
                   $"  Signals: {_signalCount}\n" +
                   $"  Timeouts: {_timeoutCount}\n" +
                   $"  Reconnects: {_reconnectCount}\n" +
                   $"  Avg latency: {AverageSignalTimeUs:F1}µs\n" +
                   $"  Max latency: {_maxSignalTimeUs}µs";
        }

        /// <summary>
        /// Resets performance metrics.
        /// </summary>
        public void ResetMetrics()
        {
            _totalSignalTimeUs = 0;
            _signalCount = 0;
            _maxSignalTimeUs = 0;
            _timeoutCount = 0;
            _reconnectCount = 0;
        }

        #endregion

        #region Private Methods

        private bool InitializeServer()
        {
            if (IsLinux())
            {
                return InitializeUnixServer();
            }
            else
            {
                return InitializeTcpServer();
            }
        }

        private bool InitializeClient()
        {
            if (IsLinux())
            {
                return InitializeUnixClient();
            }
            else
            {
                return InitializeTcpClient();
            }
        }

        private bool InitializeUnixServer()
        {
            // Clean up existing socket file
            if (File.Exists(_socketPath))
            {
                File.Delete(_socketPath);
            }

            _socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
            _socket.Bind(new UnixDomainSocketEndPoint(_socketPath));
            _socket.Listen(1);

            _isListening = true;

            // Start accept thread
            _stopAcceptThread = false;
            _acceptThread = new Thread(AcceptLoop)
            {
                IsBackground = true,
                Name = "SyncSocket-Accept"
            };
            _acceptThread.Start();

            Debug.Log($"[SyncSocket] Server listening on Unix socket: {_socketPath}");
            return true;
        }

        private bool InitializeTcpServer()
        {
            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            _socket.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.NoDelay, true);
            _socket.Bind(new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, _tcpPort));
            _socket.Listen(1);

            _isListening = true;

            // Start accept thread
            _stopAcceptThread = false;
            _acceptThread = new Thread(AcceptLoop)
            {
                IsBackground = true,
                Name = "SyncSocket-Accept"
            };
            _acceptThread.Start();

            Debug.Log($"[SyncSocket] Server listening on TCP port: {_tcpPort}");
            return true;
        }

        private bool InitializeUnixClient()
        {
            if (!File.Exists(_socketPath))
            {
                Debug.LogWarning($"[SyncSocket] Unix socket does not exist: {_socketPath}");
                return false;
            }

            _socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
            _socket.Connect(new UnixDomainSocketEndPoint(_socketPath));

            _stream = new NetworkStream(_socket, ownsSocket: false);
            _isConnected = true;

            Debug.Log($"[SyncSocket] Connected to Unix socket: {_socketPath}");
            OnConnected?.Invoke();

            return true;
        }

        private bool InitializeTcpClient()
        {
            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            _socket.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.NoDelay, true);

            try
            {
                _socket.Connect(System.Net.IPAddress.Loopback, _tcpPort);
            }
            catch (SocketException)
            {
                Debug.LogWarning($"[SyncSocket] Could not connect to TCP port: {_tcpPort}");
                return false;
            }

            _stream = new NetworkStream(_socket, ownsSocket: false);
            _isConnected = true;

            Debug.Log($"[SyncSocket] Connected to TCP port: {_tcpPort}");
            OnConnected?.Invoke();

            return true;
        }

        private void AcceptLoop()
        {
            while (!_stopAcceptThread)
            {
                try
                {
                    // Use Poll to check for incoming connections (100ms timeout)
                    // This allows us to check _stopAcceptThread periodically
                    if (!_socket.Poll(100000, SelectMode.SelectRead))  // 100ms in microseconds
                    {
                        continue;  // No pending connection, check stop flag and loop
                    }

                    Socket client = _socket.Accept();

                    // Close any existing client
                    CloseClientConnection();

                    _clientSocket = client;

                    // Only set TCP options for TCP sockets (not Unix domain sockets)
                    if (_clientSocket.AddressFamily == AddressFamily.InterNetwork)
                    {
                        _clientSocket.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.NoDelay, true);
                    }

                    _stream = new NetworkStream(_clientSocket, ownsSocket: false);
                    _isConnected = true;

                    Debug.Log("[SyncSocket] Client connected");

                    // Invoke on main thread would be better, but for simplicity:
                    OnConnected?.Invoke();
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.Interrupted ||
                                                  ex.SocketErrorCode == SocketError.OperationAborted)
                {
                    // Socket was closed - exit loop
                    break;
                }
                catch (ObjectDisposedException)
                {
                    // Socket was disposed - exit loop
                    break;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[SyncSocket] Accept error: {ex.Message}");
                    Thread.Sleep(RECONNECT_INTERVAL_MS);
                }
            }
        }

        private NetworkStream GetActiveStream()
        {
            return _stream;
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
                Debug.LogWarning($"[SyncSocket] Disconnected: {ex.Message}");
            }
            else
            {
                Debug.Log("[SyncSocket] Connection closed");
            }

            OnDisconnected?.Invoke();

            // Clean up client connection (server keeps listening)
            CloseClientConnection();
        }

        private void CloseClientConnection()
        {
            _isConnected = false;

            try
            {
                _stream?.Close();
                _stream?.Dispose();
            }
            catch { }
            _stream = null;

            if (_isServer)
            {
                try
                {
                    _clientSocket?.Shutdown(SocketShutdown.Both);
                    _clientSocket?.Close();
                    _clientSocket?.Dispose();
                }
                catch { }
                _clientSocket = null;
            }
        }

        private void UpdateSignalMetrics()
        {
            long signalTimeUs = _signalStopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;
            _totalSignalTimeUs += signalTimeUs;
            _signalCount++;
            if (signalTimeUs > _maxSignalTimeUs)
            {
                _maxSignalTimeUs = signalTimeUs;
            }
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
        /// Releases all resources.
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Releases resources.
        /// </summary>
        protected virtual void Dispose(bool disposing)
        {
            if (_isDisposed)
            {
                return;
            }

            if (disposing)
            {
                // Stop accept thread
                _stopAcceptThread = true;

                // Close connections
                CloseClientConnection();

                // Close server socket
                try
                {
                    _socket?.Shutdown(SocketShutdown.Both);
                }
                catch { }

                try
                {
                    _socket?.Close();
                    _socket?.Dispose();
                }
                catch { }
                _socket = null;

                // Clean up Unix socket file
                if (IsLinux() && _isServer && File.Exists(_socketPath))
                {
                    try
                    {
                        File.Delete(_socketPath);
                    }
                    catch { }
                }

                // Wait for accept thread
                if (_acceptThread != null && _acceptThread.IsAlive)
                {
                    _acceptThread.Join(1000);
                }

                _isListening = false;
            }

            _isDisposed = true;

            Debug.Log($"[SyncSocket] Disposed. {GetPerformanceReport()}");
        }

        /// <summary>
        /// Finalizer.
        /// </summary>
        ~SyncSocket()
        {
            Dispose(false);
        }

        #endregion
    }
}
