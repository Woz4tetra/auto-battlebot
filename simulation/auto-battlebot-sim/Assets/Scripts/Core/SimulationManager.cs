// Auto-Battlebot Simulation System
// Central simulation manager singleton that orchestrates all simulation components

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Central manager singleton that orchestrates all simulation components.
    /// Handles initialization ordering, lifecycle management, and provides
    /// access to simulation state.
    /// 
    /// Script Execution Order: -1000 (earliest)
    /// </summary>
    [DefaultExecutionOrder(-1000)]
    public class SimulationManager : MonoBehaviour
    {
        #region Singleton

        private static SimulationManager _instance;

        /// <summary>
        /// Singleton instance of the SimulationManager.
        /// </summary>
        public static SimulationManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindFirstObjectByType<SimulationManager>();

                    if (_instance == null)
                    {
                        Debug.LogError("[SimulationManager] No SimulationManager found in scene. " +
                            "Please add a SimulationManager to your scene.");
                    }
                }
                return _instance;
            }
        }

        /// <summary>
        /// Returns true if the SimulationManager instance exists.
        /// </summary>
        public static bool Exists => _instance != null;

        #endregion

        #region Events

        /// <summary>
        /// Fired when the simulation begins initialization.
        /// </summary>
        public event Action OnInitializationStarted;

        /// <summary>
        /// Fired when all initialization phases are complete.
        /// </summary>
        public event Action OnInitializationComplete;

        /// <summary>
        /// Fired when the simulation starts running.
        /// </summary>
        public event Action OnSimulationStarted;

        /// <summary>
        /// Fired when the simulation is stopped.
        /// </summary>
        public event Action OnSimulationStopped;

        /// <summary>
        /// Fired when the simulation is shutting down.
        /// </summary>
        public event Action OnShutdown;

        #endregion

        #region Serialized Fields

        [Header("Simulation Settings")]
        [SerializeField]
        [Tooltip("Simulation mode: Standalone (no C++ app), HardwareInLoop (with C++ app), or DataGeneration")]
        private RobotSimulationMode _currentMode = RobotSimulationMode.Standalone;

        [SerializeField]
        [Tooltip("Whether to automatically start the simulation after initialization")]
        private bool _autoStart = true;

        [SerializeField]
        [Tooltip("Enable verbose logging for debugging")]
        private bool _verboseLogging = false;

        #endregion

        #region Private Fields

        private readonly List<IInitializable> _initializables = new List<IInitializable>();
        private bool _isInitialized = false;
        private bool _isRunning = false;
        private bool _hasShutdown = false;

        #endregion

        #region Properties

        /// <summary>
        /// Returns true if all initialization phases have completed.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Returns true if the simulation is currently running.
        /// </summary>
        public bool IsRunning => _isRunning;

        /// <summary>
        /// Current simulation mode (HIL with C++ app, data generation, or standalone).
        /// </summary>
        public RobotSimulationMode CurrentMode => _currentMode;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            // Singleton enforcement
            if (_instance != null && _instance != this)
            {
                Debug.LogWarning("[SimulationManager] Duplicate SimulationManager detected. Destroying this instance.");
                Destroy(gameObject);
                return;
            }

            _instance = this;

            // Ensure we're a root GameObject before calling DontDestroyOnLoad
            if (transform.parent != null)
            {
                transform.SetParent(null);
            }
            DontDestroyOnLoad(gameObject);

            Log("SimulationManager initialized");
        }

        private void Start()
        {
            // Discover and register all IInitializable components in the scene
            DiscoverInitializables();

            // Run initialization sequence
            RunInitialization();

            // Auto-start if configured
            if (_autoStart && _isInitialized)
            {
                StartSimulation();
            }
        }

        private void OnDestroy()
        {
            if (_instance == this)
            {
                Shutdown();
                _instance = null;
            }
        }

        private void OnApplicationQuit()
        {
            Shutdown();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Registers an IInitializable component for managed initialization.
        /// Call this before Start() if you want the component to be initialized
        /// during the standard initialization sequence.
        /// </summary>
        public void RegisterInitializable(IInitializable initializable)
        {
            if (initializable == null)
            {
                Debug.LogWarning("[SimulationManager] Attempted to register null IInitializable");
                return;
            }

            if (!_initializables.Contains(initializable))
            {
                _initializables.Add(initializable);
                Log($"Registered initializable: {initializable.GetType().Name} (Phase: {initializable.Phase})");
            }
        }

        /// <summary>
        /// Unregisters an IInitializable component.
        /// </summary>
        public void UnregisterInitializable(IInitializable initializable)
        {
            if (_initializables.Remove(initializable))
            {
                Log($"Unregistered initializable: {initializable.GetType().Name}");
            }
        }

        /// <summary>
        /// Sets the simulation mode. Should be called before initialization.
        /// </summary>
        public void SetMode(RobotSimulationMode mode)
        {
            if (_isInitialized)
            {
                Debug.LogWarning("[SimulationManager] Cannot change mode after initialization");
                return;
            }

            _currentMode = mode;
            Log($"Simulation mode set to: {mode}");
        }

        /// <summary>
        /// Starts the simulation. Only valid after initialization is complete.
        /// </summary>
        public void StartSimulation()
        {
            if (!_isInitialized)
            {
                Debug.LogError("[SimulationManager] Cannot start simulation before initialization is complete");
                return;
            }

            if (_isRunning)
            {
                Debug.LogWarning("[SimulationManager] Simulation is already running");
                return;
            }

            _isRunning = true;
            Log("Simulation started");
            OnSimulationStarted?.Invoke();
        }

        /// <summary>
        /// Stops the simulation.
        /// </summary>
        public void StopSimulation()
        {
            if (!_isRunning)
            {
                Debug.LogWarning("[SimulationManager] Simulation is not running");
                return;
            }

            _isRunning = false;
            Log("Simulation stopped");
            OnSimulationStopped?.Invoke();
        }

        /// <summary>
        /// Restarts the simulation by stopping and re-initializing all components.
        /// </summary>
        public void RestartSimulation()
        {
            Log("Restarting simulation...");

            if (_isRunning)
            {
                StopSimulation();
            }

            // Shutdown all components
            ShutdownInitializables();

            _isInitialized = false;
            _hasShutdown = false;

            // Re-discover and re-initialize
            DiscoverInitializables();
            RunInitialization();

            if (_autoStart)
            {
                StartSimulation();
            }
        }

        #endregion

        #region Private Methods

        private void DiscoverInitializables()
        {
            _initializables.Clear();

            // Find all MonoBehaviours that implement IInitializable
            var initializables = FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None)
                .OfType<IInitializable>();

            foreach (var initializable in initializables)
            {
                RegisterInitializable(initializable);
            }

            Log($"Discovered {_initializables.Count} initializable components");
        }

        private void RunInitialization()
        {
            Log("Starting initialization sequence...");
            OnInitializationStarted?.Invoke();

            // Sort by phase
            var sortedInitializables = _initializables
                .OrderBy(i => (int)i.Phase)
                .ToList();

            // Initialize each phase
            foreach (InitializationPhase phase in Enum.GetValues(typeof(InitializationPhase)))
            {
                Log($"Running initialization phase: {phase}");

                var phaseComponents = sortedInitializables
                    .Where(i => i.Phase == phase)
                    .ToList();

                foreach (var component in phaseComponents)
                {
                    try
                    {
                        Log($"  Initializing: {component.GetType().Name}");
                        component.Initialize();
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[SimulationManager] Error initializing {component.GetType().Name}: {ex.Message}\n{ex.StackTrace}");
                    }
                }
            }

            _isInitialized = true;
            Log("Initialization complete");
            OnInitializationComplete?.Invoke();
        }

        private void ShutdownInitializables()
        {
            Log("Shutting down components...");

            // Shutdown in reverse order
            var reverseOrder = _initializables
                .OrderByDescending(i => (int)i.Phase)
                .ToList();

            foreach (var component in reverseOrder)
            {
                try
                {
                    Log($"  Shutting down: {component.GetType().Name}");
                    component.Shutdown();
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SimulationManager] Error shutting down {component.GetType().Name}: {ex.Message}");
                }
            }
        }

        private void Shutdown()
        {
            if (_hasShutdown)
            {
                return;
            }
            _hasShutdown = true;

            if (_isRunning)
            {
                StopSimulation();
            }

            OnShutdown?.Invoke();
            ShutdownInitializables();

            Log("SimulationManager shutdown complete");
        }

        private void Log(string message)
        {
            if (_verboseLogging)
            {
                Debug.Log($"[SimulationManager] {message}");
            }
        }

        #endregion
    }
}
