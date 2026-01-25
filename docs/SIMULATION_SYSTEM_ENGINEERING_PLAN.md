# Auto-Battlebot Simulation System Engineering Plan

**Document Version:** 1.0  
**Date:** 2026-01-24  
**Author:** Engineering Team  
**Status:** Draft  

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Project Overview](#project-overview)
3. [Requirements](#requirements)
   - [Functional Requirements](#functional-requirements)
   - [Non-Functional Requirements](#non-functional-requirements)
4. [System Architecture](#system-architecture)
5. [Architecture Diagrams](#architecture-diagrams)
6. [Risk Assessment](#risk-assessment)
7. [Sprint Planning](#sprint-planning)
8. [Ticket Breakdown](#ticket-breakdown)

---

## Executive Summary

This document outlines the engineering plan for developing a Unity-based hardware-in-the-loop (HIL) simulation system for the Auto-Battlebot project. The simulation will enable end-to-end testing of the C++ perception and navigation stack without requiring physical hardware, accelerating development cycles and enabling automated testing scenarios.

The system will provide a virtual environment that accurately mimics the Stereolabs ZED camera output (RGB, depth, and visual SLAM pose) and receives velocity commands from the C++ application to control simulated robots. The simulation will support configurable autonomous opponent agents, enabling comprehensive testing of combat scenarios.

Additionally, the simulation will serve as a synthetic data generation platform** for training machine learning models. Python scripts will orchestrate the Unity simulation to produce large-scale, annotated datasets with automatic ground-truth labels, domain randomization, and configurable scenario generation—all without involving the C++ application.

**Key Deliverables:**
- High-performance IPC bridge between Unity and C++ application
- Physically accurate robot simulation with CAD-derived assets
- Configurable autonomous agent behaviors
- Scene lighting matching real NHRL arena conditions
- Extensible robot archetype system
- Python-orchestrated synthetic data generation pipeline
- Automatic ground-truth annotation export (bounding boxes, segmentation masks, keypoints)

---

## Project Overview

### Background

The Auto-Battlebot C++ application processes camera data from a Stereolabs ZED 2i to perceive the battle arena, detect robots, and generate velocity commands for a combat robot. Currently, testing requires physical hardware setup, which is time-consuming and limits iteration speed.

### Goals

1. Enable rapid iteration on perception and navigation algorithms without physical hardware
2. Provide deterministic, reproducible test scenarios
3. Support automated regression testing
4. Allow testing of edge cases and failure modes safely
5. Generate synthetic training data for machine learning models (orchestrated via Python, independent of C++ application)

### Scope

**In Scope:**
- Unity simulation mimicking ZED camera output
- Bidirectional communication with existing C++ application
- Robot physics simulation with accurate mass/inertia properties
- Configurable opponent AI behaviors
- Scene recreation matching NHRL arena lighting
- Python-orchestrated synthetic data generation pipeline
- Automatic ground-truth annotation export (bounding boxes, segmentation, keypoints)
- Domain randomization for training data diversity

**Out of Scope:**
- Weapon physics/damage simulation (Phase 2)
- Multi-robot coordination testing (Phase 2)
- VR/AR visualization modes
- Cloud-based simulation scaling
- Real-time training integration (online learning)

---

## Requirements

### Functional Requirements

| ID | Requirement | Priority | Rationale |
|----|-------------|----------|-----------|
| FR-001 | System shall render RGB images at minimum 720p resolution at 30+ FPS | Must | Match ZED camera minimum output specifications |
| FR-002 | System shall generate depth images aligned with RGB images | Must | Required for obstacle detection and navigation |
| FR-003 | System shall provide 6-DOF camera pose (visual SLAM equivalent) | Must | Required for world-frame robot tracking |
| FR-004 | System shall receive velocity commands (linear_x, linear_y, angular_z) from C++ application | Must | Core control interface |
| FR-005 | System shall apply velocity commands to simulated robot motors | Must | Enable closed-loop control testing |
| FR-006 | Image transfer latency shall be ≤10ms | Must | Maintain real-time control loop viability |
| FR-007 | System shall support at least 1 opponent robot at a time | Must | Match typical combat scenario |
| FR-008 | System shall support 0-1 neutral robots | Should | Test avoidance behaviors |
| FR-009 | Opponent robots shall implement configurable targeting AI | Must | Test defensive maneuvers |
| FR-010 | System shall load robot meshes from CAD-exported assets (DAE, FBX, OBJ) | Must | Maintain visual accuracy with real robots |
| FR-011 | System shall support robot archetypes: 4-wheel vertical spinner, 2-wheel horizontal spinner, 2-wheel wedge | Must | Cover common robot configurations |
| FR-012 | System shall be configurable via external configuration files | Must | Enable test scenario scripting |
| FR-013 | Camera intrinsics shall match ZED 2i specifications | Should | Ensure perception algorithm compatibility |
| FR-014 | Scene lighting shall replicate NHRL arena conditions | Should | Minimize domain gap in perception |
| FR-015 | System shall support recording/playback of simulation sessions | Should | Enable regression testing |
| FR-016 | System shall export 2D bounding box annotations for all robots | Must | Training data for object detection models |
| FR-017 | System shall export instance segmentation masks | Should | Training data for segmentation models |
| FR-018 | System shall export keypoint annotations matching robot joint definitions | Must | Training data for pose estimation models |
| FR-019 | System shall support domain randomization (lighting, textures, positions) | Must | Improve model generalization |
| FR-020 | Python orchestration shall control simulation without C++ application | Must | Decouple data generation from runtime system |
| FR-021 | System shall generate datasets in formats compatible with existing training pipeline | Must | Integration with `training/` directory structure |
| FR-022 | System shall support batch generation of thousands of annotated frames | Must | Scale synthetic data production |
| FR-023 | Annotations shall include object class labels matching `classes.toml` | Must | Consistency with existing label schema |
| FR-024 | System shall support up to 3 opponent robots at a time | Should | Match typical combat scenario |

### Non-Functional Requirements

| ID | Requirement | Priority | Rationale |
|----|-------------|----------|-----------|
| NFR-001 | Unity project shall use Unity 6 LTS or later | Must | Long-term support and modern rendering |
| NFR-002 | IPC mechanism shall support Linux (primary) and Windows (secondary) | Must | Match development environment |
| NFR-003 | System shall run on machines without dedicated GPU at reduced quality | Should | Enable CI/CD integration |
| NFR-004 | Memory usage shall not exceed 4GB for simulation | Should | Enable parallel test execution |
| NFR-005 | Codebase shall follow Unity C# conventions and include XML documentation | Must | Maintainability |
| NFR-006 | Adding a new robot shall require <30 minutes of configuration | Should | Extensibility |
| NFR-007 | System shall support headless operation | Should | CI/CD compatibility |
| NFR-008 | Scripts shall use explicit initialization order via Script Execution Order | Must | Avoid race conditions |
| NFR-009 | Python orchestration shall use existing project virtual environment | Must | Consistency with `training/` tooling |
| NFR-010 | Data generation shall achieve >100 annotated frames per minute | Should | Practical dataset sizes |
| NFR-011 | Annotation format shall be YOLO-compatible (txt) and COCO-compatible (JSON) | Must | Support common training frameworks |
| NFR-012 | Python scripts shall be executable from command line with configurable parameters | Must | Automation and scripting |

---

## System Architecture

### High-Level Architecture

The simulation system consists of four main layers:

1. **Unity Simulation Layer** - Handles rendering, physics, and robot control
2. **Communication Layer** - Manages bidirectional data transfer between Unity and C++
3. **C++ Application Layer** - Existing perception and navigation stack (unchanged)
4. **Python Data Generation Layer** - Orchestrates synthetic data generation (independent of C++)

### Communication Strategy

After evaluating options (ROS, gRPC, custom TCP, shared memory), I recommend **shared memory with memory-mapped files** for the following reasons:

- **Latency:** Sub-millisecond transfer times for large image buffers
- **Throughput:** No serialization overhead for raw image data
- **Simplicity:** Both Unity (via C# marshaling) and C++ support memory-mapped files natively
- **Cross-platform:** Works on Linux and Windows

A lightweight messaging layer using **Unix domain sockets** (or named pipes on Windows) will handle synchronization signals and metadata.

### Data Flow

```
Unity Simulation                    C++ Application
┌─────────────────┐                ┌─────────────────┐
│  Scene Renderer │────RGB+Depth──▶│ SimRgbdCamera   │
│  Camera System  │────Pose───────▶│ (implements     │
│                 │                │  RgbdCameraInterface)
│  Robot Physics  │◀───Velocity────│                 │
│  Motor Control  │                │ SimTransmitter  │
│                 │                │ (implements     │
│                 │                │  TransmitterInterface)
└─────────────────┘                └─────────────────┘
```

### Unity Component Architecture

```
SimulationManager (Singleton)
├── CameraSimulator
│   ├── RgbCameraCapture
│   ├── DepthCameraCapture
│   └── PoseTracker
├── CommunicationBridge
│   ├── SharedMemoryWriter (images)
│   ├── SharedMemoryReader (commands)
│   └── SyncSocket
├── RobotManager
│   ├── ControlledRobot
│   │   ├── DriveController
│   │   └── RobotModel
│   └── AutonomousRobot[]
│       ├── AIController
│       ├── DriveController
│       └── RobotModel
├── ArenaManager
│   ├── ArenaGeometry
│   └── LightingController
├── ConfigurationLoader
└── DataGenerationController (for synthetic data mode)
    ├── AnnotationExporter
    ├── DomainRandomizer
    └── ScenarioGenerator
```

### Synthetic Data Generation Architecture

The synthetic data generation pipeline operates independently of the C++ application, using Python to orchestrate Unity via a lightweight TCP command interface.

```
Python Orchestrator                    Unity Simulation
┌─────────────────────┐               ┌─────────────────────┐
│  DatasetGenerator   │───Commands───▶│ DataGenerationCtrl  │
│  - scenario configs │               │ - scene setup       │
│  - randomization    │               │ - robot positioning │
│  - batch control    │               │ - lighting control  │
│                     │               │                     │
│  AnnotationWriter   │◀──Frames+─────│ AnnotationExporter  │
│  - YOLO format      │   Annotations │ - bounding boxes    │
│  - COCO format      │               │ - segmentation      │
│  - keypoints        │               │ - keypoints         │
│                     │               │                     │
│  DomainRandomizer   │───Params─────▶│ Randomizer          │
│  - lighting ranges  │               │ - texture swap      │
│  - pose sampling    │               │ - lighting adjust   │
└─────────────────────┘               └─────────────────────┘
         │
         ▼
┌─────────────────────┐
│  training/data/     │
│  - images/          │
│  - labels/          │
│  - annotations.json │
└─────────────────────┘
```

### Python Component Architecture

```
training/generator/
├── generate_dataset.py          # Main entry point
├── unity_client.py              # TCP communication with Unity
├── scenario_sampler.py          # Random scenario generation
├── domain_randomizer.py         # Randomization parameter sampling
├── annotation_writer.py         # Multi-format annotation export
├── config/
│   └── generation_config.toml   # Dataset generation settings
└── schemas/
    ├── yolo_format.py           # YOLO annotation utilities
    └── coco_format.py           # COCO annotation utilities
```

---

## Architecture Diagrams

### System Data Flow Diagram

![Alt text](system_data_flow_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph SystemDataFlow {
    rankdir=LR;
    node [shape=box, style=filled];
    
    subgraph cluster_unity {
        label="Unity Simulation";
        style=filled;
        color=lightblue;
        
        scene [label="Scene\nRenderer", fillcolor=lightcyan];
        camera [label="Virtual\nCamera", fillcolor=lightcyan];
        physics [label="Physics\nEngine", fillcolor=lightcyan];
        motor [label="Motor\nController", fillcolor=lightcyan];
        ai [label="AI\nController", fillcolor=lightyellow];
        
        scene -> camera [label="render"];
        physics -> scene [label="transforms"];
        motor -> physics [label="forces"];
        ai -> motor [label="opponent\ncommands", style=dashed];
    }
    
    subgraph cluster_ipc {
        label="IPC Layer";
        style=filled;
        color=lightgray;
        
        shm_img [label="Shared Memory\n(Images)", shape=cylinder, fillcolor=white];
        shm_cmd [label="Shared Memory\n(Commands)", shape=cylinder, fillcolor=white];
        sync [label="Sync\nSocket", shape=diamond, fillcolor=white];
    }
    
    subgraph cluster_cpp {
        label="C++ Application";
        style=filled;
        color=lightgreen;
        
        sim_camera [label="SimRgbdCamera\n(RgbdCameraInterface)", fillcolor=palegreen];
        perception [label="Perception\nPipeline", fillcolor=palegreen];
        navigation [label="Navigation\nSystem", fillcolor=palegreen];
        sim_tx [label="SimTransmitter\n(TransmitterInterface)", fillcolor=palegreen];
        
        sim_camera -> perception [label="CameraData"];
        perception -> navigation [label="RobotDescriptions"];
        navigation -> sim_tx [label="VelocityCommand"];
    }
    
    camera -> shm_img [label="RGB+Depth\n+Pose"];
    shm_img -> sim_camera;
    sim_tx -> shm_cmd [label="velocity"];
    shm_cmd -> motor;
    
    sync -> camera [dir=both, style=dashed, label="frame sync"];
    sync -> sim_camera [dir=both, style=dashed];
}
```

</details>

### Unity Class Structure Diagram

![Alt text](unity_class_structure_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph UnityClassStructure {
    rankdir=TB;
    node [shape=record, style=filled, fillcolor=lightyellow];
    
    // Interfaces
    subgraph cluster_interfaces {
        label="Interfaces";
        color=gray;
        
        IRobotController [label="{«interface»\nIRobotController|+ Initialize() : void\l+ ApplyVelocity(vel: Vector3) : void\l+ GetPose() : Pose\l}"];
        IAIBehavior [label="{«interface»\nIAIBehavior|+ Initialize(config: AIConfig) : void\l+ Update() : VelocityCommand\l+ SetTarget(target: Transform) : void\l}"];
        ICameraSimulator [label="{«interface»\nICameraSimulator|+ CaptureRgb() : Texture2D\l+ CaptureDepth() : float[,]\l+ GetPose() : Matrix4x4\l}"];
    }
    
    // Core Managers
    subgraph cluster_managers {
        label="Managers (Singletons)";
        color=blue;
        
        SimulationManager [label="{SimulationManager|− instance : SimulationManager\l− config : SimulationConfig\l− isRunning : bool\l|+ Initialize() : void\l+ StartSimulation() : void\l+ StopSimulation() : void\l+ GetConfig() : SimulationConfig\l}", fillcolor=lightblue];
        
        RobotManager [label="{RobotManager|− controlledRobot : ControlledRobot\l− opponents : List\<AutonomousRobot\>\l− neutrals : List\<AutonomousRobot\>\l|+ SpawnRobot(config: RobotConfig) : Robot\l+ GetControlledRobot() : ControlledRobot\l+ GetOpponents() : List\<Robot\>\l}", fillcolor=lightblue];
        
        ArenaManager [label="{ArenaManager|− lighting : LightingConfig\l− boundaries : Collider[]\l|+ SetLighting(config: LightingConfig) : void\l+ GetBounds() : Bounds\l}", fillcolor=lightblue];
    }
    
    // Communication
    subgraph cluster_comm {
        label="Communication";
        color=green;
        
        CommunicationBridge [label="{CommunicationBridge|− imageWriter : SharedMemoryWriter\l− commandReader : SharedMemoryReader\l− syncSocket : UnixSocket\l|+ SendFrame(frame: FrameData) : void\l+ ReceiveCommand() : VelocityCommand\l+ WaitForSync() : void\l}", fillcolor=palegreen];
        
        SharedMemoryWriter [label="{SharedMemoryWriter|− memoryHandle : IntPtr\l− bufferSize : int\l|+ Write(data: byte[]) : void\l+ Dispose() : void\l}", fillcolor=palegreen];
        
        SharedMemoryReader [label="{SharedMemoryReader|− memoryHandle : IntPtr\l|+ Read\<T\>() : T\l+ Dispose() : void\l}", fillcolor=palegreen];
    }
    
    // Robots
    subgraph cluster_robots {
        label="Robot Components";
        color=orange;
        
        RobotBase [label="{«abstract»\nRobotBase|# config : RobotConfig\l# driveController : DriveController\l# rigidBody : Rigidbody\l|+ Initialize(config: RobotConfig) : void\l+ GetPose() : Pose\l# abstract OnUpdate() : void\l}", fillcolor=peachpuff];
        
        ControlledRobot [label="{ControlledRobot|− lastCommand : VelocityCommand\l|+ ApplyCommand(cmd: VelocityCommand) : void\l# override OnUpdate() : void\l}", fillcolor=peachpuff];
        
        AutonomousRobot [label="{AutonomousRobot|− aiBehavior : IAIBehavior\l− target : Transform\l|+ SetBehavior(behavior: IAIBehavior) : void\l# override OnUpdate() : void\l}", fillcolor=peachpuff];
        
        DriveController [label="{DriveController|− wheels : WheelCollider[]\l− driveType : DriveType\l|+ SetVelocity(linear: Vector3, angular: float) : void\l+ GetCurrentVelocity() : Vector3\l}", fillcolor=peachpuff];
    }
    
    // AI Behaviors
    subgraph cluster_ai {
        label="AI Behaviors";
        color=purple;
        
        AggressiveAI [label="{AggressiveAI|− chargeSpeed : float\l− attackDistance : float\l|+ Update() : VelocityCommand\l}", fillcolor=plum];
        
        PatrolAI [label="{PatrolAI|− waypoints : Vector3[]\l− currentWaypoint : int\l|+ Update() : VelocityCommand\l}", fillcolor=plum];
        
        NeutralAI [label="{NeutralAI|− avoidanceRadius : float\l|+ Update() : VelocityCommand\l}", fillcolor=plum];
    }
    
    // Camera
    subgraph cluster_camera {
        label="Camera System";
        color=red;
        
        CameraSimulator [label="{CameraSimulator|− rgbCamera : Camera\l− depthCamera : Camera\l− intrinsics : Matrix4x4\l|+ CaptureRgb() : Texture2D\l+ CaptureDepth() : float[,]\l+ GetPose() : Matrix4x4\l}", fillcolor=lightsalmon];
        
        ZED2iProfile [label="{ZED2iProfile|+ width : int = 1280\l+ height : int = 720\l+ fov : float = 110\l+ fx, fy, cx, cy : float\l}", fillcolor=lightsalmon];
    }
    
    // Configuration
    subgraph cluster_config {
        label="Configuration";
        color=brown;
        
        SimulationConfig [label="{SimulationConfig|+ frameRate : int\l+ robots : RobotConfig[]\l+ lighting : LightingConfig\l+ ipcSettings : IPCConfig\l}", fillcolor=wheat];
        
        RobotConfig [label="{RobotConfig|+ label : string\l+ group : RobotGroup\l+ archetype : RobotArchetype\l+ meshPath : string\l+ mass : float\l+ aiConfig : AIConfig\l}", fillcolor=wheat];
        
        AIConfig [label="{AIConfig|+ behaviorType : AIBehaviorType\l+ aggressiveness : float\l+ reactionTime : float\l+ parameters : Dictionary\l}", fillcolor=wheat];
    }
    
    // Data Generation (Synthetic Training Data)
    subgraph cluster_datagen {
        label="Data Generation";
        color=darkgreen;
        
        DataGenerationServer [label="{DataGenerationServer|− tcpListener : TcpListener\l− port : int\l|+ StartServer() : void\l+ HandleCommand(cmd: Command) : Response\l+ StopServer() : void\l}", fillcolor=darkseagreen];
        
        AnnotationExporter [label="{AnnotationExporter|− camera : Camera\l|+ GetBoundingBoxes() : BBox[]\l+ GetSegmentationMask() : Texture2D\l+ GetKeypoints() : Keypoint[]\l+ ExportAnnotations() : AnnotationData\l}", fillcolor=darkseagreen];
        
        DomainRandomizer [label="{DomainRandomizer|− config : RandomizationConfig\l− seed : int\l|+ RandomizeLighting() : void\l+ RandomizeGeometry() : void\l+ RandomizeAppearance() : void\l+ RandomizeCamera() : void\l+ SetSeed(seed: int) : void\l}", fillcolor=darkseagreen];
        
        ScenarioController [label="{ScenarioController|− currentScenario : Scenario\l|+ SetupScenario(config: ScenarioConfig) : void\l+ PlaceRobots(positions: Pose[]) : void\l+ Reset() : void\l}", fillcolor=darkseagreen];
    }
    
    // Relationships
    SimulationManager -> RobotManager [arrowhead=diamond];
    SimulationManager -> ArenaManager [arrowhead=diamond];
    SimulationManager -> CommunicationBridge [arrowhead=diamond];
    SimulationManager -> CameraSimulator [arrowhead=diamond];
    
    CommunicationBridge -> SharedMemoryWriter [arrowhead=diamond];
    CommunicationBridge -> SharedMemoryReader [arrowhead=diamond];
    
    RobotManager -> ControlledRobot [arrowhead=odiamond];
    RobotManager -> AutonomousRobot [arrowhead=odiamond, label="0..*"];
    
    RobotBase -> IRobotController [style=dashed, arrowhead=empty];
    ControlledRobot -> RobotBase [arrowhead=empty];
    AutonomousRobot -> RobotBase [arrowhead=empty];
    
    RobotBase -> DriveController [arrowhead=diamond];
    AutonomousRobot -> IAIBehavior [arrowhead=odiamond];
    
    AggressiveAI -> IAIBehavior [style=dashed, arrowhead=empty];
    PatrolAI -> IAIBehavior [style=dashed, arrowhead=empty];
    NeutralAI -> IAIBehavior [style=dashed, arrowhead=empty];
    
    CameraSimulator -> ICameraSimulator [style=dashed, arrowhead=empty];
    CameraSimulator -> ZED2iProfile [arrowhead=odiamond];
    
    SimulationConfig -> RobotConfig [arrowhead=odiamond, label="1..*"];
    RobotConfig -> AIConfig [arrowhead=odiamond];
    
    // Data Generation relationships
    SimulationManager -> DataGenerationServer [arrowhead=odiamond, style=dashed, label="optional"];
    DataGenerationServer -> AnnotationExporter [arrowhead=diamond];
    DataGenerationServer -> DomainRandomizer [arrowhead=diamond];
    DataGenerationServer -> ScenarioController [arrowhead=diamond];
    ScenarioController -> RobotManager [arrowhead=odiamond];
    AnnotationExporter -> CameraSimulator [arrowhead=odiamond];
}
```

</details>

### Robot Archetype Hierarchy Diagram

![Alt text](robot_archetype_hierarchy_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph RobotArchetypes {
    rankdir=TB;
    node [shape=record, style=filled];
    
    subgraph cluster_archetypes {
        label="Robot Archetype System";
        color=darkblue;
        
        RobotArchetype [label="{«abstract»\nRobotArchetype|# chassisCollider : Collider\l# wheelPositions : Vector3[]\l# weaponMount : Transform\l|+ abstract ConfigurePhysics(rb: Rigidbody) : void\l+ abstract GetWheelConfig() : WheelConfig[]\l+ LoadMesh(path: string) : void\l}", fillcolor=lightsteelblue];
        
        FourWheelVerticalSpinner [label="{FourWheelVerticalSpinner|− spinnerMass : float\l− spinnerSpeed : float\l|+ ConfigurePhysics(rb) : void\l+ GetWheelConfig() : WheelConfig[]\l}", fillcolor=lightskyblue];
        
        TwoWheelHorizontalSpinner [label="{TwoWheelHorizontalSpinner|− diskRadius : float\l− diskMass : float\l|+ ConfigurePhysics(rb) : void\l+ GetWheelConfig() : WheelConfig[]\l}", fillcolor=lightskyblue];
        
        TwoWheelWedge [label="{TwoWheelWedge|− wedgeAngle : float\l− flipperForce : float\l|+ ConfigurePhysics(rb) : void\l+ GetWheelConfig() : WheelConfig[]\l}", fillcolor=lightskyblue];
        
        WheelConfig [label="{WheelConfig|+ position : Vector3\l+ radius : float\l+ suspensionDistance : float\l+ motorTorque : float\l+ brakeTorque : float\l}", fillcolor=lemonchiffon];
    }
    
    FourWheelVerticalSpinner -> RobotArchetype [arrowhead=empty];
    TwoWheelHorizontalSpinner -> RobotArchetype [arrowhead=empty];
    TwoWheelWedge -> RobotArchetype [arrowhead=empty];
    
    FourWheelVerticalSpinner -> WheelConfig [arrowhead=odiamond, label="4"];
    TwoWheelHorizontalSpinner -> WheelConfig [arrowhead=odiamond, label="2"];
    TwoWheelWedge -> WheelConfig [arrowhead=odiamond, label="2"];
}
```

</details>

### Communication Sequence Diagram

![Alt text](communication_sequence_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph CommunicationSequence {
    rankdir=TB;
    node [shape=box];
    
    subgraph cluster_sequence {
        label="Frame Communication Sequence";
        color=gray;
        
        // Nodes represent states/actions
        unity_render [label="Unity: Render Frame", fillcolor=lightblue, style=filled];
        unity_capture [label="Unity: Capture RGB+Depth", fillcolor=lightblue, style=filled];
        unity_pose [label="Unity: Get Camera Pose", fillcolor=lightblue, style=filled];
        unity_write [label="Unity: Write to SharedMem", fillcolor=palegreen, style=filled];
        unity_signal [label="Unity: Signal Frame Ready", fillcolor=lightyellow, style=filled];
        
        cpp_wait [label="C++: Wait for Signal", fillcolor=lightyellow, style=filled];
        cpp_read [label="C++: Read from SharedMem", fillcolor=palegreen, style=filled];
        cpp_process [label="C++: Process Frame", fillcolor=lightcoral, style=filled];
        cpp_command [label="C++: Generate Command", fillcolor=lightcoral, style=filled];
        cpp_write [label="C++: Write Command", fillcolor=palegreen, style=filled];
        cpp_signal [label="C++: Signal Command Ready", fillcolor=lightyellow, style=filled];
        
        unity_readcmd [label="Unity: Read Command", fillcolor=palegreen, style=filled];
        unity_apply [label="Unity: Apply to Motors", fillcolor=lightblue, style=filled];
        
        // Sequence flow
        unity_render -> unity_capture;
        unity_capture -> unity_pose;
        unity_pose -> unity_write;
        unity_write -> unity_signal;
        unity_signal -> cpp_wait [style=dashed, label="IPC signal"];
        cpp_wait -> cpp_read;
        cpp_read -> cpp_process;
        cpp_process -> cpp_command;
        cpp_command -> cpp_write;
        cpp_write -> cpp_signal;
        cpp_signal -> unity_readcmd [style=dashed, label="IPC signal"];
        unity_readcmd -> unity_apply;
        unity_apply -> unity_render [label="next frame", style=dotted];
    }
}
```

</details>

### Synthetic Data Generation Flow Diagram

![Alt text](synthetic_data_generation_flow_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph SyntheticDataFlow {
    rankdir=TB;
    node [shape=box, style=filled];
    
    subgraph cluster_python {
        label="Python Orchestration";
        style=filled;
        color=lightyellow;
        
        config [label="Load Generation\nConfig", fillcolor=wheat];
        sampler [label="Scenario\nSampler", fillcolor=wheat];
        randomizer [label="Domain\nRandomizer", fillcolor=wheat];
        client [label="Unity\nTCP Client", fillcolor=gold];
        writer [label="Annotation\nWriter", fillcolor=wheat];
        
        config -> sampler;
        sampler -> randomizer;
        randomizer -> client;
    }
    
    subgraph cluster_unity {
        label="Unity Simulation";
        style=filled;
        color=lightblue;
        
        server [label="Command\nServer", fillcolor=lightcyan];
        scene [label="Scene\nController", fillcolor=lightcyan];
        capture [label="Frame\nCapture", fillcolor=lightcyan];
        annotator [label="Ground Truth\nAnnotator", fillcolor=palegreen];
        
        server -> scene [label="setup"];
        scene -> capture [label="render"];
        capture -> annotator [label="objects"];
    }
    
    subgraph cluster_output {
        label="Output Dataset";
        style=filled;
        color=lightgray;
        
        images [label="images/\n*.png", shape=folder, fillcolor=white];
        labels_yolo [label="labels/\n*.txt (YOLO)", shape=folder, fillcolor=white];
        labels_coco [label="annotations.json\n(COCO)", shape=note, fillcolor=white];
        keypoints [label="keypoints/\n*.json", shape=folder, fillcolor=white];
    }
    
    client -> server [label="TCP commands", style=bold];
    annotator -> client [label="annotations", style=bold];
    capture -> client [label="frame data", style=bold];
    
    writer -> images;
    writer -> labels_yolo;
    writer -> labels_coco;
    writer -> keypoints;
    
    client -> writer [label="save"];
}
```

</details>

### Domain Randomization Parameters Diagram

![Alt text](domain_randomization_parameters_diagram.png)

<details>
    <summary>Dot diagram code</summary>

```dot
digraph DomainRandomization {
    rankdir=LR;
    node [shape=record, style=filled, fillcolor=lightyellow];
    
    subgraph cluster_randomization {
        label="Domain Randomization Categories";
        color=purple;
        
        lighting [label="{Lighting|intensity: [0.5, 1.5]\lcolor_temp: [4000K, 6500K]\lshadow_strength: [0.3, 1.0]\lambient_intensity: [0.1, 0.5]\l}"];
        
        geometry [label="{Geometry|robot_positions: arena_bounds\lrobot_orientations: [0°, 360°]\lcamera_height: [0.8m, 1.2m]\lcamera_tilt: [-15°, 15°]\l}"];
        
        appearance [label="{Appearance|floor_texture: [concrete, metal, ...]\lrobot_material_roughness: [0.2, 0.8]\lrobot_color_variation: ±10%\ldirt/scratches: [0%, 30%]\l}"];
        
        scene [label="{Scene Composition|num_opponents: [1, 3]\lnum_neutrals: [0, 1]\locclusion_objects: [0, 2]\lbackground_clutter: [low, high]\l}"];
        
        camera [label="{Camera Effects|exposure: [0.8, 1.2]\lmotion_blur: [0%, 5%]\lnoise_level: [0%, 2%]\lchromatic_aberration: [0%, 1%]\l}"];
    }
}
```

</details>

---

## Risk Assessment

### Technical Risks

| ID | Risk | Probability | Impact | Mitigation Strategy |
|----|------|-------------|--------|---------------------|
| TR-001 | Shared memory IPC latency exceeds requirements | Low | High | Prototype IPC layer first; have TCP fallback ready |
| TR-002 | Unity physics does not accurately model robot dynamics | Medium | Medium | Tune physics parameters against real robot telemetry; accept approximation |
| TR-003 | Depth rendering does not match ZED sensor characteristics | Medium | Medium | Apply post-processing to simulate sensor noise and artifacts |
| TR-004 | Cross-platform shared memory complexity | Medium | Low | Abstract IPC behind interface; platform-specific implementations |
| TR-005 | Unity rendering performance insufficient for real-time | Low | High | Profile early; reduce quality settings; support headless mode |
| TR-006 | Script initialization order causes race conditions | Medium | Medium | Use explicit Script Execution Order; document dependencies |
| TR-007 | CAD mesh import issues (scale, orientation, format) | High | Low | Standardize export pipeline; create validation tool |
| TR-008 | Sim-to-real domain gap degrades model performance | High | High | Extensive domain randomization; validate with real data holdout |
| TR-009 | Annotation accuracy differs from manual labels | Medium | High | Validate against manually labeled subset; tune projection math |
| TR-010 | Python-Unity TCP communication unreliable | Low | Medium | Implement retry logic; use well-tested socket patterns |
| TR-011 | Synthetic data generation too slow for practical dataset sizes | Medium | Medium | Profile and optimize; support parallel Unity instances |

### Schedule Risks

| ID | Risk | Probability | Impact | Mitigation Strategy |
|----|------|-------------|--------|---------------------|
| SR-001 | IPC implementation takes longer than estimated | Medium | High | Allocate buffer time; can use TCP socket as MVP |
| SR-002 | Robot archetype system over-engineered | Medium | Medium | Start with concrete implementations; abstract later |
| SR-003 | Lighting tuning requires many iterations | High | Low | Capture reference images early; accept subpar accuracy |
| SR-004 | Domain randomization parameter tuning is time-consuming | High | Medium | Start with literature-based defaults; iterate based on model performance |

### Integration Risks

| ID | Risk | Probability | Impact | Mitigation Strategy |
|----|------|-------------|--------|---------------------|
| IR-001 | C++ interface changes break simulation | Low | Medium | Define interface contract; version the IPC protocol |
| IR-002 | Coordinate system mismatch (Unity Y-up vs. ROS convention) | High | Medium | Document coordinate transforms clearly; unit tests |
| IR-003 | Timing differences between simulation and real system | Medium | Medium | Support configurable time scaling; lockstep mode |
| IR-004 | Synthetic labels incompatible with existing training pipeline | Medium | High | Match existing `training/` conventions exactly; test early |
| IR-005 | Class label mismatch between simulation and `classes.toml` | Low | High | Use same TOML config; automated validation |

---

## Sprint Planning

### Sprint Overview

| Sprint | Duration | Focus | Key Deliverables |
|--------|----------|-------|------------------|
| Sprint 1 | 2 weeks | Foundation | IPC prototype, basic Unity structure, C++ interfaces |
| Sprint 2 | 2 weeks | Camera System | RGB/Depth capture, pose tracking, camera profile |
| Sprint 3 | 2 weeks | Robot System | Robot base classes, archetypes, controlled robot |
| Sprint 4 | 2 weeks | AI & Arena | Autonomous agents, arena setup, lighting |
| Sprint 5 | 2 weeks | Integration | End-to-end testing, configuration system, polish |
| Sprint 6 | 2 weeks | Synthetic Data | Python orchestration, annotation export, domain randomization |
| Sprint 7 | 1 week | Stabilization | Bug fixes, documentation, CI/CD integration |

### Sprint 1: Foundation (Weeks 1-2)

**Goal:** Establish communication infrastructure and project structure

**Tickets:** SIM-001 through SIM-005

### Sprint 2: Camera System (Weeks 3-4)

**Goal:** Implement virtual camera matching ZED 2i specifications

**Tickets:** SIM-006 through SIM-010

### Sprint 3: Robot System (Weeks 5-6)

**Goal:** Implement robot physics and archetype system

**Tickets:** SIM-011 through SIM-017

### Sprint 4: AI & Arena (Weeks 7-8)

**Goal:** Implement autonomous agents and arena environment

**Tickets:** SIM-018 through SIM-023

### Sprint 5: Integration (Weeks 9-10)

**Goal:** Full system integration and configuration

**Tickets:** SIM-024 through SIM-028

### Sprint 6: Synthetic Data Generation (Weeks 11-12)

**Goal:** Implement Python-orchestrated synthetic data generation pipeline

**Tickets:** SIM-032 through SIM-039

### Sprint 7: Stabilization (Week 13)

**Goal:** Polish, documentation, and CI/CD

**Tickets:** SIM-029 through SIM-031

---

## Ticket Breakdown

---

### SIM-001: Create Unity Project Structure and Script Execution Framework

**Sprint:** 1  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Set up the foundational Unity project structure with proper folder organization, assembly definitions, and script execution order configuration. This establishes the baseline for all subsequent development and ensures scripts initialize in a deterministic order.

**Tasks:**
1. Create folder structure: `Scripts/Core`, `Scripts/Communication`, `Scripts/Robots`, `Scripts/AI`, `Scripts/Camera`, `Scripts/Arena`, `Scripts/Configuration`
2. Create assembly definition files for each module to enforce dependency boundaries
3. Configure Script Execution Order in Project Settings for core managers
4. Create `SimulationManager` singleton with initialization lifecycle hooks
5. Implement `IInitializable` interface for components requiring ordered initialization
6. Create `InitializationPhase` enum (PreInit, Init, PostInit, Ready)
7. Set up `.gitignore` for Unity-specific files

**Acceptance Criteria:**
- [ ] Folder structure matches specification
- [ ] Assembly definitions compile without circular dependencies
- [ ] `SimulationManager` initializes before all other simulation scripts
- [ ] Components implementing `IInitializable` are called in correct phase order
- [ ] Project compiles without warnings
- [ ] Unity version is 6 LTS or later

**Dependencies:** None

**Notes:**
```
Execution Order Reference:
-1000: SimulationManager
-500: ConfigurationLoader
-100: CommunicationBridge
0: Default (RobotManager, ArenaManager, CameraSimulator)
100: Robot components
200: AI components
```

---

### SIM-002: Implement Shared Memory Writer for Unity (C#)

**Sprint:** 1  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Implement a high-performance shared memory writer in C# that can write image data (RGB and depth) to a memory-mapped file accessible by the C++ application. This is the core of the simulation's output pipeline.

**Tasks:**
1. Create `SharedMemoryWriter` class using `System.IO.MemoryMappedFiles`
2. Define memory layout structure:
   - Header (64 bytes): frame_id, timestamp, width, height, rgb_offset, depth_offset, pose_offset
   - RGB data: width × height × 3 bytes (BGR format for OpenCV compatibility)
   - Depth data: width × height × 4 bytes (float32)
   - Pose data: 64 bytes (4x4 matrix, row-major, float64)
3. Implement double-buffering to prevent tearing
4. Create platform abstraction for Linux (`/dev/shm/`) vs Windows (named memory)
5. Implement `IDisposable` pattern for proper cleanup
6. Add performance metrics (write time, throughput)

**Acceptance Criteria:**
- [ ] Can write 1280x720 RGB + depth frame in <5ms
- [ ] Memory layout is documented and matches C++ reader expectations
- [ ] Double-buffering prevents partial frame reads
- [ ] Works on Linux (primary) and Windows (secondary)
- [ ] Memory is properly released on disposal
- [ ] Unit tests verify write/read consistency

**Dependencies:** SIM-001

**Technical Notes:**
```csharp
// Memory layout (total: ~3.7MB for 720p)
struct FrameHeader {
    ulong frame_id;      // 8 bytes
    double timestamp;    // 8 bytes
    int width;           // 4 bytes
    int height;          // 4 bytes
    int rgb_offset;      // 4 bytes
    int depth_offset;    // 4 bytes
    int pose_offset;     // 4 bytes
    byte[28] reserved;   // padding to 64 bytes
}
```

---

### SIM-003: Implement Shared Memory Reader for Unity (C#)

**Sprint:** 1  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Implement a shared memory reader in C# that reads velocity commands from the C++ application. Commands are small and fixed-size, making this simpler than the image writer.

**Tasks:**
1. Create `SharedMemoryReader` class using `System.IO.MemoryMappedFiles`
2. Define command structure:
   - command_id (8 bytes): incrementing counter to detect new commands
   - linear_x (8 bytes): double
   - linear_y (8 bytes): double
   - angular_z (8 bytes): double
3. Implement polling with configurable timeout
4. Add command validation (sanity checks on velocity ranges)
5. Create platform abstraction matching writer

**Acceptance Criteria:**
- [ ] Can read commands in <1ms
- [ ] Detects new commands via command_id comparison
- [ ] Handles case where no new command is available
- [ ] Validates command values are within expected ranges
- [ ] Works on Linux and Windows
- [ ] Memory properly released on disposal

**Dependencies:** SIM-001

---

### SIM-004: Implement Synchronization Socket for Frame Timing

**Sprint:** 1  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** High  

**Description:**

Implement a lightweight synchronization mechanism using Unix domain sockets (Linux) or named pipes (Windows) to coordinate frame timing between Unity and C++. This prevents the C++ application from reading stale frames.

**Tasks:**
1. Create `SyncSocket` class with platform-specific implementations
2. Implement frame-ready signal from Unity to C++
3. Implement command-ready signal from C++ to Unity
4. Support blocking wait with timeout
5. Handle connection/disconnection gracefully
6. Add reconnection logic for development iteration

**Acceptance Criteria:**
- [ ] Signal latency <0.5ms
- [ ] Unity blocks on send until C++ acknowledges
- [ ] C++ can timeout if Unity stops sending
- [ ] Reconnection works without restarting either application
- [ ] Works in both lockstep and free-running modes
- [ ] Error states are logged clearly

**Dependencies:** SIM-001

---

### SIM-005: Implement SimRgbdCamera and SimTransmitter C++ Classes

**Sprint:** 1  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create C++ implementations of `RgbdCameraInterface` and `TransmitterInterface` that communicate with the Unity simulation via shared memory. These integrate with the existing auto-battlebot architecture.

**Tasks:**
1. Create `SimRgbdCamera` class implementing `RgbdCameraInterface`
   - Implement `initialize()`: open shared memory, connect sync socket
   - Implement `get()`: read frame from shared memory, populate `CameraData`
   - Implement `should_close()`: check for shutdown signal
2. Create `SimTransmitter` class implementing `TransmitterInterface`
   - Implement `initialize()`: open command shared memory
   - Implement `send()`: write `VelocityCommand` to shared memory
   - Implement `update()`: return empty `CommandFeedback` (simulation mode)
   - Implement `did_init_button_press()`: return true after first frame
3. Add configuration options for shared memory paths
4. Create factory functions for simulation mode

**Acceptance Criteria:**
- [ ] `SimRgbdCamera::get()` returns valid `CameraData` with RGB, depth, and pose
- [ ] `SimTransmitter::send()` correctly writes velocity commands
- [ ] Integration test passes with mock Unity writer
- [ ] Memory leaks checked with Valgrind
- [ ] Existing unit tests still pass
- [ ] Configuration via TOML matches existing patterns

**Dependencies:** SIM-002, SIM-003, SIM-004

**Files to Create:**
- `include/rgbd_camera/sim_rgbd_camera.hpp`
- `src/rgbd_camera/sim_rgbd_camera.cpp`
- `include/transmitter/sim_transmitter.hpp`
- `src/transmitter/sim_transmitter.cpp`

---

### SIM-006: Implement RGB Camera Capture in Unity

**Sprint:** 2  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Implement the RGB camera capture system that renders the scene and extracts pixel data for transmission to the C++ application. Must match ZED 2i camera characteristics.

**Tasks:**
1. Create `CameraSimulator` MonoBehaviour
2. Configure camera with ZED 2i intrinsics:
   - Resolution: 1280x720 (configurable to 1920x1080, 640x360)
   - FOV: 110° horizontal
   - Intrinsic matrix matching ZED SDK values
3. Set up RenderTexture for off-screen rendering
4. Implement efficient GPU→CPU texture readback using `AsyncGPUReadback`
5. Convert to BGR format for OpenCV compatibility
6. Profile and optimize for target frame rate

**Acceptance Criteria:**
- [ ] Renders at 30+ FPS at 720p
- [ ] Camera intrinsics match ZED 2i specifications
- [ ] AsyncGPUReadback reduces CPU stalls
- [ ] Output format is BGR uint8 (OpenCV compatible)
- [ ] Resolution is configurable
- [ ] Lens distortion can be optionally applied

**Dependencies:** SIM-001

**Technical Notes:**
```
ZED 2i Intrinsics (720p):
fx = 528.0, fy = 528.0
cx = 640.0, cy = 360.0
k1, k2, p1, p2, k3 = [distortion coefficients]
```

---

### SIM-007: Implement Depth Camera Capture in Unity

**Sprint:** 2  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Implement depth image capture that generates per-pixel depth values matching the ZED camera's depth output characteristics. Depth should be in meters and aligned with the RGB image.

**Tasks:**
1. Create depth rendering camera with custom shader
2. Implement linearized depth calculation (Unity depth is non-linear)
3. Configure depth range matching ZED 2i (0.3m - 20m)
4. Render to RenderTexture with R32_SFloat format
5. Implement depth readback to float array
6. Add optional depth noise simulation (Gaussian noise, distance-dependent)
7. Handle sky/infinity as special depth value (0 or max_depth)

**Acceptance Criteria:**
- [ ] Depth values are in meters
- [ ] Depth is aligned with RGB image (same camera pose)
- [ ] Valid range is 0.3m - 20m
- [ ] Invalid depth (sky, out of range) handled consistently
- [ ] Depth noise approximates real sensor characteristics
- [ ] Performance impact <2ms per frame

**Dependencies:** SIM-006

**Shader Reference:**
```hlsl
// Linear eye depth from Unity depth buffer
float LinearEyeDepth(float z) {
    return 1.0 / (_ZBufferParams.z * z + _ZBufferParams.w);
}
```

---

### SIM-008: Implement Camera Pose Tracking

**Sprint:** 2  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Track the camera's 6-DOF pose in the simulation world frame and convert it to the coordinate system expected by the C++ application (ROS conventions). This mimics the ZED's visual SLAM output.

**Tasks:**
1. Extract camera transform from Unity (position + rotation)
2. Convert from Unity coordinate system (Y-up, left-handed) to ROS (Z-up, right-handed)
3. Build 4x4 homogeneous transformation matrix
4. Implement transform from world frame to visual odometry frame
5. Add optional pose noise for realism
6. Ensure timestamp synchronization with image capture

**Acceptance Criteria:**
- [ ] Pose is a valid 4x4 homogeneous transform
- [ ] Coordinate system matches `TransformStamped` expectations
- [ ] Rotation is valid (orthonormal, det=1)
- [ ] Pose timestamp matches image timestamps
- [ ] Transform chain is documented

**Dependencies:** SIM-006

**Coordinate Transform:**
```
Unity:     X-right, Y-up, Z-forward (left-handed)
ROS/C++:   X-forward, Y-left, Z-up (right-handed)

T_ros = T_convert * T_unity * T_convert^-1
```

---

### SIM-009: Create ZED 2i Camera Profile Configuration

**Sprint:** 2  
**Estimate:** 2 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Create a data-driven camera profile system that encapsulates ZED 2i specifications and allows for future camera profiles. This centralizes camera parameters for easy maintenance.

**Tasks:**
1. Create `CameraProfile` ScriptableObject
2. Define fields for all ZED 2i parameters:
   - Resolution presets (720p, 1080p, VGA)
   - Intrinsic matrix
   - Distortion coefficients
   - Depth range
   - Frame rate limits
3. Create default ZED 2i profile asset
4. Integrate profile with `CameraSimulator`
5. Support runtime profile switching

**Acceptance Criteria:**
- [ ] ZED 2i profile matches official specifications
- [ ] Profile is editable in Unity Inspector
- [ ] Camera uses profile parameters at runtime
- [ ] Profile can be changed without code modification
- [ ] Documentation includes parameter sources

**Dependencies:** SIM-006

---

### SIM-010: Integrate Camera System with Communication Bridge

**Sprint:** 2  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Connect the camera capture system to the shared memory communication bridge, implementing the complete frame transmission pipeline from render to C++ availability.

**Tasks:**
1. Create `CommunicationBridge` MonoBehaviour as coordinator
2. Implement frame assembly (header + RGB + depth + pose)
3. Wire camera capture callbacks to bridge
4. Implement frame rate limiting
5. Add frame skip detection and logging
6. Create performance dashboard (optional)

**Acceptance Criteria:**
- [ ] Complete frames written to shared memory at target rate
- [ ] Frame assembly <2ms overhead
- [ ] Dropped frames are logged with reason
- [ ] C++ application receives valid CameraData
- [ ] End-to-end latency <15ms (capture to C++ receive)

**Dependencies:** SIM-002, SIM-003, SIM-004, SIM-006, SIM-007, SIM-008

---

### SIM-011: Implement Robot Base Class and Interface

**Sprint:** 3  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the foundational robot class hierarchy that all simulated robots inherit from. This provides common functionality for physics, positioning, and state management.

**Tasks:**
1. Create `IRobotController` interface
2. Create abstract `RobotBase` MonoBehaviour
   - Rigidbody configuration
   - Pose getter/setter
   - Collision layer setup
   - Debug visualization
3. Implement common physics setup (mass, drag, angular drag)
4. Create `RobotState` struct (position, velocity, health placeholder)
5. Implement bounds checking against arena
6. Add reset/respawn functionality

**Acceptance Criteria:**
- [ ] `RobotBase` compiles and can be inherited
- [ ] Rigidbody configured with sensible defaults
- [ ] Pose getter returns correct world-space pose
- [ ] Robots stay within arena bounds
- [ ] Reset returns robot to spawn position
- [ ] Debug gizmos show robot bounds and orientation

**Dependencies:** SIM-001

---

### SIM-012: Implement Drive Controller for Robot Locomotion

**Sprint:** 3  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the drive controller that translates velocity commands into wheel/track forces. Must support differential drive (2-wheel) and mecanum-style (4-wheel) configurations.

**Tasks:**
1. Create `DriveController` component
2. Define `DriveType` enum (Differential, Mecanum, Tank)
3. Implement differential drive kinematics:
   - `linear_x` → forward thrust
   - `angular_z` → differential wheel speeds
4. Implement mecanum drive kinematics:
   - Full holonomic motion (linear_x, linear_y, angular_z)
5. Configure wheel colliders for ground contact
6. Add velocity limiting and acceleration ramping
7. Tune friction and slip parameters

**Acceptance Criteria:**
- [ ] Differential drive responds correctly to (linear_x, 0, angular_z)
- [ ] Mecanum drive responds correctly to (linear_x, linear_y, angular_z)
- [ ] Maximum velocities are configurable
- [ ] Acceleration is smooth (no instant velocity changes)
- [ ] Robots maintain traction on arena floor
- [ ] Works with different wheel configurations

**Dependencies:** SIM-011

**Kinematics Reference:**
```
Differential Drive:
v_left = linear_x - angular_z * wheel_base / 2
v_right = linear_x + angular_z * wheel_base / 2

Mecanum Drive:
v_fl = linear_x - linear_y - angular_z * (L + W)
v_fr = linear_x + linear_y + angular_z * (L + W)
v_rl = linear_x + linear_y - angular_z * (L + W)
v_rr = linear_x - linear_y + angular_z * (L + W)
```

---

### SIM-013: Implement Controlled Robot Class

**Sprint:** 3  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the `ControlledRobot` class representing the robot controlled by the C++ application. This robot receives velocity commands from the communication bridge and applies them via the drive controller.

**Tasks:**
1. Create `ControlledRobot` class extending `RobotBase`
2. Implement command receiving from `CommunicationBridge`
3. Apply commands to `DriveController`
4. Handle command timeout (stop if no commands received)
5. Expose state for debugging (last command, velocity)
6. Add command interpolation for smooth motion

**Acceptance Criteria:**
- [ ] Receives commands from C++ application via shared memory
- [ ] Applies commands to drive system immediately
- [ ] Stops gracefully if commands timeout (>100ms)
- [ ] Command history available for debugging
- [ ] Integrates with `RobotManager`

**Dependencies:** SIM-011, SIM-012, SIM-010

---

### SIM-014: Implement Robot Archetype Base and Factory

**Sprint:** 3  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** High  

**Description:**

Create the archetype system that provides templates for common robot configurations. Archetypes define wheel placement, weapon mounting, and physics properties that can be customized per-robot.

**Tasks:**
1. Create abstract `RobotArchetype` ScriptableObject
2. Define archetype properties:
   - Wheel positions (relative to chassis center)
   - Wheel configuration (radius, suspension, torque)
   - Weapon mount point
   - Default mass distribution
   - Collision mesh references
3. Create `RobotArchetypeFactory` for instantiation
4. Implement mesh loading from path
5. Create prefab generation from archetype + mesh

**Acceptance Criteria:**
- [ ] Archetypes are editable ScriptableObjects
- [ ] Factory creates configured robots from archetype + config
- [ ] Mesh loading works for DAE, FBX, OBJ formats
- [ ] Generated robots have correct physics properties
- [ ] Archetype system is documented with examples

**Dependencies:** SIM-011, SIM-012

---

### SIM-015: Implement Four-Wheel Vertical Spinner Archetype

**Sprint:** 3  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** High  

**Description:**

Create the archetype for four-wheel vertical spinner robots (like MR STABS). This is the primary archetype for the controlled robot.

**Tasks:**
1. Create `FourWheelVerticalSpinnerArchetype` ScriptableObject
2. Configure wheel positions for 4-corner layout
3. Set up weapon spinner attachment point (front-mounted)
4. Configure mass distribution (heavy front for spinner)
5. Create archetype asset with default values
6. Test with MR STABS MK1 and MK2 meshes

**Acceptance Criteria:**
- [ ] Four wheels positioned correctly
- [ ] Weapon mount positioned at front
- [ ] Works with existing MR STABS meshes
- [ ] Physics feel appropriate for combat robot
- [ ] Can customize wheel spacing and spinner position

**Dependencies:** SIM-014

---

### SIM-016: Implement Two-Wheel Horizontal Spinner Archetype

**Sprint:** 3  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** High  

**Description:**

Create the archetype for two-wheel horizontal spinner robots (like MRS BUFF). These are common opponent configurations.

**Tasks:**
1. Create `TwoWheelHorizontalSpinnerArchetype` ScriptableObject
2. Configure wheel positions for rear-mounted wheels
3. Set up horizontal weapon disk mount
4. Configure mass distribution (centered for balance)
5. Create archetype asset
6. Test with MRS BUFF MK1 and MK2 meshes

**Acceptance Criteria:**
- [ ] Two wheels positioned at rear
- [ ] Horizontal spinner mount configured
- [ ] Works with MRS BUFF meshes
- [ ] Differential drive kinematics work correctly
- [ ] Customizable weapon disk position

**Dependencies:** SIM-014

---

### SIM-017: Implement Two-Wheel Wedge Archetype

**Sprint:** 3  
**Estimate:** 2 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Create the archetype for two-wheel wedge robots. These are simpler robots often used as neutral bots or basic opponents.

**Tasks:**
1. Create `TwoWheelWedgeArchetype` ScriptableObject
2. Configure wheel positions
3. Set up wedge collision geometry
4. Configure low center of mass
5. Create archetype asset

**Acceptance Criteria:**
- [ ] Two wheels configured correctly
- [ ] Wedge collision works as expected
- [ ] Low center of gravity prevents tipping
- [ ] Suitable for neutral robot behavior

**Dependencies:** SIM-014

---

### SIM-018: Implement Autonomous Robot Class

**Sprint:** 4  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the `AutonomousRobot` class for AI-controlled robots (opponents and neutrals). These robots use behavior components to generate their own velocity commands.

**Tasks:**
1. Create `AutonomousRobot` class extending `RobotBase`
2. Implement `IAIBehavior` interface hookup
3. Create behavior update loop (runs in FixedUpdate)
4. Implement target assignment system
5. Add behavior switching capability
6. Support pause/resume for debugging

**Acceptance Criteria:**
- [ ] Autonomous robots move independently
- [ ] Behavior can be assigned at runtime
- [ ] Targets can be set and changed
- [ ] Behavior updates at physics rate
- [ ] Can pause AI for debugging

**Dependencies:** SIM-011, SIM-012

---

### SIM-019: Implement Aggressive AI Behavior

**Sprint:** 4  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Implement the aggressive AI behavior that actively pursues and attacks the target robot. This is the primary opponent behavior.

**Tasks:**
1. Create `AggressiveAI` class implementing `IAIBehavior`
2. Implement target tracking (position prediction)
3. Create pursuit steering behavior
4. Implement attack patterns (charge, circle, feint)
5. Add configurable parameters:
   - Aggression level (affects attack frequency)
   - Reaction time (artificial delay)
   - Accuracy (aim wobble)
6. Integrate with pathfinding (simple obstacle avoidance)

**Acceptance Criteria:**
- [ ] AI pursues target robot
- [ ] Attack patterns vary based on configuration
- [ ] Reaction time creates exploitable delays
- [ ] AI avoids arena walls
- [ ] Difficulty is configurable via parameters
- [ ] Behavior is deterministic given same seed

**Dependencies:** SIM-018

---

### SIM-020: Implement Patrol AI Behavior

**Sprint:** 4  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Implement patrol behavior where robots follow waypoints, useful for creating predictable opponent patterns or neutral robot movement.

**Tasks:**
1. Create `PatrolAI` class implementing `IAIBehavior`
2. Implement waypoint following
3. Support loop and ping-pong patrol modes
4. Add dwell time at waypoints
5. Implement smooth cornering

**Acceptance Criteria:**
- [ ] Robot follows waypoint sequence
- [ ] Loop and ping-pong modes work
- [ ] Smooth movement between waypoints
- [ ] Dwell time is configurable
- [ ] Can be combined with aggression (patrol until target spotted)

**Dependencies:** SIM-018

---

### SIM-021: Implement Neutral AI Behavior

**Sprint:** 4  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Implement neutral robot behavior that avoids combat. Neutral robots should wander and avoid both the controlled robot and opponents.

**Tasks:**
1. Create `NeutralAI` class implementing `IAIBehavior`
2. Implement avoidance steering for all robots
3. Create random wandering behavior
4. Add configurable avoidance radius
5. Implement flee behavior when approached

**Acceptance Criteria:**
- [ ] Neutral robot avoids all other robots
- [ ] Wanders when not fleeing
- [ ] Avoidance radius is configurable
- [ ] Does not get stuck in corners
- [ ] Prioritizes survival over pathing

**Dependencies:** SIM-018

---

### SIM-022: Set Up NHRL Arena Environment

**Sprint:** 4  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** High  

**Description:**

Configure the simulation arena to match the NHRL (National Havoc Robot League) battle box specifications, including geometry, materials, and boundaries.

**Tasks:**
1. Import and configure NHRL Cage model
2. Set up collision boundaries
3. Configure floor material (friction properties)
4. Set up wall collision layers
5. Add arena bounds for robot containment
6. Create hazard zones (if applicable)
7. Configure physics layers and collision matrix

**Acceptance Criteria:**
- [ ] Arena matches NHRL specifications (dimensions)
- [ ] Robots cannot escape arena bounds
- [ ] Floor friction feels appropriate
- [ ] Walls have proper collision response
- [ ] Physics layers prevent unwanted collisions
- [ ] Arena loads from existing NHRL Cage asset

**Dependencies:** SIM-001

---

### SIM-023: Implement Arena Lighting System

**Sprint:** 4  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** High  

**Description:**

Configure lighting to match real NHRL arena conditions for minimal domain gap between simulation and real camera footage.

**Tasks:**
1. Research NHRL lighting setup (overhead fluorescent/LED arrays)
2. Configure HDRP lighting settings
3. Create main overhead light array
4. Add fill lights to reduce harsh shadows
5. Configure ambient lighting
6. Set up light probes for indirect lighting on robots
7. Create reference comparison tool (real vs sim screenshots)
8. Fine-tune based on real footage

**Acceptance Criteria:**
- [ ] Lighting subjectively matches real footage
- [ ] No extreme shadows that obscure robots
- [ ] Consistent lighting across arena
- [ ] Robot materials render correctly
- [ ] Performance within budget (no dynamic shadow updates needed)
- [ ] Reference images documented

**Dependencies:** SIM-022

---

### SIM-024: Implement Configuration System

**Sprint:** 5  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create a comprehensive configuration system that allows scenarios to be defined in external files (TOML format for consistency with C++ application).

**Tasks:**
1. Create `ConfigurationLoader` class
2. Define TOML schema for simulation configuration:
   - Simulation settings (frame rate, IPC paths)
   - Robot configurations (label, group, archetype, mesh, spawn position)
   - AI configurations (behavior type, parameters)
   - Lighting presets
3. Integrate TOML parser (Tommy or Tomlyn)
4. Create configuration validation
5. Support hot-reloading during development
6. Create example configuration files

**Acceptance Criteria:**
- [ ] Configuration loads from TOML files
- [ ] Schema is documented
- [ ] Invalid configurations produce clear errors
- [ ] Hot-reload works in Editor (optional in build)
- [ ] Example configs provided for common scenarios
- [ ] Format compatible with existing `robots.toml`

**Dependencies:** SIM-001, SIM-013, SIM-018

**Example Configuration:**
```toml
[simulation]
frame_rate = 30
ipc_image_path = "/dev/shm/auto_battlebot_frames"
ipc_command_path = "/dev/shm/auto_battlebot_commands"

[[robots]]
label = "MR_STABS_MK2"
group = "OURS"
archetype = "FourWheelVerticalSpinner"
mesh_path = "Models/MR STABS MK2/Mr Stabs Mk2 Chassis.fbx"
spawn_position = { x = 0, y = 0.1, z = -1.5 }

[[robots]]
label = "OPPONENT_1"
group = "THEIRS"
archetype = "TwoWheelHorizontalSpinner"
mesh_path = "Models/MRS BUFF MK2/Mrs Buff Mk2 Chassis.dae"
spawn_position = { x = 0, y = 0.1, z = 1.5 }

[robots.ai]
behavior = "Aggressive"
aggression = 0.8
reaction_time = 0.15
```

---

### SIM-025: Implement Robot Manager and Spawning System

**Sprint:** 5  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the RobotManager that orchestrates robot instantiation, lifecycle, and provides access to robot references for other systems.

**Tasks:**
1. Create `RobotManager` singleton
2. Implement robot spawning from configuration
3. Track controlled robot reference
4. Track opponent and neutral lists
5. Implement robot reset/respawn
6. Add robot removal/destruction handling
7. Expose robot queries (by label, by group)

**Acceptance Criteria:**
- [ ] Spawns all configured robots at start
- [ ] Controlled robot accessible via `GetControlledRobot()`
- [ ] Opponents accessible via `GetOpponents()`
- [ ] Respawn resets robots to spawn positions
- [ ] Can query robots by label
- [ ] Handles robot destruction gracefully

**Dependencies:** SIM-011, SIM-013, SIM-018, SIM-024

---

### SIM-026: End-to-End Integration Testing

**Sprint:** 5  
**Estimate:** 8 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Perform comprehensive integration testing to verify the complete pipeline from Unity rendering through C++ processing to motor command application.

**Tasks:**
1. Create integration test scene
2. Verify image quality matches expectations
3. Verify depth accuracy with known distances
4. Verify pose accuracy with known positions
5. Verify velocity command application
6. Measure and optimize end-to-end latency
7. Test with full C++ application stack
8. Document performance characteristics

**Acceptance Criteria:**
- [ ] C++ application receives valid camera data
- [ ] Depth values accurate within 5% at 1-3m range
- [ ] Pose values accurate within 1cm translation, 1° rotation
- [ ] Velocity commands applied within one physics step
- [ ] End-to-end latency <20ms at 30 FPS
- [ ] Full perception pipeline produces valid robot detections
- [ ] Navigation generates reasonable velocity commands

**Dependencies:** SIM-010, SIM-013, SIM-022, SIM-023, SIM-025

---

### SIM-027: Create Robot Asset Import Pipeline

**Sprint:** 5  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Create tooling and documentation for importing new robot CAD assets into the simulation with correct scale, orientation, and material setup.

**Tasks:**
1. Document CAD export requirements (format, units, orientation)
2. Create import settings preset for robot meshes
3. Create material assignment workflow
4. Build validation script to check imported meshes
5. Document complete workflow with screenshots
6. Test with at least one new robot import

**Acceptance Criteria:**
- [ ] Import workflow documented step-by-step
- [ ] Import settings preset available
- [ ] Validation script catches common issues
- [ ] New robot can be added in <30 minutes
- [ ] Existing models verified against workflow

**Dependencies:** SIM-014

---

### SIM-028: Implement Recording and Playback System

**Sprint:** 5  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Create a system to record simulation sessions (robot positions, commands, timing) for replay and regression testing.

**Tasks:**
1. Create `SessionRecorder` class
2. Define recording format (JSON or binary)
3. Record: timestamps, robot poses, velocity commands, events
4. Create `SessionPlayback` class
5. Implement playback with speed control
6. Add comparison mode (overlay two sessions)
7. Integrate with Unity Test Framework for assertions

**Acceptance Criteria:**
- [ ] Sessions can be recorded to file
- [ ] Sessions can be played back deterministically
- [ ] Playback speed is adjustable (0.5x - 4x)
- [ ] Recording file size reasonable (<10MB/minute)
- [ ] Can use recordings in automated tests
- [ ] Comparison mode shows divergence

**Dependencies:** SIM-025

---

### SIM-032: Implement Unity TCP Command Server for Data Generation

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create a TCP server in Unity that accepts commands from the Python orchestrator for controlling scene setup, robot positioning, and frame capture. This enables Python to drive the simulation without requiring the C++ application.

**Tasks:**
1. Create `DataGenerationServer` MonoBehaviour
2. Implement TCP listener on configurable port (default: 9999)
3. Define command protocol (JSON-based messages):
   - `SET_SCENE`: Configure arena and lighting
   - `SPAWN_ROBOT`: Add robot with specified properties
   - `SET_ROBOT_POSE`: Position and orient a robot
   - `SET_CAMERA_POSE`: Position the virtual camera
   - `CAPTURE_FRAME`: Render and return frame data
   - `GET_ANNOTATIONS`: Return ground truth for current frame
   - `RANDOMIZE`: Apply domain randomization with parameters
   - `RESET`: Clear scene to initial state
4. Implement message framing (length-prefixed)
5. Handle multiple sequential requests
6. Add timeout and error handling

**Acceptance Criteria:**
- [ ] Server accepts TCP connections on configurable port
- [ ] All defined commands implemented and tested
- [ ] Command protocol documented with examples
- [ ] Error responses include descriptive messages
- [ ] Server handles malformed requests gracefully
- [ ] Connection can be reused for multiple commands

**Dependencies:** SIM-001, SIM-006, SIM-025

**Protocol Example:**
```json
// Request
{"command": "SET_ROBOT_POSE", "robot_id": "OPPONENT_1", "position": [1.0, 0.1, 0.5], "rotation": [0, 45, 0]}

// Response
{"status": "ok", "timestamp": 1706123456.789}
```

---

### SIM-033: Implement Python Unity Client Library

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create a Python client library that communicates with the Unity TCP server, providing a clean API for controlling the simulation and retrieving frames with annotations.

**Tasks:**
1. Create `unity_client.py` module in `training/generator/`
2. Implement `UnitySimClient` class with connection management
3. Implement high-level methods:
   - `connect()` / `disconnect()`
   - `set_scene(config)`
   - `spawn_robot(robot_config)`
   - `set_robot_pose(robot_id, position, rotation)`
   - `set_camera_pose(position, rotation)`
   - `capture_frame()` → returns RGB image + depth + pose
   - `get_annotations()` → returns bounding boxes, masks, keypoints
   - `apply_randomization(params)`
   - `reset()`
4. Implement automatic reconnection on failure
5. Add type hints and docstrings
6. Create unit tests with mock server

**Acceptance Criteria:**
- [ ] Client connects to Unity server reliably
- [ ] All server commands have corresponding client methods
- [ ] Methods return typed dataclasses (not raw dicts)
- [ ] Connection failures raise clear exceptions
- [ ] Automatic retry on transient failures
- [ ] Unit tests achieve >80% coverage
- [ ] Works with project's existing virtual environment

**Dependencies:** SIM-032

**Usage Example:**
```python
from training.generator.unity_client import UnitySimClient

client = UnitySimClient(host="localhost", port=9999)
client.connect()

client.spawn_robot(RobotConfig(label="MR_STABS_MK2", group="OURS"))
client.set_robot_pose("MR_STABS_MK2", position=(0, 0.1, -1), rotation=(0, 0, 0))

frame = client.capture_frame()
annotations = client.get_annotations()

client.disconnect()
```

---

### SIM-034: Implement Ground Truth Annotation Exporter in Unity

**Sprint:** 6  
**Estimate:** 8 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the Unity-side annotation system that extracts ground truth labels from the scene, including 2D bounding boxes, segmentation masks, and keypoint locations.

**Tasks:**
1. Create `AnnotationExporter` class
2. Implement 2D bounding box calculation:
   - Project robot mesh bounds to screen space
   - Handle partial occlusion (visible bounds only)
   - Calculate tight-fitting boxes from mesh vertices
3. Implement instance segmentation mask generation:
   - Render object IDs to separate render target
   - Use replacement shader for ID rendering
   - Export as PNG with unique colors per instance
4. Implement keypoint annotation:
   - Define keypoint positions on robot archetypes
   - Project 3D keypoints to 2D image coordinates
   - Mark occluded keypoints as not visible
5. Include class labels from robot configuration
6. Output in structured format for Python consumption

**Acceptance Criteria:**
- [ ] Bounding boxes tightly fit visible robot portions
- [ ] Occluded robots have reduced/no bounding boxes
- [ ] Segmentation masks have pixel-perfect instance separation
- [ ] Keypoints correctly project to image coordinates
- [ ] Occluded keypoints marked with visibility flag
- [ ] Class labels match `classes.toml` definitions
- [ ] All annotation data serializable to JSON

**Dependencies:** SIM-006, SIM-011, SIM-025

**Annotation Output Structure:**
```json
{
  "frame_id": 1234,
  "image_width": 1280,
  "image_height": 720,
  "objects": [
    {
      "id": "MR_STABS_MK2",
      "class": "MR_STABS_MK2",
      "class_id": 1,
      "bbox": [x, y, width, height],
      "bbox_normalized": [x, y, w, h],
      "segmentation_color": [255, 0, 0],
      "keypoints": [
        {"name": "front_left_wheel", "x": 320, "y": 400, "visible": true},
        {"name": "weapon_center", "x": 340, "y": 380, "visible": true}
      ]
    }
  ]
}
```

---

### SIM-035: Implement Domain Randomization System

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create a domain randomization system that varies visual and geometric properties of the scene to improve model generalization from synthetic to real data.

**Tasks:**
1. Create `DomainRandomizer` component in Unity
2. Implement lighting randomization:
   - Intensity variation
   - Color temperature shifts
   - Shadow strength
   - Additional random light sources
3. Implement geometry randomization:
   - Robot spawn positions within arena
   - Robot orientations
   - Camera position/angle perturbations
4. Implement appearance randomization:
   - Material property variations (roughness, metallic)
   - Color tint variations per robot
   - Floor texture swapping
   - Dirt/scratch overlay effects
5. Implement camera effect randomization:
   - Exposure variation
   - Minor motion blur
   - Sensor noise injection
6. Create `RandomizationConfig` to control ranges
7. Support seeded randomization for reproducibility

**Acceptance Criteria:**
- [ ] Lighting varies within configured ranges
- [ ] Robot positions uniformly sampled in arena
- [ ] Material properties visibly vary between frames
- [ ] Camera effects applied without artifacts
- [ ] Randomization seed produces identical results
- [ ] All parameters configurable via TOML
- [ ] Visual diversity verified with sample grid image

**Dependencies:** SIM-022, SIM-023, SIM-025

**Configuration Example:**
```toml
[randomization.lighting]
intensity_range = [0.6, 1.4]
color_temp_range = [4500, 6000]
shadow_strength_range = [0.4, 1.0]

[randomization.geometry]
robot_position_noise = 0.5  # meters
robot_rotation_noise = 180  # degrees
camera_position_noise = 0.1  # meters

[randomization.appearance]
material_roughness_range = [0.2, 0.8]
color_variation = 0.1  # ±10% RGB
enable_dirt_overlay = true
dirt_intensity_range = [0.0, 0.3]

[randomization.camera]
exposure_range = [0.9, 1.1]
noise_intensity = 0.01
```

---

### SIM-036: Implement Annotation Writer (YOLO + COCO Formats)

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create Python module that converts Unity annotations to standard training formats (YOLO and COCO), enabling direct use with common object detection training pipelines.

**Tasks:**
1. Create `annotation_writer.py` module
2. Implement YOLO format writer:
   - One `.txt` file per image
   - Format: `class_id x_center y_center width height` (normalized)
   - Support keypoint extension: `class_id x y w h kp1_x kp1_y kp1_v ...`
3. Implement COCO format writer:
   - Single `annotations.json` for dataset
   - Include images, annotations, categories sections
   - Support bounding boxes and keypoints
   - Support segmentation polygons (from masks)
4. Implement image saving with consistent naming
5. Generate dataset split files (train/val/test)
6. Create `classes.txt` and `data.yaml` for YOLO training
7. Validate output against format specifications

**Acceptance Criteria:**
- [ ] YOLO format validates with `ultralytics` library
- [ ] COCO format validates with `pycocotools`
- [ ] Image filenames match annotation references
- [ ] Class IDs consistent with `classes.toml`
- [ ] Split ratios configurable (default 80/10/10)
- [ ] Output structure compatible with existing `training/` directory

**Dependencies:** SIM-033, SIM-034

**Output Structure:**
```
training/data/synthetic_dataset_YYYYMMDD/
├── images/
│   ├── train/
│   │   ├── 000001.png
│   │   └── ...
│   ├── val/
│   └── test/
├── labels/
│   ├── train/
│   │   ├── 000001.txt
│   │   └── ...
│   ├── val/
│   └── test/
├── annotations/
│   ├── train.json  (COCO format)
│   ├── val.json
│   └── test.json
├── classes.txt
└── data.yaml
```

---

### SIM-037: Implement Dataset Generation Orchestrator

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Create the main Python script that orchestrates large-scale synthetic dataset generation, coordinating scenario sampling, Unity control, and annotation writing.

**Tasks:**
1. Create `generate_dataset.py` main entry point
2. Implement command-line interface with argparse:
   - `--config`: Path to generation config TOML
   - `--output`: Output directory
   - `--num-frames`: Number of frames to generate
   - `--seed`: Random seed for reproducibility
   - `--split`: Train/val/test ratios
3. Implement scenario sampling:
   - Random robot counts and types
   - Random positions respecting constraints
   - Random lighting and appearance
4. Implement batch generation loop:
   - Setup scene via Unity client
   - Apply randomization
   - Capture frame and annotations
   - Write to disk
   - Progress reporting
5. Implement resumption from partial runs
6. Add generation statistics logging

**Acceptance Criteria:**
- [ ] CLI accepts all specified arguments
- [ ] Generates specified number of frames
- [ ] Seed produces reproducible datasets
- [ ] Progress reported to console
- [ ] Can resume interrupted generation
- [ ] Statistics logged (generation rate, errors)
- [ ] Exit code 0 on success, non-zero on failure

**Dependencies:** SIM-033, SIM-035, SIM-036

**Usage Example:**
```bash
cd training/generator
python generate_dataset.py \
    --config config/generation_config.toml \
    --output ../data/synthetic_v1 \
    --num-frames 10000 \
    --seed 42 \
    --split 0.8 0.1 0.1
```

---

### SIM-038: Create Scenario Configuration System

**Sprint:** 6  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** High  

**Description:**

Define the TOML configuration schema for synthetic data generation scenarios, controlling what types of scenes are generated and with what parameters.

**Tasks:**
1. Create `generation_config.toml` schema
2. Define sections:
   - `[unity]`: Connection settings
   - `[dataset]`: Output format options
   - `[scenarios]`: Scene composition rules
   - `[randomization]`: Domain randomization ranges
   - `[robots]`: Available robot types and weights
3. Implement configuration loader in Python
4. Implement configuration validation
5. Create example configurations for different use cases:
   - Basic detection training
   - Keypoint training
   - Specific robot focus
6. Document all configuration options

**Acceptance Criteria:**
- [ ] Schema covers all generation parameters
- [ ] Loader produces typed configuration objects
- [ ] Invalid configurations produce clear errors
- [ ] Example configs provided and documented
- [ ] Integration with generate_dataset.py

**Dependencies:** SIM-033

**Example Configuration:**
```toml
[unity]
host = "localhost"
port = 9999
timeout_seconds = 30

[dataset]
image_width = 1280
image_height = 720
formats = ["yolo", "coco"]
include_depth = false
include_segmentation = true
include_keypoints = true

[scenarios]
min_robots = 2
max_robots = 4
controlled_robot = "MR_STABS_MK2"
opponent_pool = ["MRS_BUFF_MK1", "MRS_BUFF_MK2", "OPPONENT"]
neutral_probability = 0.2
neutral_pool = ["WEDGE_BOT"]

[scenarios.camera]
height_range = [0.9, 1.1]
look_at_arena_center = true
position_noise = 0.05

[robots.MR_STABS_MK2]
archetype = "FourWheelVerticalSpinner"
mesh_path = "Models/MR STABS MK2/Mr Stabs Mk2 Chassis.fbx"
class_id = 1
keypoints = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel", "weapon_center"]

[robots.MRS_BUFF_MK1]
archetype = "TwoWheelHorizontalSpinner"
mesh_path = "Models/MRS BUFF MK1/Mrs. Buff Chassis Parts.dae"
class_id = 2
keypoints = ["left_wheel", "right_wheel", "weapon_center"]
```

---

### SIM-039: Validate Synthetic Data Pipeline End-to-End

**Sprint:** 6  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** Critical  

**Description:**

Perform comprehensive validation of the synthetic data generation pipeline, ensuring generated datasets work with the existing training infrastructure and produce models that perform on real data.

**Tasks:**
1. Generate test dataset (1000 frames)
2. Validate annotation accuracy:
   - Visual inspection of bounding box overlays
   - Check keypoint projections on sample images
   - Verify segmentation mask alignment
3. Test training pipeline integration:
   - Load dataset with YOLO training script
   - Run abbreviated training (few epochs)
   - Verify training completes without errors
4. Measure domain gap:
   - Train model on synthetic data
   - Evaluate on real validation set
   - Document performance delta
5. Benchmark generation performance:
   - Measure frames per minute
   - Identify bottlenecks
   - Document hardware requirements
6. Create validation report

**Acceptance Criteria:**
- [ ] 95%+ of bounding boxes visually correct
- [ ] Keypoints within 5 pixels of correct position
- [ ] YOLO training script loads dataset without modification
- [ ] Training runs complete without errors
- [ ] Performance metrics documented
- [ ] Generation achieves >100 frames/minute target
- [ ] Validation report written

**Dependencies:** SIM-034, SIM-036, SIM-037

---

### SIM-029: Implement Headless Mode for CI/CD

**Sprint:** 7  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Enable the simulation to run without a display for automated testing in CI/CD pipelines.

**Tasks:**
1. Configure build for headless/server mode
2. Ensure all rendering still occurs (off-screen)
3. Add command-line argument parsing
4. Implement exit conditions (time, events)
5. Create test script for CI integration
6. Document CI setup requirements

**Acceptance Criteria:**
- [ ] Simulation runs with `-batchmode -nographics` flag
- [ ] Camera capture still works in headless mode
- [ ] Can run on GitHub Actions Linux runner
- [ ] Exit code reflects test success/failure
- [ ] CI integration documented

**Dependencies:** SIM-026

---

### SIM-030: Performance Optimization Pass

**Sprint:** 7  
**Estimate:** 3 points  
**Type:** Task  
**Priority:** Medium  

**Description:**

Profile and optimize the simulation to meet performance targets on reference hardware.

**Tasks:**
1. Profile with Unity Profiler
2. Identify CPU bottlenecks
3. Identify GPU bottlenecks
4. Optimize IPC transfer (if needed)
5. Optimize camera capture (if needed)
6. Create quality presets (High, Medium, Low)
7. Document performance characteristics

**Acceptance Criteria:**
- [ ] 30+ FPS on reference hardware (GTX 1060 equivalent)
- [ ] <4GB memory usage
- [ ] IPC latency <10ms
- [ ] Quality presets provide options
- [ ] Performance documented for different hardware

**Dependencies:** SIM-026

---

### SIM-031: Documentation and Developer Guide

**Sprint:** 7  
**Estimate:** 5 points  
**Type:** Task  
**Priority:** High  

**Description:**

Create comprehensive documentation for developers working with the simulation system, including both the C++ integration mode and the Python synthetic data generation pipeline.

**Tasks:**
1. Create README with quick start guide
2. Document architecture overview
3. Document configuration options
4. Create "Adding a New Robot" guide
5. Create "Creating AI Behaviors" guide
6. Document IPC protocol specification (shared memory + TCP)
7. Create troubleshooting guide
8. Add inline code documentation
9. Document synthetic data generation pipeline:
   - Installation and setup
   - Configuration options
   - Running dataset generation
   - Output format specifications
10. Create "Training with Synthetic Data" guide
11. Document domain randomization parameters and tuning

**Acceptance Criteria:**
- [ ] README enables new developer to run simulation in <15 minutes
- [ ] All configuration options documented
- [ ] New robot guide tested by someone unfamiliar with system
- [ ] IPC protocol fully specified
- [ ] Code has XML documentation for public APIs
- [ ] Synthetic data generation documented with examples
- [ ] Data generation can be run by following docs alone

**Dependencies:** All previous tickets

---

## Appendix A: Technology Stack

| Component | Technology | Version | Rationale |
|-----------|------------|---------|-----------|
| Game Engine | Unity | 6 LTS | Modern rendering, C# scripting, physics |
| Render Pipeline | HDRP | Latest | Realistic lighting for minimal domain gap |
| IPC (Images) | Memory-mapped files | N/A | Low-latency large buffer transfer |
| IPC (Sync) | Unix domain sockets | N/A | Low-latency signaling |
| IPC (Data Gen) | TCP sockets | N/A | Cross-process Python↔Unity communication |
| Configuration | TOML | 1.0 | Consistent with C++ application |
| Testing | Unity Test Framework | Latest | Native Unity integration |
| Python | Python | 3.10+ | Match existing training environment |
| Data Gen Orchestration | Custom scripts | N/A | Flexible scenario generation |
| Annotation Format | YOLO + COCO | N/A | Industry standard training formats |
| Image Processing | OpenCV, Pillow | Latest | Image I/O and manipulation |
| COCO Tools | pycocotools | Latest | COCO format validation |

## Appendix B: Reference Hardware

**Development:**
- Intel i7 or equivalent
- 16GB RAM
- NVIDIA GTX 1060 or equivalent
- Ubuntu 22.04 or Windows 10/11

**CI/CD:**
- 4 vCPU
- 8GB RAM
- Software rendering (headless mode)
- Ubuntu 22.04

## Appendix C: Glossary

| Term | Definition |
|------|------------|
| NHRL | National Havoc Robot League - combat robot competition |
| ZED | Stereolabs depth camera product line |
| Visual SLAM | Simultaneous Localization and Mapping using camera images |
| IPC | Inter-Process Communication |
| HDRP | High Definition Render Pipeline (Unity) |
| Archetype | Template defining common robot configuration patterns |
| HIL | Hardware-in-the-Loop testing |
| Domain Randomization | Technique of varying simulation parameters to improve model generalization |
| Sim-to-Real Gap | Performance difference when applying simulation-trained models to real data |
| YOLO | You Only Look Once - popular object detection architecture and label format |
| COCO | Common Objects in Context - standard dataset format for object detection |
| Ground Truth | Correct annotations used for training/evaluation (vs. predictions) |
| Instance Segmentation | Per-pixel labeling that distinguishes individual object instances |
| Keypoint Detection | Locating specific semantic points on objects (e.g., wheel centers) |

---

*Document End*
