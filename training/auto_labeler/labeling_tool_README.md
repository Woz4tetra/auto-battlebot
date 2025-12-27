# Video Labeling Tool

An interactive client-server application for video object segmentation labeling using SAM3 (Segment Anything Model 3).

## Architecture

- **Server**: Runs on a headless machine with GPU(s). Handles video processing, SAM3 inference, and annotation storage.
- **Client**: Runs on a machine with a display. Provides interactive UI for labeling.

## Features

- Click to add positive/negative points for object segmentation
- Multiple object labels with distinct colors
- Mask propagation through video using SAM3
- Undo, clear, and delete point operations
- Navigate between frames (slider, keyboard shortcuts)
- Jump to manually labeled frames or next unlabeled frame
- Preview masks before propagation
- Persistent state - resume labeling where you left off
- COCO format annotation export

## Installation

### Server (GPU machine)

```bash
# Install dependencies
pip install flask flask-cors opencv-python numpy torch pyyaml tqdm pycocotools

# Install SAM3 (follow SAM3 installation instructions)
pip install sam3
```

### Client (display machine)

```bash
# Install dependencies
pip install requests pillow
```

Note: The client uses Tkinter which is included with Python on most systems.

## Usage

### 1. Configure the server

Edit `config/labeling_config.yaml`:

```yaml
# Path to your video file
video_path: "/path/to/your/video.mp4"

# Define object labels to track
object_labels:
  - id: 1
    name: "robot_blue"
    color: [0, 120, 255]
  - id: 2
    name: "robot_red"
    color: [255, 50, 50]

# Output directories
manual_annotations_dir: "./output/manual_annotations"
generated_annotations_dir: "./output/generated_annotations"

# Propagation settings
propagate_length: 30  # frames to propagate forward

# Server settings
host: "0.0.0.0"
port: 8765
gpu_id: 0
```

### 2. Start the server

On your GPU machine:

```bash
cd training/auto_labeler
python run_server.py --config config/labeling_config.yaml
```

### 3. Start the client

On your display machine:

```bash
cd training/auto_labeler
python run_client.py --server http://<server-ip>:8765
```

## Client UI

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Left/Right | Previous/next frame |
| Shift+Left/Right | Jump 10 frames |
| Home/End | First/last frame |
| Ctrl+Z | Undo last point |
| Space | Preview mask |
| Enter | Propagate masks |
| Escape | Toggle add/remove mode |
| 1-9 | Select object label |

### Workflow

1. **Select an object label** from the sidebar
2. **Click on the image** to add points:
   - Green (+) points include the area
   - Red (-) points exclude the area
3. **Preview** to see the predicted mask
4. **Propagate** to extend masks through the video
5. **Navigate** through frames to verify results
6. Add more points if needed and re-propagate

### Point Modes

- **Add (Include)**: Points tell SAM3 "this is part of the object"
- **Remove (Exclude)**: Points tell SAM3 "this is NOT part of the object"

Use exclude points to refine masks by marking areas that shouldn't be included.

## Output

### Manual Annotations

Saved to `manual_annotations_dir`:
```
manual_annotations/
├── annotations/
│   └── session_state.json  # Point prompts and session state
└── images/
    └── frame_000000.jpg    # Reference frames
```

### Generated Annotations

Saved to `generated_annotations_dir`:
```
generated_annotations/
├── annotations/
│   └── annotations.json    # COCO format annotations
├── images/
│   └── frame_000000.jpg    # Labeled frames
└── masks/
    └── frame_000000_obj_1.png  # Binary masks
```

## API Reference

### Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/config` | GET | Get server configuration |
| `/api/status` | GET | Get annotation status |
| `/api/frame/<idx>` | GET | Get frame image |
| `/api/frame/<idx>/info` | GET | Get frame information |
| `/api/points` | POST | Add a point |
| `/api/points/undo` | POST | Undo last point |
| `/api/points/clear` | POST | Clear points |
| `/api/points/delete` | POST | Delete specific point |
| `/api/preview` | POST | Get mask preview |
| `/api/propagate` | POST | Propagate masks |
| `/api/navigation/next-unlabeled` | GET | Get next unlabeled frame |
| `/api/navigation/manual-frames` | GET | Get manual frame list |
| `/api/navigation/prev-manual` | GET | Get previous manual frame |
| `/api/navigation/next-manual` | GET | Get next manual frame |
| `/api/save` | POST | Force save state |
| `/api/export/coco` | GET | Export COCO annotations |

## Resuming Work

The server automatically saves state when:
- Points are added/removed
- Masks are propagated
- The server shuts down

When you restart the server, it loads the previous state from `session_state.json`.
