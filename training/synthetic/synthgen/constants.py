"""Shared constants: category ids, class naming, and tuning thresholds."""

ANNOTATION_MODE_KEYPOINTS_BBOX = "keypoints_bbox"
ANNOTATION_MODE_SEGMENTATION_BBOX = "segmentation_bbox"

BACKGROUND_CATEGORY_ID = 0
ROBOT_CATEGORY_ID = 1
DISTRACTOR_CATEGORY_ID = 2
HOUSE_BOT_CATEGORY_ID = 3

SEG_FLOOR_CLASS_ID = 1
SEG_OBJECT_CLASS_ID = 2
SEG_ROBOT_CLASS_ID = 3

NHRL_ROBOT_CLASS_NAME = "nhrl_robot"
HOUSE_BOT_CLASS_NAME = "house_bot"
NHRL_DISTRACTOR_INSTANCE_ID_BASE = 1000

MODEL_EXTENSIONS = {".glb", ".gltf", ".obj", ".ply"}

MIN_KEYPOINT_BBOX_DIM_PX = 5

PROMINENT_UNLABELED_ROBOT_MIN_PX = MIN_KEYPOINT_BBOX_DIM_PX**2

KEYPOINT_DEPTH_TOLERANCE_M = 0.05

COLOR_MATCH_FALLBACK_MAX_DIST = 45.0

CAMERA_TARGET_RETRIES = 100
CAMERA_GEOMETRY_RESAMPLES = 5

MAX_SCENE_ATTEMPTS_FACTOR = 3

MAX_CONSECUTIVE_FAILED_SCENES = 25

DISTRACTOR_OFFSCREEN_LOCATION = (1000.0, 1000.0, 1000.0)

PROGRESS_LOG_SCENE_INTERVAL = 50

ARENA_DENOISER = "OPTIX"

# Camera views a cage frame can be written in; see synthgen.lens. `pinhole` renders at the
# rectified matrix, `distorted` is the raw sensor frame, `rectified` is that frame put back
# through the C++ Rectifier's maps.
VIEW_PINHOLE = "pinhole"
VIEW_DISTORTED = "distorted"
VIEW_RECTIFIED = "rectified"
VIEWS: tuple[str, ...] = (VIEW_PINHOLE, VIEW_DISTORTED, VIEW_RECTIFIED)
ALPHAS: tuple[float, ...] = (1.0, 0.0)
