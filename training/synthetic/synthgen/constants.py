"""Shared constants: category ids, class naming, and tuning thresholds.

Every magic number that gates an annotation or frame lives here so the keep/drop
policy is discoverable in one place. Values must not be changed casually: they
shape the training data distribution.
"""

ANNOTATION_MODE_KEYPOINTS_BBOX = "keypoints_bbox"
ANNOTATION_MODE_SEGMENTATION_BBOX = "segmentation_bbox"

# Category ids used for the BlenderProc segmentation pass in keypoint mode.
BACKGROUND_CATEGORY_ID = 0
ROBOT_CATEGORY_ID = 1
DISTRACTOR_CATEGORY_ID = 2

# Class ids used in segmentation mode. Per-robot classes are appended after
# SEG_ROBOT_CLASS_ID. This scheme is consumed downstream by
# training/yolo/remap_config_synthetic.toml and training/deeplab/remap_robots.toml.
SEG_FLOOR_CLASS_ID = 1
SEG_OBJECT_CLASS_ID = 2
SEG_ROBOT_CLASS_ID = 3

# Generic YOLO class + instance-id base for CAD (NHRL robot) distractors that
# carry keypoint sidecars; only used in keypoints_bbox annotation mode.
NHRL_ROBOT_CLASS_NAME = "nhrl_robot"
NHRL_DISTRACTOR_INSTANCE_ID_BASE = 1000

MODEL_EXTENSIONS = {".glb", ".gltf", ".obj", ".ply"}

# Minimum bbox edge (pixels) for a robot or distractor keypoint annotation.
MIN_KEYPOINT_BBOX_DIM_PX = 5

# A robot that fails its annotation gate but shows at least this many visible
# pixels is "prominent": leaving it unlabeled would create a false negative in
# the training data, so the frame is dropped instead.
PROMINENT_UNLABELED_ROBOT_MIN_PX = MIN_KEYPOINT_BBOX_DIM_PX**2

# Depth agreement (meters) between the projected keypoint and the rendered
# depth map for the keypoint to count as unoccluded.
KEYPOINT_DEPTH_TOLERANCE_M = 0.05

# Nearest-color fallback ceiling for robot part -> material matching when no
# configured color_mapping entry is within tolerance.
COLOR_MATCH_FALLBACK_MAX_DIST = 45.0

# Camera look-at sampling: noise resamples per geometry, geometry resamples
# before falling back to a deterministic centered target.
CAMERA_TARGET_RETRIES = 100
CAMERA_GEOMETRY_RESAMPLES = 5

# Scene retry budget: at most this multiple of the minimum scene count is
# attempted before the run stops (with a loud shortfall warning).
MAX_SCENE_ATTEMPTS_FACTOR = 3

# Abort the run after this many consecutive scenes produce no segmentation maps.
MAX_CONSECUTIVE_FAILED_SCENES = 25

# Parking position for distractors that are loaded but not active in a scene.
DISTRACTOR_OFFSCREEN_LOCATION = (1000.0, 1000.0, 1000.0)

# Progress log cadence (scenes).
PROGRESS_LOG_SCENE_INTERVAL = 50
