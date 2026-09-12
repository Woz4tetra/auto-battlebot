"""Python mirrors of the C++ perception path, used to score and compare engines.

trt_yolo            TensorRT YOLO inference matching YoloKeypointModel
detection_viz       box / keypoint overlays and class-name loading for the engine CLIs
camera_geometry     pixel-to-floor projection matching project_keypoint_onto_plane
camera_calibration  config/cameras/<id>.toml loader and the C++ Rectifier's undistort maps
field_pose          camera-from-field fits: depth-plane port and the RGB corner homography
cage_calibration    config/cages/<id>.toml format and the field / world / Blender frame chain
"""
