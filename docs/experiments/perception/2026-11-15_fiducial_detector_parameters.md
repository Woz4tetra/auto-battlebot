# 2026-11-15 fiducial detector parameters

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/field_filter/fiducial_field_filter.cpp`

Beside `cv::aruco::DetectorParameters small_apriltag_detector_parameters() {`:

Detector tuned for small, distant 36h11 tags. The hard limit is pixels on target: face on, 36h11 decodes reliably only above about 18 px of edge and no parameter recovers a marker below about 15 px. Within that budget these settings buy margin. The adaptive-threshold window range is deliberately tight: windows above 21 px cost 33 ms per 1080p frame against 11 ms and added no detections once auto_gamma normalized brightness.

### `src/field_filter/fiducial_field_filter.cpp`

Beside `cv::aruco::DetectorParameters board_detector_parameters() {`:

The board is close and bright, and its white bleeds into each marker's black border. SUBPIX then lands every corner about 1.9 px inside the true square (markers detect 3.6% small), and the 65 mm board at 0.9 m reprojects at 2.5 px mean, 8 px max, against the 3 px gate. The AprilTag edge fit takes it to 0.55 px mean, 1.2 px max. OpenCV swaps in AprilTag's whole quad search for it, about 530 ms per 1920x1200 frame on the ZED Box against 56 ms, so it suits a detector that runs only for a field lock, not one that runs every frame.

### `src/field_filter/fiducial_field_filter.cpp`

Beside `std::vector<cv::Point2f> reprojected;`:

15 markers give 60 correspondences against 6 unknowns, so this residual measures something, unlike the four-corner homography's. It catches a mis-measured marker_size, a board printed at "fit to page" rather than 100%, a mirrored id mapping, and a board that was not flat.
