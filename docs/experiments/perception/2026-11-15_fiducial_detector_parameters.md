# 2026-11-15 fiducial detector parameters

## Small tag detector parameters

Detector tuned for small, distant 36h11 tags. The hard limit is pixels on target: face on, 36h11 decodes reliably only above about 18 px of edge and no parameter recovers a marker below about 15 px. Within that budget these settings buy margin. The adaptive-threshold window range is deliberately tight: windows above 21 px cost 33 ms per 1080p frame against 11 ms and added no detections once auto_gamma normalized brightness.

## Board detector parameters

The board is close and bright, and its white bleeds into each marker's black border. SUBPIX then lands every corner about 1.9 px inside the true square (markers detect 3.6% small), and the 65 mm board at 0.9 m reprojects at 2.5 px mean, 8 px max, against the 3 px gate. The AprilTag edge fit takes it to 0.55 px mean, 1.2 px max. OpenCV swaps in AprilTag's whole quad search for it, about 530 ms per 1920x1200 frame on the ZED Box against 56 ms, so it suits a detector that runs only for a field lock, not one that runs every frame.

## Reprojection residual

15 markers give 60 correspondences against 6 unknowns, so this residual measures something, unlike the four-corner homography's. It catches a mis-measured marker_size, a board printed at "fit to page" rather than 100%, a mirrored id mapping, and a board that was not flat.

## Board id order

Pairing the printed board's ids with `cv::aruco::GridBoard` in the wrong order (no reversal within each row) gave a mirrored pose with 440 px of reprojection error. The reversed ids give 16 px.

The manufactured board numbers markers right to left within a row, and `GridBoard` fills them left to right. The wrong pairing gives a reflection, and a reflection still decodes, so only the reprojection error shows it. Checked against a photo of the real board: with `first_id` 160 the top-left tag is 162 and the top-right is 160.

## Board placement and render accuracy

- The printed board has a white paper margin. Without one the perimeter markers' black borders sit straight on the grey floor, and the AprilTag quad search finds only the three interior markers. The synthetic render keeps a 0.02 m margin for the same reason.
- The board goes at a near corner. At the far corner a 65 mm marker is 8 px after foreshortening and does not decode.
- The test accepts 40 mm of range error at 2.4 m (1.7%). That is the synthetic render's limit, not the solver's: the board is rasterized, then perspective-warped down to markers about 30 px on a side, so their corners carry a fraction of a pixel of resampling bias that subpixel refinement cannot undo. The reprojection residual comes out under 1 px against the 3 px guard.
