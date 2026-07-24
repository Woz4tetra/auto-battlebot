# Latency report: auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_13-48-01_repaired

Context: live Jetson, mr_stabs_mk2, 720p30. parallel_models = false (sequential model calls), max_loop_rate = 30. TrtEngine already has per-engine CUDA streams. Recorded before the publish_camera_data guard/reorder. Sequential side of the first Jetson A/B; mcap footer repaired.

- Source: `/home/ben/Desktop/auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_13-48-01_repaired.mcap`
- Generated: 2026-07-24 14:34 by `scripts/mcap_latency_report.py`
- Duration: 60.1 s
- Window: after field init (19.4 s into the recording)
- Loop rate: 28.5 Hz mean
- End-to-end latency: mean 99.9 ms / p95 119.9 ms / max 452.3 ms
- Budget: 60 ms -> p95 is OVER BUDGET

| Stage | n | mean (ms) | median (ms) | p95 (ms) | max (ms) | % of tick |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| point_cloud_field_filter.compute_field | 1 | 163.17 | 163.17 | 163.17 | 163.17 | 473.5% |
| point_cloud_field_filter.find_minimum_rectangle | 1 | 112.84 | 112.84 | 112.84 | 112.84 | 327.4% |
| pipeline.latency | 1,690 | 99.95 | 103.06 | 119.95 | 452.26 | - |
| ros_publisher.publish_initial_field_description | 1 | 88.22 | 88.22 | 88.22 | 88.22 | 256.0% |
| runner.tick | 1,690 | 34.46 | 33.26 | 44.11 | 456.27 | 100.0% |
| ros_publisher.publish_field_mask | 1 | 27.77 | 27.77 | 27.77 | 27.77 | 80.6% |
| deeplab_mask_model.update | 1 | 27.23 | 27.23 | 27.23 | 27.23 | 79.0% |
| point_cloud_field_filter.fit_plane_ransac | 1 | 20.38 | 20.38 | 20.38 | 20.38 | 59.1% |
| runner.keypoint_model.update | 1,690 | 11.49 | 10.51 | 17.61 | 26.74 | 33.3% |
| yolo_keypoint_model.update | 1,690 | 11.48 | 10.50 | 17.61 | 26.73 | 33.3% |
| runner.robot_mask_model.update | 1,690 | 10.99 | 9.76 | 18.06 | 28.48 | 31.9% |
| yolo_bbox_robot_blob_model.update | 1,690 | 10.98 | 9.75 | 18.06 | 28.47 | 31.9% |
| runner.publishers | 1,690 | 10.09 | 9.54 | 13.69 | 22.64 | 29.3% |
| ros_publisher.publish_camera_data | 1,690 | 9.90 | 9.36 | 13.48 | 22.52 | 28.7% |
| point_cloud_field_filter.create_point_cloud_from_depth | 1 | 7.55 | 7.55 | 7.55 | 7.55 | 21.9% |
| point_cloud_field_filter.plane_center_from_inliers | 1 | 6.31 | 6.31 | 6.31 | 6.31 | 18.3% |
| point_cloud_field_filter.transform_points | 1 | 6.03 | 6.03 | 6.03 | 6.03 | 17.5% |
| point_cloud_field_filter.mask_depth_image | 1 | 2.80 | 2.80 | 2.80 | 2.80 | 8.1% |
| point_cloud_field_filter.find_largest_contour_mask | 1 | 2.65 | 2.65 | 2.65 | 2.65 | 7.7% |
| point_cloud_field_filter.point_cloud_to_2d | 1 | 2.28 | 2.28 | 2.28 | 2.28 | 6.6% |
| point_cloud_field_filter.extract_inliers | 1 | 1.93 | 1.93 | 1.93 | 1.93 | 5.6% |
| runner.camera.get | 1,690 | 1.07 | 0.01 | 5.99 | 115.26 | 3.1% |
| runner.robot_filter.update | 1,690 | 0.09 | 0.07 | 0.10 | 3.95 | 0.3% |
| ros_publisher.publish_robots | 1,690 | 0.08 | 0.05 | 0.14 | 4.86 | 0.2% |
| ros_publisher.publish_navigation | 1,690 | 0.04 | 0.03 | 0.06 | 1.14 | 0.1% |
| ros_publisher.publish_blob_detections | 1,690 | 0.03 | 0.02 | 0.05 | 3.87 | 0.1% |
| ros_publisher.publish_keypoint_detections | 1,690 | 0.03 | 0.01 | 0.04 | 4.21 | 0.1% |
| runner.field_filter.track_field | 1,690 | 0.02 | 0.01 | 0.03 | 4.55 | 0.1% |
| ros_publisher.publish_field_description | 1,690 | 0.01 | 0.01 | 0.01 | 1.01 | 0.0% |
| point_cloud_field_filter.get_rectangle_angle | 1 | 0.01 | 0.01 | 0.01 | 0.01 | 0.0% |
