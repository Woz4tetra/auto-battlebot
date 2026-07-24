# Latency report: auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_13-50-38_repaired

- Source: `/home/ben/Desktop/auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_13-50-38_repaired.mcap`
- Generated: 2026-07-24 14:34 by `scripts/mcap_latency_report.py`
- Duration: 73.0 s
- Window: after field init (4.0 s into the recording)
- Loop rate: 30.2 Hz mean
- End-to-end latency: mean 76.2 ms / p95 86.8 ms / max 317.9 ms
- Budget: 60 ms -> p95 is OVER BUDGET

| Stage | n | mean (ms) | median (ms) | p95 (ms) | max (ms) | % of tick |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| point_cloud_field_filter.compute_field | 1 | 121.43 | 121.43 | 121.43 | 121.43 | 386.8% |
| point_cloud_field_filter.find_minimum_rectangle | 1 | 81.24 | 81.24 | 81.24 | 81.24 | 258.8% |
| pipeline.latency | 2,189 | 76.19 | 74.80 | 86.76 | 317.91 | - |
| ros_publisher.publish_initial_field_description | 1 | 70.24 | 70.24 | 70.24 | 70.24 | 223.7% |
| runner.tick | 2,189 | 31.40 | 31.19 | 37.98 | 322.02 | 100.0% |
| deeplab_mask_model.update | 1 | 21.38 | 21.38 | 21.38 | 21.38 | 68.1% |
| ros_publisher.publish_field_mask | 1 | 17.13 | 17.13 | 17.13 | 17.13 | 54.6% |
| point_cloud_field_filter.fit_plane_ransac | 1 | 13.98 | 13.98 | 13.98 | 13.98 | 44.5% |
| runner.perception_batch.update | 2,189 | 13.37 | 12.73 | 18.30 | 36.98 | 42.6% |
| runner.keypoint_model.update | 2,189 | 12.61 | 12.27 | 16.22 | 36.94 | 40.2% |
| yolo_keypoint_model.update | 2,189 | 12.60 | 12.26 | 16.21 | 36.93 | 40.1% |
| runner.robot_mask_model.update | 2,189 | 12.27 | 11.94 | 15.75 | 34.74 | 39.1% |
| yolo_bbox_robot_blob_model.update | 2,189 | 12.26 | 11.93 | 15.74 | 34.73 | 39.1% |
| point_cloud_field_filter.create_point_cloud_from_depth | 1 | 12.10 | 12.10 | 12.10 | 12.10 | 38.6% |
| runner.publishers | 2,189 | 10.12 | 9.56 | 13.74 | 16.67 | 32.2% |
| ros_publisher.publish_camera_data | 2,189 | 9.95 | 9.39 | 13.57 | 16.55 | 31.7% |
| runner.camera.get | 2,189 | 7.16 | 7.46 | 12.47 | 57.58 | 22.8% |
| point_cloud_field_filter.transform_points | 1 | 5.44 | 5.44 | 5.44 | 5.44 | 17.3% |
| point_cloud_field_filter.point_cloud_to_2d | 1 | 2.85 | 2.85 | 2.85 | 2.85 | 9.1% |
| point_cloud_field_filter.find_largest_contour_mask | 1 | 1.83 | 1.83 | 1.83 | 1.83 | 5.8% |
| point_cloud_field_filter.extract_inliers | 1 | 1.49 | 1.49 | 1.49 | 1.49 | 4.7% |
| point_cloud_field_filter.mask_depth_image | 1 | 1.38 | 1.38 | 1.38 | 1.38 | 4.4% |
| point_cloud_field_filter.plane_center_from_inliers | 1 | 0.73 | 0.73 | 0.73 | 0.73 | 2.3% |
| runner.robot_filter.update | 2,189 | 0.10 | 0.09 | 0.12 | 5.50 | 0.3% |
| ros_publisher.publish_robots | 2,189 | 0.07 | 0.05 | 0.13 | 3.25 | 0.2% |
| ros_publisher.publish_navigation | 2,189 | 0.04 | 0.03 | 0.06 | 1.66 | 0.1% |
| ros_publisher.publish_keypoint_detections | 2,189 | 0.04 | 0.01 | 0.04 | 2.96 | 0.1% |
| ros_publisher.publish_blob_detections | 2,189 | 0.03 | 0.02 | 0.04 | 2.20 | 0.1% |
| runner.field_filter.track_field | 2,189 | 0.02 | 0.02 | 0.05 | 0.19 | 0.1% |
| ros_publisher.publish_field_description | 2,189 | 0.01 | 0.01 | 0.01 | 0.06 | 0.0% |
| point_cloud_field_filter.get_rectangle_angle | 1 | 0.01 | 0.01 | 0.01 | 0.01 | 0.0% |
