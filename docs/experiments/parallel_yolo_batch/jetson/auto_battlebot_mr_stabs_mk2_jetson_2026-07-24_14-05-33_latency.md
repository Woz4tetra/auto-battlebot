# Latency report: auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_14-05-33

Context: live Jetson, mr_stabs_mk2, 720p30. parallel_models = true, max_loop_rate = 30. Longer rerun of the parallel side of the first Jetson A/B, before the publish_camera_data guard/reorder.

- Source: `/home/ben/Desktop/auto_battlebot_mr_stabs_mk2_jetson_2026-07-24_14-05-33.mcap`
- Generated: 2026-07-24 14:34 by `scripts/mcap_latency_report.py`
- Duration: 121.5 s
- Window: after field init (4.3 s into the recording)
- Loop rate: 30.2 Hz mean
- End-to-end latency: mean 75.4 ms / p95 82.8 ms / max 281.6 ms
- Budget: 60 ms -> p95 is OVER BUDGET

| Stage | n | mean (ms) | median (ms) | p95 (ms) | max (ms) | % of tick |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| point_cloud_field_filter.compute_field | 1 | 99.50 | 99.50 | 99.50 | 99.50 | 316.5% |
| pipeline.latency | 3,639 | 75.39 | 74.25 | 82.75 | 281.56 | - |
| point_cloud_field_filter.find_minimum_rectangle | 1 | 65.26 | 65.26 | 65.26 | 65.26 | 207.6% |
| ros_publisher.publish_initial_field_description | 1 | 65.17 | 65.17 | 65.17 | 65.17 | 207.3% |
| runner.tick | 3,639 | 31.43 | 31.38 | 37.92 | 286.05 | 100.0% |
| deeplab_mask_model.update | 1 | 18.23 | 18.23 | 18.23 | 18.23 | 58.0% |
| ros_publisher.publish_field_mask | 1 | 16.70 | 16.70 | 16.70 | 16.70 | 53.1% |
| point_cloud_field_filter.fit_plane_ransac | 1 | 13.70 | 13.70 | 13.70 | 13.70 | 43.6% |
| runner.perception_batch.update | 3,639 | 12.98 | 12.49 | 16.15 | 45.25 | 41.3% |
| runner.keypoint_model.update | 3,639 | 12.29 | 12.03 | 14.92 | 34.99 | 39.1% |
| yolo_keypoint_model.update | 3,639 | 12.28 | 12.02 | 14.92 | 34.99 | 39.1% |
| runner.robot_mask_model.update | 3,639 | 11.96 | 11.71 | 14.52 | 41.59 | 38.1% |
| yolo_bbox_robot_blob_model.update | 3,639 | 11.96 | 11.70 | 14.51 | 41.59 | 38.0% |
| runner.publishers | 3,639 | 10.34 | 9.73 | 13.93 | 19.87 | 32.9% |
| ros_publisher.publish_camera_data | 3,639 | 10.16 | 9.56 | 13.75 | 19.61 | 32.3% |
| runner.camera.get | 3,639 | 7.44 | 7.85 | 12.49 | 59.09 | 23.7% |
| point_cloud_field_filter.transform_points | 1 | 6.89 | 6.89 | 6.89 | 6.89 | 21.9% |
| point_cloud_field_filter.create_point_cloud_from_depth | 1 | 5.90 | 5.90 | 5.90 | 5.90 | 18.8% |
| point_cloud_field_filter.point_cloud_to_2d | 1 | 2.41 | 2.41 | 2.41 | 2.41 | 7.7% |
| point_cloud_field_filter.extract_inliers | 1 | 1.45 | 1.45 | 1.45 | 1.45 | 4.6% |
| point_cloud_field_filter.find_largest_contour_mask | 1 | 1.43 | 1.43 | 1.43 | 1.43 | 4.6% |
| point_cloud_field_filter.mask_depth_image | 1 | 1.36 | 1.36 | 1.36 | 1.36 | 4.3% |
| point_cloud_field_filter.plane_center_from_inliers | 1 | 0.73 | 0.73 | 0.73 | 0.73 | 2.3% |
| runner.robot_filter.update | 3,639 | 0.09 | 0.08 | 0.11 | 4.19 | 0.3% |
| ros_publisher.publish_robots | 3,639 | 0.07 | 0.05 | 0.13 | 3.82 | 0.2% |
| ros_publisher.publish_navigation | 3,639 | 0.04 | 0.03 | 0.06 | 2.15 | 0.1% |
| ros_publisher.publish_keypoint_detections | 3,639 | 0.04 | 0.01 | 0.04 | 3.54 | 0.1% |
| ros_publisher.publish_blob_detections | 3,639 | 0.03 | 0.02 | 0.04 | 1.61 | 0.1% |
| runner.field_filter.track_field | 3,639 | 0.02 | 0.02 | 0.05 | 2.41 | 0.1% |
| ros_publisher.publish_field_description | 3,639 | 0.01 | 0.01 | 0.01 | 1.25 | 0.0% |
| point_cloud_field_filter.get_rectangle_angle | 1 | 0.01 | 0.01 | 0.01 | 0.01 | 0.0% |
