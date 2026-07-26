# Don't use Meshy AI models for synthetic data

Meshy models work only if the output render is actually a close visual match to the real robot.
It fails if the render is poor or if the builder modified the robot.
For this reason, this method will not work for my application.

docs/experiments/perception_performance/meshy_grade_2026-07-16.md

# Keep using CAD models for synthetic data

CAD based synthetic renders work because they don't have any visual hallucinations. I can also instruct not to change the design.
Or when there are external design changes, I can retrain from the changed CAD.

docs/experiments/perception_performance/meshy_grade_2026-07-16.md

# Don't combine bounding box and keypoints in one model

The keypoints model learns the specific robot appearance independent context.
The bounding box model is trying to generalize all NHRL robots. The synthetic data has random backgrounds and meshy AI models that don't look exactly like the real robots.
The model needs to learn the context NHRL robots exist in.

This report attempts to probe whether the model is learning robot appearance or context: docs/experiments/perception_performance/synthetic_plus_bbox_2026-07-22.md
The answer is, a bit of both. But I can't rely on one or the other.

# Don't split the opponent category by archetype

Similar reason to the above section. The model is learning the context the robots exist in and not really solely on robot appears.
All robots look pretty different even within the same archetype so the model has a hard time learning the split.
Also each archetype has wildly different amounts of data so some classes pollute the recall.

docs/experiments/perception_performance/nhrl_robots_7class_2026-07-13.md

# Don't split the opponent category by individual robot

The class imbalance for each robot name causes all metrics to tank. "Softmax dilutes confidence".
The model also fails to generalize to new robots since they don't match the appearance of any other robot.

docs/experiments/perception_performance/indiv_blob_2026-07-09.md

# Use bounding box model not segmentation

For bounding box metrics, the two models are nearly identical.
Segmentation doesn't improve localization much since I use the centroid for location.
On desktop, the bounding box is 18% faster than the segmentation model.

docs/experiments/perception_performance/seg_vs_bbox_2026-07-18.md

# When do I stop training YOLO? (what metric threshold do I need to satisfy baseline metrics requirements?)

# How many synthetic images do I need to generate for our keypoints model?

# For our keypoints model, how much real data do I need?

# How many real images do I need to label?

# For the bounding box opponent model, how much synthetic data of our robots do I need to mix in?

# Is starting from a checkpoint better than cold start?

# How many field images for deeplab do I need for each field type?

# When do I stop training deeplab?
