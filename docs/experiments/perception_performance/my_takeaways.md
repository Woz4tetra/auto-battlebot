# Don't use Meshy AI models for synthetic data if there's no real data to back it up

Meshy models work only if the output render is actually a close visual match to the real robot or there's real data
to reinforce it.
If there's no real data, it fails if the render is poor or if the builder modified the robot.
If I can collect massive amounts of keypoint data without labeling manually, this method will work for keypoints.
Since I have this for bounding box data, it did slightly improve results for bounding box detection.

docs/experiments/perception_performance/meshy_grade_2026-07-16.md
docs/experiments/perception_performance/synthetic_arms_2026-07-31.md

# Keep using CAD models for synthetic data

CAD based synthetic renders work because they don't have any visual hallucinations. I can also instruct not to change the design.
Or when there are external design changes, I can retrain from the changed CAD.

docs/experiments/perception_performance/meshy_grade_2026-07-16.md

# Don't combine bounding box and keypoints in one model

The keypoints model learns the specific robot appearance independent context.
The bounding box model is trying to generalize all NHRL robots. The synthetic data has random backgrounds and meshy AI models that don't look exactly like the real robots.
A bounding box model trained only on real footage ends up leaning on context.

This report attempts to probe whether the model is learning robot appearance or context:
docs/experiments/perception_performance/synthetic_arms_2026-07-31.md

Adding synthetic data does improve the model's performance slightly and cut-paste tests show the model can be less environment dependent
with synthetic data introduced, but combining with keypoints is still not the correct strategy.

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

# Should "our robots" be separate labels in the opponents model?

Mrs Buff MK2 was mistakenly mapped to Mrs Buff MK3. This actually strengthened detection for the bounding box and segmentation model
since MK2 is visually similar to MK3. This raises a question of whether it's worth putting our robots as separate categories for a model
that generalizes an NHRL robot in a cage. It also eliminates the risk of this model mislabeling an opponent as Mrs Buff MK3 which is a
worse side effect than labeling as opponent.

I think it's worth pursuing training an "any robot in the cage model" and separating "specific robots with CAD" to the keypoints model.
The house bot is an exception since there's plenty of real data of this robot.

It's now a question of whether downstream filtering can avoid getting confused.

docs/experiments/perception_performance/category_addition_2026-07-25.md

# When do I stop training YOLO? (what metric threshold do I need to satisfy baseline metrics requirements?)

~100 epochs seems to be enough. At this point, recall and precision are within the noise of baseline with unseeded runs.

docs/experiments/perception_performance/category_addition_2026-07-25.md

# What's a good number for mAP?

In my case, 0.5 is good. The objects I'm working with are small wrt the image size. So this score will never get super high.

# How many synthetic images do I need to generate for our keypoints model?

# For our keypoints model, how much real data do I need?

# How many images do I need to label for an individual robot?

# How many images do I need to label for a generalized NHRL robot?

Comparing to a baseline of ~26000 images, the floor is between 50% and 75%. The drop from 100% to 75% wasn't significant.
Based on context, this means the data I have doesn't have enough variation.
The data is drawn from 47 scenes. Randomly sampling was within the noise of 100%. 75% of scenes
saw a noticeable drop in performance. This tells me I need more scene variation.

What had a much bigger impact was data hygiene. Scrubbing through the dataset, I noticed lots of
errors. The conversion from segmentation to bounding box revealed there were lots of small polygons that broke the conversion.
Synthetic data leaked into earlier experiments. I thought this was a problem until I reran the experiment with clean bounding box and synthetic data.
Introducing synthetic data again didn't make recall worse but it didn't significantly improve the results either.
It improved precision but seeded reruns are needed to see if this is within the noise or not.

Going forward, validate all new images even ones that were validated in segmentation and go through
bounding box conversion. Consider the bounding box conversion a source of error.
Grab 25% fewer frames per scene. They extend training time and don't add much value.

docs/experiments/perception_performance/data_scaling_2026-07-27.md

# For the bounding box opponent model, how much synthetic data of our robots do I need to mix in?

docs/experiments/perception_performance/category_addition_2026-07-25.md

This experiment doesn't give a number but synthetic is definitely needed if Mrs Buff MK3 is to remain a category in the bounding box model.
The number of real images I've labeled for Mrs Buff MK3 is not sufficient.

# Is starting from a checkpoint better than cold start?

Yes, slightly. It saves ~25 epochs/45 min.

docs/experiments/perception_performance/category_addition_2026-07-25.md

# How many field images for deeplab do I need for each field type?

# When do I stop training deeplab?

# Does model size matter for my application? Is the latency trade off worth the improved performance?

Yes, size matters. On my dataset, s, m, and l perform similarly. The runs were seeded but it's possible retrainings
on different seeds will spread the results differently. x showed a significant improvement in recall.
s +0.059, m +0.057, l +0.057, x +0.088. At this point data variety matters more than data size.

Latency needs to be tested on the Jetson. s is an easy choice since it has similar latency, but x shows the best performance.
Looking at x run on massD playback data, it was far better at ignoring the arena logo. But if the latency exceeds the camera capture rate,
the runner could drop an image which costs ~30 ms.

docs/experiments/perception_performance/model_size_2026-09-04.md
