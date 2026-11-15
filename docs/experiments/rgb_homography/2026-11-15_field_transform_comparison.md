# Field transform comparison

## Field transform comparison

### `playground/field_transform/compare_field_transform.py`

`decode_organized_cloud`:

An organized XYZ+BGRA PointCloud2 back into a BGR image and a metric depth image.

The 2024 competition bags never recorded the ZED image or depth streams; the only per-pixel data is this cloud, published once when the field was initialised. It is in the optical frame (verified: unprojecting z through K reproduces the cloud's own x and y to 0.0000 m), so z is exactly the depth image the C++ path expects.
