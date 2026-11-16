# 2026-11-15 field outline fit

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/field_filter/field_outline.cpp`

Beside `bool seed_quad(const cv::Mat &binary, std::array<cv::Point2d, 4> &out) {`:

Four seed corners from the convex hull, loosening approxPolyDP until exactly four survive. Under perspective a rectangle projects to a general quadrilateral, not a rotated rectangle, so minAreaRect is wrong here: it returns the bounding box of the quad and the error grows with camera tilt.

### `src/field_filter/field_outline.cpp`

Beside `std::array<std::vector<cv::Point2d>, 4> owned;`:

Assign each boundary point to whichever side it is nearest, then keep the straight middle stretch. approxPolyDP can only return vertices lying on the outline, so where the mat has rounded corners its quad chords across them and comes out short; fitting the middle of each side and intersecting extrapolates back to the real corners.
