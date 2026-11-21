# Homography field filter (RGB outline, no depth)

## Agreement with the depth path

On the two well-framed 2024-10-26 tripod recordings the homography path and the depth path agree to 5.0 cm and 5.2 cm in camera range, with plane tilt inside 0.53 degrees.

## Assumed mat size

The assumed size enters linearly: a mat that is 4% wrong makes every range 4% wrong. `field_size_x` and `field_size_y` are the floor mat, measured per venue, not the cage. Nominal 8 ft runs about 6% long against measured mats of 2.30 to 2.40 m.

## Outline guards

- Contour points close to the frame edge are dropped before the edge fit. They sit where the field crosses the image border, not where the field ends, and a line fitted through them is the border rather than the mat edge.
- Mask area over quad area above about 1.05 means the outline is not the quadrilateral the fit assumes, so at least one side is not a field edge. Reprojection error cannot see this.

## Rounded corners and coverage guard

- `cv::approxPolyDP` returns vertices on the outline, so at the mat's rounded corners it chords across them. The quad came out 19% small in area, 11% linearly, and put the camera 11% too far away. Details: `field_transform_homography_2026-09-08.md`.
- Before edge refinement the same chording inscribed the quad inside the mat, and the mask came out 24% larger than its own fit. With refinement the fitted quad circumscribes the mask.
- Coverage is measured against the quad clipped to the image. Unclipped, a field running off the frame has corners off the sensor, its quad is larger than anything the mask could fill, and the ratio reads as a bad outline on exactly the case this filter exists for.
