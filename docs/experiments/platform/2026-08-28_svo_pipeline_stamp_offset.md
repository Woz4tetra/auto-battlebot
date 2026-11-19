# SVO frame stamp vs pipeline stamp offset

Recorded 2026-08-28, from the notes in `scripts/combine_mcap_svo.py` next to `/camera/svo_frame`.

How well stamps alone identify a frame depends on when the recording was made.

- 2026-08-23 and 2026-08-28 Jetson recordings: the pipeline's stamps equal the SVO frame stamp exactly, 0.00 ms over 9521 frames, for both `/camera/frame_meta.image_stamp_ns` and `/camera/camera_info`. Stamp joins and index joins agree.
- 2026-05-02 NHRL recordings: the pipeline stamp lands 0.45 to 0.79 of the way through the SVO frame interval. Nearest-timestamp matching silently costs up to a frame there.

Join on the index, or on "pipeline stamp within `[svo_stamp, next_svo_stamp)`".
