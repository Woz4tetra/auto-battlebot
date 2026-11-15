# JPEG encoder selection

The JPEG encoder uses the Jetson's NVJPG engine when the build has it and it opens, then OpenCV. It logs which one it picked. There is no config field: where NVJPG exists it is always the right choice.
