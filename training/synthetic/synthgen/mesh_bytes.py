"""GPU memory estimates for meshes and textures."""

POSITION_BYTES = 12
NORMAL_BYTES = 12
UV_BYTES = 8
COLOR_BYTES = 4
TRIANGLE_INDEX_BYTES = 3 * 4
MIP_CHAIN_FACTOR = 4.0 / 3.0


def estimate_geometry_bytes(
    vertices: int, triangles: int, uv_bytes: int = UV_BYTES, color_bytes: int = COLOR_BYTES
) -> int:
    """Vertex attribute payload plus the triangle index buffer."""
    per_vertex = POSITION_BYTES + NORMAL_BYTES + uv_bytes + color_bytes
    return vertices * per_vertex + triangles * TRIANGLE_INDEX_BYTES


def estimate_mip_chain_bytes(base_level_bytes: int) -> int:
    """Base level size scaled to include a full mip chain."""
    return int(base_level_bytes * MIP_CHAIN_FACTOR)
