"""review_nhrl_thumbnails token parsing and the rejected-pool move."""

from pathlib import Path

import pytest
import review_nhrl_thumbnails as review


@pytest.mark.parametrize(
    "token",
    ["yeschef", "yeschef.png", "trash:///yeschef.png", "file:///path/yeschef.png", " yeschef\n"],
)
def test_token_resolves_to_clean_name(token: str) -> None:
    assert review.clean_name_from_token(token) == "yeschef"


def test_rejecting_moves_png_out_of_the_pool(tmp_path: Path) -> None:
    thumbs = tmp_path / "thumbs"
    rejected = tmp_path / "rejected"
    thumbs.mkdir()
    (thumbs / "yeschef.png").write_bytes(b"png")
    review.set_rejected({}, True, thumb_dir=thumbs, rejected_dir=rejected, clean_name="yeschef")
    assert not list(thumbs.glob("*.png"))
    assert (rejected / "yeschef.png").exists()
