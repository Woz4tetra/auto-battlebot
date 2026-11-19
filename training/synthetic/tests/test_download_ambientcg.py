"""download_ambientcg.py: asset names parsed from download URLs."""

import download_ambientcg


def test_parse_asset_from_url() -> None:
    url = "https://ambientcg.com/get?file=Plastic007_2K-JPG.zip"
    assert download_ambientcg.parse_asset_from_url(url) == "Plastic007"
