#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
FLOOR_DIR = TEST_DIR.parent
sys.path.insert(0, str(FLOOR_DIR))

from scrape_brettzone import discover_fight_links, parse_fight_identifiers  # noqa: E402


class TestScrapeParsing(unittest.TestCase):
    def test_discover_fight_links_from_html(self) -> None:
        html = """
        <a href="fightReviewSync.php?gameID=W-1&tournamentID=nhrl_mar26_3lb_">Fight</a>
        <a href="fightReviewSync.php?gameID=W-2&tournamentID=nhrl_mar26_3lb_">Fight2</a>
        """
        links = discover_fight_links("https://brettzone.nhrl.io/brettZone/", html)
        self.assertEqual(len(links), 2)
        self.assertTrue(links[0].startswith("https://brettzone.nhrl.io/brettZone/fightReviewSync.php"))

    def test_parse_fight_identifiers(self) -> None:
        ids = parse_fight_identifiers(
            "https://brettzone.nhrl.io/brettZone/fightReviewSync.php?gameID=W-12&tournamentID=nhrl_mar26_3lb_"
        )
        self.assertEqual(ids["game_id"], "W-12")
        self.assertEqual(ids["tournament_id"], "nhrl_mar26_3lb_")


if __name__ == "__main__":
    unittest.main()
