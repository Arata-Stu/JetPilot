from __future__ import annotations

import unittest

from object_detection_learning.contract import classes_from_dataset


class DatasetClassContractTests(unittest.TestCase):
    def test_accepts_list_and_contiguous_mapping(self) -> None:
        self.assertEqual(
            classes_from_dataset({"names": ["vehicle", "barrier"]}),
            ["vehicle", "barrier"],
        )
        self.assertEqual(
            classes_from_dataset({"names": {1: "barrier", 0: "vehicle"}}),
            ["vehicle", "barrier"],
        )

    def test_rejects_non_contiguous_mapping(self) -> None:
        with self.assertRaises(ValueError):
            classes_from_dataset({"names": {0: "vehicle", 2: "barrier"}})


if __name__ == "__main__":
    unittest.main()
