import unittest

from sitl_mission import make_sitl_four_waypoint_mission


class SitlMissionFixtureTests(unittest.TestCase):
    def test_creates_four_unique_waypoints_inside_home_boundary(self):
        mission = make_sitl_four_waypoint_mission("SITL_TEST", "SITL_ROVER")
        self.assertEqual(mission["waypoint_count"], 4)
        self.assertEqual([point["id"] for point in mission["waypoints"]], [1, 2, 3, 4])
        home = mission["home_point"]
        lats = [point["latitude"] for point in mission["boundary"]]
        lons = [point["longitude"] for point in mission["boundary"]]
        self.assertTrue(min(lats) <= home["latitude"] <= max(lats))
        self.assertTrue(min(lons) <= home["longitude"] <= max(lons))
        for point in mission["waypoints"]:
            self.assertTrue(min(lats) < point["latitude"] < max(lats))
            self.assertTrue(min(lons) < point["longitude"] < max(lons))


if __name__ == "__main__":
    unittest.main()
