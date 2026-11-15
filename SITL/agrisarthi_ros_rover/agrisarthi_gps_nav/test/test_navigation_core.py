import unittest

from agrisarthi_gps_nav.navigation_core import NavigationController, validate_mission


def square_mission():
    return {
        "mission_id": "test-001",
        "boundary": [
            {"latitude": 0.0, "longitude": 0.0},
            {"latitude": 0.0, "longitude": 0.001},
            {"latitude": 0.001, "longitude": 0.001},
            {"latitude": 0.001, "longitude": 0.0},
        ],
        "waypoints": [{"id": 1, "latitude": 0.0005, "longitude": 0.0009}],
    }


class MissionValidationTests(unittest.TestCase):
    def test_accepts_in_bounds_wgs84_mission(self):
        mission = validate_mission(square_mission())
        self.assertEqual(mission["mission_id"], "test-001")
        self.assertEqual(len(mission["waypoints"]), 1)

    def test_accepts_geojson_closed_ring(self):
        mission = square_mission()
        mission["boundary"].append(dict(mission["boundary"][0]))
        self.assertEqual(len(validate_mission(mission)["boundary"]), 4)

    def test_rejects_missing_geofence(self):
        mission = square_mission()
        del mission["boundary"]
        with self.assertRaisesRegex(ValueError, "boundary"):
            validate_mission(mission)

    def test_rejects_waypoint_outside_geofence(self):
        mission = square_mission()
        mission["waypoints"][0]["longitude"] = 0.002
        with self.assertRaisesRegex(ValueError, "outside"):
            validate_mission(mission)

    def test_rejects_duplicate_waypoint_ids(self):
        mission = square_mission()
        mission["waypoints"].append({"id": 1, "latitude": 0.0004, "longitude": 0.0008})
        with self.assertRaisesRegex(ValueError, "duplicate waypoint ID"):
            validate_mission(mission)

    def test_rejects_home_point_outside_geofence(self):
        mission = square_mission()
        mission["home_point"] = {"latitude": 0.002, "longitude": 0.002}
        with self.assertRaisesRegex(ValueError, "home_point lies outside"):
            validate_mission(mission)

    def test_rejects_self_intersecting_geofence(self):
        mission = square_mission()
        mission["boundary"] = [
            {"latitude": 0.0, "longitude": 0.0},
            {"latitude": 0.001, "longitude": 0.001},
            {"latitude": 0.0, "longitude": 0.001},
            {"latitude": 0.001, "longitude": 0.0},
        ]
        with self.assertRaisesRegex(ValueError, "self-intersects"):
            validate_mission(mission)

    def test_rejects_touching_nonadjacent_geofence_edges(self):
        mission = square_mission()
        mission["boundary"] = [
            {"latitude": 0.0, "longitude": 0.0},
            {"latitude": 0.0, "longitude": 0.004},
            {"latitude": 0.002, "longitude": 0.004},
            {"latitude": 0.001, "longitude": 0.002},
            {"latitude": 0.002, "longitude": 0.0},
            {"latitude": 0.001, "longitude": 0.002},
        ]
        with self.assertRaisesRegex(ValueError, "self-intersects"):
            validate_mission(mission)


class NavigationSafetyTests(unittest.TestCase):
    def make_controller(self):
        controller = NavigationController(waypoint_tolerance_m=1.5)
        controller.load_mission(square_mission())
        return controller

    def test_positive_clockwise_error_turns_right(self):
        controller = self.make_controller()
        controller.set_gps(0.0005, 0.0005, 10.0)
        controller.set_heading(0.0, 10.0)
        self.assertTrue(controller.start())
        command = controller.step(10.1)
        self.assertGreater(command["left"], command["right"])
        self.assertGreater(command["angular"], 0.0)

    def test_arrival_threshold_completes_and_stops(self):
        controller = self.make_controller()
        controller.set_gps(0.0005, 0.0009, 10.0)
        controller.set_heading(90.0, 10.0)
        controller.start()
        command = controller.step(10.1)
        self.assertEqual(command["state"], "WAYPOINT_REACHED")
        self.assertFalse(command["enable"])
        self.assertEqual(controller.index, 0)
        self.assertFalse(controller.start())
        self.assertEqual(controller.complete_waypoint(1), "MISSION_COMPLETE")
        self.assertEqual(controller.index, 1)

    def test_waypoint_cannot_advance_without_matching_completion_ack(self):
        controller = self.make_controller()
        controller.set_gps(0.0005, 0.0009, 10.0)
        controller.set_heading(90.0, 10.0)
        controller.start()
        controller.step(10.1)
        with self.assertRaisesRegex(ValueError, "does not match"):
            controller.complete_waypoint(2)
        self.assertEqual(controller.index, 0)
        self.assertFalse(controller.step(10.2)["enable"])

    def test_waypoint_complete_state_persists_until_sampling_authority_restarts(self):
        mission = square_mission()
        mission["waypoints"].append({"id": 2, "latitude": 0.0005, "longitude": 0.0001})
        controller = NavigationController()
        controller.load_mission(mission)
        controller.set_gps(0.0005, 0.0009, 10.0)
        controller.set_heading(90.0, 10.0)
        self.assertTrue(controller.start())
        self.assertEqual(controller.step(10.1)["state"], "WAYPOINT_REACHED")
        self.assertEqual(controller.complete_waypoint(1), "WAYPOINT_COMPLETE")
        self.assertEqual(controller.step(10.2)["state"], "WAYPOINT_COMPLETE")
        self.assertEqual(controller.index, 1)

    def test_simulation_returns_home_only_after_final_sampling_ack(self):
        mission = square_mission()
        mission["home_point"] = {"latitude": 0.0005, "longitude": 0.0005}
        controller = NavigationController(return_home=True, waypoint_tolerance_m=1.0)
        controller.load_mission(mission)
        controller.set_gps(0.0005, 0.0009, 1.0)
        controller.set_heading(90.0, 1.0)
        self.assertTrue(controller.start())
        self.assertEqual(controller.step(1.1)["state"], "WAYPOINT_REACHED")
        self.assertEqual(controller.complete_waypoint(1), "RETURNING_HOME")
        self.assertTrue(controller.start())
        command = controller.step(1.2)
        self.assertTrue(command["enable"])
        self.assertEqual(controller.state, "NAVIGATING")
        controller.set_gps(0.0005, 0.0005, 1.3)
        controller.set_heading(270.0, 1.3)
        home = controller.step(1.4)
        self.assertEqual(home["state"], "MISSION_COMPLETE")
        self.assertFalse(home["enable"])

    def test_speed_reduces_near_waypoint_and_with_heading_error(self):
        far = self.make_controller()
        far.set_gps(0.0005, 0.0005, 10.0)
        far.set_heading(90.0, 10.0)
        far.start()
        far_speed = far.step(10.1)["linear"]
        near = self.make_controller()
        near.set_gps(0.0005, 0.00088, 10.0)
        near.set_heading(90.0, 10.0)
        near.start()
        near_speed = near.step(10.1)["linear"]
        self.assertGreater(far_speed, near_speed)

        misaligned = self.make_controller()
        misaligned.set_gps(0.0005, 0.0005, 10.0)
        misaligned.set_heading(70.0, 10.0)
        misaligned.start()
        self.assertGreater(far_speed, misaligned.step(10.1)["linear"])

    def test_gps_loss_stops_and_requires_explicit_restart(self):
        controller = self.make_controller()
        controller.set_gps(0.0005, 0.0005, 10.0)
        controller.set_heading(90.0, 10.0)
        controller.start()
        controller.set_gps(0.0, 0.0, 10.2, valid=False)
        stopped = controller.step(10.3)
        self.assertFalse(stopped["enable"])
        controller.set_gps(0.0005, 0.0005, 10.4)
        self.assertFalse(controller.step(10.5)["enable"])
        self.assertTrue(controller.start())
        self.assertTrue(controller.step(10.6)["enable"])

    def test_stale_heading_requires_explicit_restart(self):
        controller = self.make_controller()
        controller.set_gps(0.0005, 0.0005, 10.0)
        controller.set_heading(90.0, 10.0)
        controller.start()
        stopped = controller.step(11.1)
        self.assertFalse(stopped["enable"])
        controller.set_heading(90.0, 11.2)
        self.assertFalse(controller.step(11.3)["enable"])

    def test_rejected_replacement_clears_previous_mission(self):
        controller = self.make_controller()
        controller.start()
        invalid = square_mission()
        invalid["waypoints"][0]["longitude"] = 0.002
        with self.assertRaises(ValueError):
            controller.load_mission(invalid)
        command = controller.step(12.0)
        self.assertFalse(command["enable"])
        self.assertEqual(controller.points, [])
        self.assertEqual(controller.state, "MISSION_REJECTED")

    def test_gps_outside_fence_latches_stop(self):
        controller = self.make_controller()
        controller.set_gps(0.0015, 0.0005, 10.0)
        controller.set_heading(90.0, 10.0)
        controller.start()
        command = controller.step(10.1)
        self.assertEqual(command["state"], "GEOFENCE_BREACH")
        self.assertFalse(command["enable"])
        self.assertFalse(controller.start())


if __name__ == "__main__":
    unittest.main()
