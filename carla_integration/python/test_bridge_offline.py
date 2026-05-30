"""
Offline test for VDSim ↔ CARLA bridge — does NOT require a CARLA server.

Exercises:
    - module imports (vdsim, carla)
    - coordinate frame conversions
    - bridge construction with a mocked CARLA actor / world

Run:  python3 carla_integration/python/test_bridge_offline.py
"""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "carla_integration" / "python"))

import carla   # noqa: E402
from vdsim_carla_bridge import (   # noqa: E402
    BridgeConfig, VDSimCarlaBridge,
    vdsim_pose_to_carla, carla_velocity_from_vdsim_body,
    wheel_offsets_body,
)


class FrameConversionTests(unittest.TestCase):
    def test_y_axis_flip(self):
        t = vdsim_pose_to_carla(1.0, 2.0, 3.0, yaw_rad=0.0)
        self.assertAlmostEqual(t.location.x, 1.0)
        self.assertAlmostEqual(t.location.y, -2.0)   # +y_vdsim → -y_carla
        self.assertAlmostEqual(t.location.z, 3.0)

    def test_yaw_sign_flip(self):
        t = vdsim_pose_to_carla(0, 0, 0, yaw_rad=math.pi / 2)  # 90° CCW (ISO)
        self.assertAlmostEqual(t.rotation.yaw, -90.0)          # CARLA CW

    def test_velocity_world_rotation(self):
        # body vx=10, vy=0, yaw=0 → world (10, 0, 0)
        v = carla_velocity_from_vdsim_body(10.0, 0.0, 0.0)
        self.assertAlmostEqual(v.x, 10.0)
        self.assertAlmostEqual(v.y, 0.0)
        # body vx=10, vy=0, yaw=π/2 → world (0, -10) (after y flip)
        v = carla_velocity_from_vdsim_body(10.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(v.x, 0.0, places=6)
        self.assertAlmostEqual(v.y, -10.0, places=6)


class WheelOffsetTests(unittest.TestCase):
    def test_offsets_match_vehicle_params(self):
        # Use defaults from VehicleParams via a YAML
        import vdsim
        vp = vdsim.VehicleParams.from_yaml(
            str(REPO / "configs/vehicles/sedan.yaml"))
        off = wheel_offsets_body(vp)
        a, b = vp.cg_to_front, vp.cg_to_rear
        tw_f, tw_r = vp.track_front / 2, vp.track_rear / 2
        self.assertEqual(off, [(+a, +tw_f), (+a, -tw_f),
                                (-b, +tw_r), (-b, -tw_r)])


class BridgeConstructionMocked(unittest.TestCase):
    """Construct the bridge against a mocked CARLA world / actor."""

    def _mock_client(self):
        client = MagicMock(spec=carla.Client)
        world = MagicMock()
        world.get_settings.return_value = MagicMock(
            synchronous_mode=False, fixed_delta_seconds=0.0)
        world.apply_settings = MagicMock()

        # blueprint library
        bp_lib = MagicMock()
        bp = MagicMock()
        bp.set_attribute = MagicMock()
        bp_lib.find = MagicMock(return_value=bp)
        world.get_blueprint_library.return_value = bp_lib

        # spawn points
        spawn = carla.Transform(
            carla.Location(x=100.0, y=50.0, z=2.0),
            carla.Rotation(yaw=0.0, pitch=0.0, roll=0.0))
        world.get_map.return_value.get_spawn_points.return_value = [spawn]

        # actor
        actor = MagicMock()
        actor.is_alive = True
        actor.set_simulate_physics = MagicMock()
        actor.set_transform = MagicMock()
        actor.set_target_velocity = MagicMock()
        actor.set_target_angular_velocity = MagicMock()
        actor.get_transform = MagicMock(return_value=spawn)
        world.try_spawn_actor = MagicMock(return_value=actor)
        world.cast_ray = MagicMock(return_value=[
            MagicMock(location=carla.Location(x=100, y=50, z=0))])
        world.tick = MagicMock()

        client.get_world = MagicMock(return_value=world)
        return client, world, actor

    def test_constructs_and_steps(self):
        cfg = BridgeConfig(
            vehicle_yaml=str(REPO / "configs/vehicles/sedan.yaml"),
            tire_yaml=str(REPO / "configs/tires/default_pacejka.yaml"),
            level="L2", sync_mode=False, fixed_delta_seconds=0.02,
            inner_dt=0.005,
        )
        client, world, actor = self._mock_client()
        bridge = VDSimCarlaBridge(cfg, client)
        self.assertIsNotNone(bridge.actor)
        self.assertEqual(str(bridge.level_enum), "Level.L2_SevenDOF")
        # query_contacts uses cast_ray
        contacts = bridge.query_contacts()
        self.assertEqual(len(contacts), 4)
        for cp in contacts:
            self.assertTrue(cp.is_valid)
        # step
        bridge.step(throttle=0.1, brake=0.0, steer=0.05)
        actor.set_transform.assert_called()
        # telemetry
        tele = bridge.telemetry()
        self.assertIn("vx", tele)
        self.assertEqual(len(tele["Fz"]), 4)
        bridge.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
