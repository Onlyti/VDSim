"""
Record VDSim+CARLA demo with a spectator-style RGB camera attached to the ego.
Saves PNG frames + writes a CSV of telemetry.

Usage:
    python3 carla_integration/python/record_demo.py --duration 20
"""
import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import carla

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "carla_integration" / "python"))

from vdsim_carla_bridge import BridgeConfig, VDSimCarlaBridge   # noqa: E402
from run_demo import make_figure_eight, pure_pursuit_steer       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--vehicle", default=str(REPO / "configs/vehicles/sports.yaml"))
    ap.add_argument("--tire",    default=str(REPO / "configs/tires/default_pacejka.yaml"))
    ap.add_argument("--level",   default="L2")
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--v_target", type=float, default=10.0)
    ap.add_argument("--outdir", default=str(REPO / "carla_integration/results/run01"))
    ap.add_argument("--frame_every", type=int, default=5,
                    help="save every Nth tick (default 5 = 10 Hz at 50 Hz tick)")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    (outdir / "frames").mkdir(parents=True, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    cfg = BridgeConfig(
        vehicle_yaml=args.vehicle, tire_yaml=args.tire, level=args.level,
        sync_mode=True, fixed_delta_seconds=0.02, inner_dt=0.005,
        initial_vx=args.v_target,
    )

    # Cleanup any prior ego
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == 'vdsim_ego':
            a.destroy()

    bridge = VDSimCarlaBridge(cfg, client)
    world.tick()
    spawn = bridge.actor.get_transform()
    print(f"[record] spawned ego at {spawn.location}")

    # Attach RGB camera (third-person view)
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', '960')
    bp.set_attribute('image_size_y', '540')
    bp.set_attribute('fov', '90')
    cam_tf = carla.Transform(
        carla.Location(x=-7.0, z=4.0),
        carla.Rotation(pitch=-15.0))
    camera = world.spawn_actor(bp, cam_tf, attach_to=bridge.actor)

    saved_frames = []
    def on_image(image):
        if image.frame % args.frame_every != 0:
            return
        path = outdir / "frames" / f"frame_{image.frame:06d}.png"
        image.save_to_disk(str(path))
        saved_frames.append((image.frame, str(path)))
    camera.listen(on_image)

    path = make_figure_eight(spawn)
    prev_idx = 0
    csv_rows = []
    dt = cfg.fixed_delta_seconds
    n_steps = int(args.duration / dt)
    t_start = time.time()
    sim_t = 0.0
    try:
        for step in range(n_steps):
            tele = bridge.telemetry()
            vx = tele["vx"]
            loc = bridge.actor.get_transform().location
            rot = bridge.actor.get_transform().rotation
            yaw = math.radians(-rot.yaw)
            steer, prev_idx = pure_pursuit_steer(
                loc.x, -loc.y, yaw, vx, bridge.vp.wheelbase, path, prev_idx)
            e = args.v_target - vx
            throttle = max(0.0, min(1.0, 0.10 + 0.30 * e))
            brake = 0.0 if e > -1 else 0.2

            bridge.step(throttle, brake, steer)
            sim_t += dt

            csv_rows.append({
                't': round(sim_t, 4),
                'x_world': loc.x, 'y_world': loc.y,
                'yaw_deg_carla': rot.yaw,
                'vx': vx, 'vy': tele['vy'],
                'yaw_rate': tele['yaw_rate'],
                'ax': tele['ax'], 'ay': tele['ay'],
                'steer': steer,
                'throttle': throttle, 'brake': brake,
                'Fz_FL': tele['Fz'][0], 'Fz_FR': tele['Fz'][1],
                'Fz_RL': tele['Fz'][2], 'Fz_RR': tele['Fz'][3],
            })
            if step % int(1.0 / dt) == 0:
                print(f"  t={sim_t:6.2f}  vx={vx:5.2f}  steer={steer:+.3f}  "
                      f"r={tele['yaw_rate']:+.4f}")
    finally:
        elapsed = time.time() - t_start
        print(f"[record] {n_steps} ticks in {elapsed:.2f}s ({n_steps/elapsed:.1f} TPS)")

        # CSV first — independent of CARLA RPC state
        if csv_rows:
            csv_path = outdir / "telemetry.csv"
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
                writer.writeheader()
                writer.writerows(csv_rows)
            print(f"[record] {len(csv_rows)} samples -> {csv_path}")

        # Best-effort cleanup; server may already be gone
        for fn in (lambda: camera.stop(),
                   lambda: camera.destroy(),
                   lambda: bridge.close()):
            try:
                fn()
            except Exception as e:
                print(f"[record] cleanup warn: {e}")
        print(f"[record] {len(saved_frames)} frames -> {outdir / 'frames'}")


if __name__ == "__main__":
    main()
