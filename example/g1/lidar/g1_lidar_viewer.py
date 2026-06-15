"""
G1 LiDAR pontfelhő viewer — terminál statisztika + Rerun 3D plot.

A G1 Livox MID360 LiDAR-ja DDS-en publikálja a pontfelhőt.
Raw scan topic:  rt/utlidar/cloud_livox_mid360  (élő, 10 Hz)
SLAM map topic:  rt/unitree/slam_mapping/points  (akkumulált, padlóval)

Telepítés (--plot-hoz):
    pip install rerun-sdk numpy

Használat:
    python3 g1_lidar_viewer.py en6
    python3 g1_lidar_viewer.py en6 --plot
    python3 g1_lidar_viewer.py en6 --maxdist 8
"""

import sys
import time
import struct
import argparse
import threading
import math
from collections import deque

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


def parse_cloud(msg):
    """PointCloud2 bináris adatból (x, y, z) float pontok listája."""
    offsets = {f.name: f.offset for f in msg.fields}
    x_off = offsets.get('x', 0)
    y_off = offsets.get('y', 4)
    z_off = offsets.get('z', 8)
    ps = msg.point_step
    n = msg.width * msg.height
    raw = bytes(msg.data)
    points = []
    for i in range(n):
        base = i * ps
        if base + max(x_off, y_off, z_off) + 4 > len(raw):
            break
        x = struct.unpack_from('<f', raw, base + x_off)[0]
        y = struct.unpack_from('<f', raw, base + y_off)[0]
        z = struct.unpack_from('<f', raw, base + z_off)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


class LidarViewer:
    def __init__(self, topic, max_dist, n_frames):
        self.topic = topic
        self.max_dist = max_dist
        self.lock = threading.Lock()
        self.frame_buf = deque(maxlen=n_frames)  # utolsó N frame pontjai
        self._frame_count = 0
        self._fps_ts = time.time()
        self.fps = 0.0
        self.received_any = False

    def on_cloud(self, msg: PointCloud2_):
        points = parse_cloud(msg)
        self.received_any = True

        now = time.time()
        self._frame_count += 1
        dt = now - self._fps_ts
        if dt >= 1.0:
            self.fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_ts = now

        with self.lock:
            self.frame_buf.append(points)

        if not points:
            print("\r[LiDAR] üres frame              ", end="", flush=True)
            return

        dists = [math.sqrt(x*x + y*y + z*z) for x, y, z in points]
        mn, mx, mean = min(dists), max(dists), sum(dists) / len(dists)
        print(
            f"\r[LiDAR] {len(points):5d} pont  "
            f"min={mn:.2f}m  mean={mean:.2f}m  max={mx:.2f}m  "
            f"{self.fps:.1f} Hz    ",
            end="", flush=True,
        )

    def run_plot(self):
        try:
            import rerun as rr
            import numpy as np
        except ImportError:
            print("\n[Hiba] rerun-sdk nincs telepítve: pip install rerun-sdk")
            return

        rr.init("G1 LiDAR", spawn=True)
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_DOWN, static=True)
        print("Rerun viewer megnyílt. Ctrl+C a kilépéshez.\n")

        # Origó: koordináta-tengelyek (X=piros/előre, Y=zöld/bal, Z=kék/fel)
        rr.log("lidar/origó", rr.Arrows3D(
            origins=np.zeros((3, 3), dtype=np.float32),
            vectors=np.eye(3, dtype=np.float32) * 0.5,
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            radii=0.03,
        ), static=True)

        while True:
            with self.lock:
                frames = list(self.frame_buf)

            if frames:
                all_pts = [p for frame in frames for p in frame]
                arr = np.array(all_pts, dtype=np.float32)
                dist = np.linalg.norm(arr, axis=1)
                mask = dist <= self.max_dist
                arr, dist = arr[mask], dist[mask]

                if len(arr) > 0:
                    t = np.clip(dist / self.max_dist, 0, 1)
                    r = np.clip((0.5 + 2.0*t - 1.5*t**2) * 255, 0, 255).astype(np.uint8)
                    g = np.clip((0.05 + 1.8*t**0.6 - 2.0*t**2) * 255, 0, 255).astype(np.uint8)
                    b = np.clip((0.55 - 0.6*t - 0.2*t**2) * 255, 0, 255).astype(np.uint8)
                    rr.log("lidar/scan", rr.Points3D(
                        arr, colors=np.column_stack([r, g, b]), radii=0.02,
                    ))

            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="G1 LiDAR pontfelhő viewer")
    parser.add_argument("net", help="hálózati interfész, pl. en6")
    parser.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360",
                        help="DDS topic (alap: rt/utlidar/cloud_livox_mid360)")
    parser.add_argument("--plot", action="store_true",
                        help="Rerun 3D viewer megnyitása")
    parser.add_argument("--maxdist", type=float, default=10.0,
                        help="max távolság (alap: 10 m)")
    parser.add_argument("--frames", type=int, default=1,
                        help="hány frame-t akkumuláljon egyszerre (alap: 1, padlóhoz: 30-50)")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.net)

    viewer = LidarViewer(args.topic, args.maxdist, args.frames)

    sub = ChannelSubscriber(args.topic, PointCloud2_)
    sub.Init(viewer.on_cloud, 10)

    print(f"Feliratkozva: {args.topic}  ({args.frames} frame akkumulálva)")
    print("Ctrl+C a kilépéshez.\n")

    def _no_data_warn():
        time.sleep(5)
        if not viewer.received_any:
            print(
                "\n[!] 5 mp alatt nem érkezett adat.\n"
                "    Ellenőrizd, hogy a robot LiDAR-ja be van-e kapcsolva.\n"
            )
    threading.Thread(target=_no_data_warn, daemon=True).start()

    if args.plot:
        try:
            viewer.run_plot()
        except KeyboardInterrupt:
            pass
    else:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    print("\nKilépés.")


if __name__ == "__main__":
    main()
