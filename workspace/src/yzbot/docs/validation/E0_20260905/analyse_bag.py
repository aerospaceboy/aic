#!/usr/bin/env python3
"""Analyse one E1 observation bag: per-sample (v, w) episodes and plans.

Produces the low-bandwidth per-action evidence required by the E0/E1
plan: rotating-in-place episodes (|v| < 0.05, |w| >= 0.2), slow-motion
episodes, plan publication times and path lengths. Timestamps are bag
(sim) time; correlation with launch-log segments is by ORDER of plan
publications.

Usage: python3 analyse_bag.py <bag_dir> <out_json>
"""
import json
import math
import sys

import rclpy.serialization
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    messages = {"odom": [], "cmd": [], "cmd_safe": [], "plan": []}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic.endswith("/odom") or topic == "/odom":
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            messages["odom"].append(
                (stamp / 1e9, msg.twist.twist.linear.x,
                 msg.twist.twist.angular.z,
                 msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == "/cmd_vel":
            msg = rclpy.serialization.deserialize_message(data, Twist)
            messages["cmd"].append(
                (stamp / 1e9, msg.linear.x, msg.angular.z))
        elif topic.startswith("/cmd_vel_collision_safe") or \
                topic == "/cmd_vel_safe":
            msg = rclpy.serialization.deserialize_message(data, Twist)
            messages["cmd_safe"].append(
                (stamp / 1e9, msg.linear.x, msg.angular.z))
        elif topic == "/plan":
            msg = rclpy.serialization.deserialize_message(data, Path)
            poses = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]
            length = sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(poses, poses[1:]))
            messages["plan"].append((stamp / 1e9, len(poses), length))
    return messages, {t.name: t.type for t in topics}


def episodes(samples, is_episode):
    """Group consecutive samples satisfying is_episode into episodes."""
    out = []
    current = None
    for row in samples:
        if is_episode(row):
            if current is None:
                current = {"t0": row[0], "t1": row[0], "w_signed": 0.0,
                           "n": 0, "w_max": 0.0}
            current["t1"] = row[0]
            current["n"] += 1
            w = abs(row[2])
            current["w_signed"] += row[2] * 0.1  # rough, refined below
            current["w_max"] = max(current["w_max"], w)
        else:
            if current is not None and current["t1"] - current["t0"] >= 0.3:
                out.append(current)
            current = None
    if current is not None and current["t1"] - current["t0"] >= 0.3:
        out.append(current)
    for ep in out:
        ep["duration"] = round(ep["t1"] - ep["t0"], 2)
    return out


def main():
    bag_dir, out_json = sys.argv[1], sys.argv[2]
    messages, type_map = read_bag(bag_dir)
    odom = messages["odom"]
    print("odom samples:", len(odom), "cmd:", len(messages["cmd"]),
          "plan:", len(messages["plan"]))
    if not odom:
        print("NO ODOMETRY; topics:", list(type_map)[:20])
        json.dump({"error": "no odom", "topics": list(type_map)},
                  open(out_json, "w"), indent=2)
        return
    t0, t1 = odom[0][0], odom[-1][0]
    spin = episodes(odom, lambda r: abs(r[1]) < 0.05 and abs(r[2]) >= 0.2)
    slow = episodes(odom, lambda r: 0.05 <= abs(r[1]) < 0.2)
    total_spin = sum(e["duration"] for e in spin)
    total_slow = sum(e["duration"] for e in slow)
    print("sim span: %.1f s; rotate-in-place: %.1f s in %d episodes; "
          "slow-roll: %.1f s in %d episodes"
          % (t1 - t0, total_spin, len(spin), total_slow, len(slow)))
    result = {
        "bag_dir": bag_dir,
        "sim_span_sec": round(t1 - t0, 2),
        "odom_samples": len(odom),
        "plan_publications": len(messages["plan"]),
        "rotate_in_place_total_sec": round(total_spin, 2),
        "slow_roll_total_sec": round(total_slow, 2),
        "rotate_episodes": sorted(
            spin, key=lambda e: -e["duration"])[:15],
        "slow_episodes": sorted(
            slow, key=lambda e: -e["duration"])[:15],
        "plan_timeline": [
            {"t": round(t - t0, 2), "poses": n,
             "length_m": round(length, 2)}
            for t, n, length in messages["plan"]],
    }
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    print("wrote", out_json)


if __name__ == "__main__":
    main()
