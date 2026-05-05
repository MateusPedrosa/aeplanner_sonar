#!/usr/bin/env python3
#
# Captures one MarkerArray message from /variance_vis_array and writes it as a
# binary PLY file.  Called automatically at exploration completion.
#
# Usage:  variance_to_ply.py [output_path]
#   output_path defaults to "variance.ply" in the current directory.
#
# Color scale matches bgkloctomap's insert_color_point3d / heightMapColor:
#   h = (1.0 - clamp(variance / max_var_vis, 0, 1)) * 0.8
#   RGB = HSV(h, s=1, v=1)  — low variance -> blue/cyan, high variance -> red
# Default max_var_vis = 0.25 (matches bgkloctomap_server ROS param).

import sys
import rospy
from visualization_msgs.msg import MarkerArray
import numpy as np


class VarianceToPly:
    def __init__(self, topic_name, output_filename):
        self.output_filename = output_filename
        self.sub = rospy.Subscriber(topic_name, MarkerArray, self.callback)
        rospy.loginfo(f"Listening to '{topic_name}'...")

    def callback(self, msg):
        rospy.loginfo(f"Message received with {len(msg.markers)} markers. Starting extraction...")

        all_points = []
        all_colors = []

        for marker in msg.markers:
            pts = np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)

            if pts.shape[0] == 0:
                pts = np.array([[marker.pose.position.x,
                                 marker.pose.position.y,
                                 marker.pose.position.z]], dtype=np.float32)

            num_pts = pts.shape[0]
            if num_pts == 0:
                continue

            if len(marker.colors) == num_pts:
                cols = np.array([[c.r, c.g, c.b] for c in marker.colors], dtype=np.float32)
            else:
                cols = np.tile([marker.color.r, marker.color.g, marker.color.b], (num_pts, 1))

            all_points.append(pts)
            all_colors.append(cols)

        if not all_points:
            rospy.logwarn("No points found in MarkerArray!")
            return

        for marker in msg.markers:
            if len(marker.points) > 0:
                rospy.loginfo(f"  ns={marker.ns!r:12s} id={marker.id}  scale={marker.scale.x:.4f}  pts={len(marker.points)}")

        final_pts = np.vstack(all_points)
        final_cols = (np.vstack(all_colors) * 255).astype(np.uint8)

        rospy.loginfo(f"Total points extracted: {final_pts.shape[0]}")
        self.save_binary_ply(final_pts, final_cols)
        rospy.loginfo("Extraction complete.")
        rospy.signal_shutdown("Done.")

    def save_binary_ply(self, points, colors):
        num_points = points.shape[0]
        rospy.loginfo(f"Writing {num_points} points to binary PLY: {self.output_filename}")

        with open(self.output_filename, 'wb') as f:
            header = (
                f"ply\n"
                f"format binary_little_endian 1.0\n"
                f"element vertex {num_points}\n"
                f"property float x\n"
                f"property float y\n"
                f"property float z\n"
                f"property uchar red\n"
                f"property uchar green\n"
                f"property uchar blue\n"
                f"end_header\n"
            ).encode('ascii')
            f.write(header)

            dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                              ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
            data = np.empty(num_points, dtype=dtype)
            data['x'], data['y'], data['z'] = points[:, 0], points[:, 1], points[:, 2]
            data['red'], data['green'], data['blue'] = colors[:, 0], colors[:, 1], colors[:, 2]
            f.write(data.tobytes())

        rospy.loginfo(f"Saved to {self.output_filename}")


if __name__ == '__main__':
    output = sys.argv[1] if len(sys.argv) > 1 else "variance.ply"
    rospy.init_node('variance_to_ply', anonymous=True)
    converter = VarianceToPly("/variance_vis_array", output)
    rospy.spin()
