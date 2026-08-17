#!/usr/bin/env python3

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SCENE_FILE = os.path.expanduser("~/isaac_scene.json")


class FakeVisionAgent(Node):

    def __init__(self):
        super().__init__("fake_vision_agent")

        self.pub = self.create_publisher(
            String,
            "/vision/agent_state",
            10,
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_scene,
        )

        self.get_logger().info(
            f"Loading fake vision scene: {SCENE_FILE}"
        )

        with open(SCENE_FILE, "r") as f:
            self.scene = json.load(f)

        self.objects = []

        for obj_id, (label, data) in enumerate(
            self.scene.items(),
            start=1,
        ):
            xyz = data.get("xyz")

            if not isinstance(xyz, list) or len(xyz) != 3:
                continue

            self.objects.append(
                {
                    "id": obj_id,
                    "label": label,
                    "xyz": [
                        float(xyz[0]),
                        float(xyz[1]),
                        float(xyz[2]),
                    ],
                }
            )

        self.get_logger().info(
            f"Loaded {len(self.objects)} vision objects:"
        )

        for obj in self.objects:
            self.get_logger().info(
                f"  ID={obj['id']} "
                f"label={obj['label']} "
                f"xyz={obj['xyz']}"
            )

    def publish_scene(self):

        message = {
            "global_view": {
                "objects": self.objects
            }
        }

        msg = String()
        msg.data = json.dumps(message)

        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FakeVisionAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()