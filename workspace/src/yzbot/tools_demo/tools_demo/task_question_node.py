"""Publish reproducible random competition questions."""

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from tools_demo.task_protocol import generate_task_question


def task_qos() -> QoSProfile:
    """Return reliable transient-local QoS for one active round."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def input_chunk_qos() -> QoSProfile:
    """Accept live CoBridge commands without replaying an old submission."""
    return QoSProfile(
        depth=128,
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class TaskQuestionNode(Node):
    """Generate one question or a configured stream of rounds."""

    def __init__(self) -> None:
        super().__init__("competition_task_question")
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("publish_delay_sec", 1.0)
        self.declare_parameter("period_sec", 0.0)
        self.declare_parameter(
            "question_topic",
            "/competition/task/question",
        )

        seed = int(self.get_parameter("random_seed").value)
        self.randomizer = random.Random(None if seed < 0 else seed)
        self.period_sec = float(self.get_parameter("period_sec").value)
        delay_sec = float(self.get_parameter("publish_delay_sec").value)
        topic = str(self.get_parameter("question_topic").value)
        self.publisher = self.create_publisher(String, topic, task_qos())
        interval = self.period_sec if self.period_sec > 0.0 else delay_sec
        self.timer = self.create_timer(max(interval, 0.05), self.publish_question)
        self.round_number = 0

    def publish_question(self) -> None:
        """Publish a newly generated task question."""
        self.round_number += 1
        message = String()
        message.data = generate_task_question(self.randomizer)
        self.publisher.publish(message)
        self.get_logger().info(
            f"round={self.round_number} question={message.data}"
        )
        if self.period_sec <= 0.0:
            self.timer.cancel()


def main(args=None) -> None:
    """Run the task question node."""
    rclpy.init(args=args)
    node = TaskQuestionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
