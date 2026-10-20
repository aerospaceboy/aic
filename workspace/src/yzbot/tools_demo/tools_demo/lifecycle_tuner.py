"""Apply load-tolerant heartbeat timeouts to Nav2 lifecycle managers."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


MANAGERS = (
    "/lifecycle_manager",
    "/lifecycle_manager_localization",
    "/lifecycle_manager_navigation",
)


class LifecycleTuner(Node):
    """Set lifecycle bond timeouts after each remote manager appears."""

    def __init__(self):
        super().__init__("competition_lifecycle_tuner")
        self.declare_parameter("bond_timeout", 12.0)
        self.declare_parameter("discovery_timeout", 90.0)

    def tune(self):
        timeout = float(self.get_parameter("bond_timeout").value)
        discovery_timeout = float(
            self.get_parameter("discovery_timeout").value
        )
        deadline = time.monotonic() + discovery_timeout
        failures = []
        for manager in MANAGERS:
            client = AsyncParameterClient(self, manager)
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_services(timeout_sec=remaining):
                failures.append(f"{manager}: parameter service unavailable")
                continue
            future = client.set_parameters(
                [Parameter("bond_timeout", value=timeout)]
            )
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=max(0.0, deadline - time.monotonic()),
            )
            results = future.result() if future.done() else None
            if not results or not all(result.successful for result in results):
                reason = "set timed out"
                if results:
                    reason = "; ".join(
                        result.reason or "rejected"
                        for result in results
                        if not result.successful
                    )
                failures.append(f"{manager}: {reason}")
                continue
            self.get_logger().info(
                f"lifecycle_bond_timeout_ready manager={manager} "
                f"timeout={timeout:.1f}s"
            )
        return failures


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleTuner()
    failures = node.tune()
    for failure in failures:
        node.get_logger().error(f"lifecycle_tuning_failed {failure}")
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
