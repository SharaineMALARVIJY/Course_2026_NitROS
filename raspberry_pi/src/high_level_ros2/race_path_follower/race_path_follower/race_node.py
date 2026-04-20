import math
import time as t

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(navigator: BasicNavigator, p, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(p[0])
    pose.pose.position.y = float(p[1])
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(float(p[2]))
    return pose


def validate_point(logger, name: str, p) -> bool:
    if not isinstance(p, (list, tuple)) or len(p) != 3:
        logger.error(f"{name} must contain exactly 3 values: [x, y, yaw]")
        return False
    return True


def wait_for_keyboard_start() -> None:
    while True:
        key = input("Tape 'd' puis Entrée pour démarrer : ").strip().lower()
        if key == "d":
            return
        print("Commande invalide. Tape seulement 'd'.")


def wait_for_task_result(navigator: BasicNavigator, logger, lap: int, goal_idx: int):
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        if feedback:
            distance = getattr(feedback, "distance_remaining", None)
            if distance is not None:
                logger.info(
                    f"Lap {lap}: goal {goal_idx}: remaining {distance:.2f} m",
                    throttle_duration_sec=1.0 
                )
        t.sleep(0.05)
    return navigator.getResult()


def run_recovery(navigator: BasicNavigator, logger) -> None:
    logger.warn("Recovery: backup")
    navigator.backup(backup_dist=0.3, backup_speed=0.5, time_allowance=2)

    while not navigator.isTaskComplete():
        t.sleep(0.05)

    recovery_result = navigator.getResult()
    logger.info(f"Recovery result: {recovery_result}")


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator(node_name="race_node")
    logger = navigator.get_logger()

    navigator.declare_parameter("laps", 1)
    navigator.declare_parameter("max_retries", 30)

    navigator.declare_parameter("p0", [0.0, 0.0, 0.0])

    navigator.declare_parameter("use_p1", False)
    navigator.declare_parameter("use_p2", False)
    navigator.declare_parameter("use_p3", False)
    navigator.declare_parameter("use_p4", False)

    navigator.declare_parameter("p1", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p2", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p3", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p4", [0.0, 0.0, 0.0])

    laps = int(navigator.get_parameter("laps").value)
    max_retries = int(navigator.get_parameter("max_retries").value)

    p0 = navigator.get_parameter("p0").value

    use_p1 = navigator.get_parameter("use_p1").value
    use_p2 = navigator.get_parameter("use_p2").value
    use_p3 = navigator.get_parameter("use_p3").value
    use_p4 = navigator.get_parameter("use_p4").value

    p1 = navigator.get_parameter("p1").value
    p2 = navigator.get_parameter("p2").value
    p3 = navigator.get_parameter("p3").value
    p4 = navigator.get_parameter("p4").value

    if not validate_point(logger, "p0", p0):
        navigator.destroy_node()
        rclpy.shutdown()
        return

    points = {
        "p1": (use_p1, p1),
        "p2": (use_p2, p2),
        "p3": (use_p3, p3),
        "p4": (use_p4, p4),
    }

    goal_points_one_lap = []

    for name, (use, p) in points.items():
        if not validate_point(logger, name, p):
            navigator.destroy_node()
            rclpy.shutdown()
            return

        if use:
            goal_points_one_lap.append((name, make_pose(navigator, p)))

    if not goal_points_one_lap:
        logger.error("No active goal points. Enable at least one of use_p1..use_p4")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if laps < 1:
        logger.error("laps must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if max_retries < 1:
        logger.error("max_retries must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    logger.info(f"Initial pose: {p0}")
    logger.info(f"Active goals per lap: {len(goal_points_one_lap)}")
    logger.info(f"Total goals over all laps: {len(goal_points_one_lap) * laps}")
    logger.info(f"Max retries per goal: {max_retries}")

    navigator.setInitialPose(make_pose(navigator, p0))

    logger.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    logger.info("Nav2 ready")

    wait_for_keyboard_start()

    for lap in range(1, laps + 1):
        logger.info(f"Starting lap {lap}/{laps}")

        for goal_idx, (goal_name, goal_values) in enumerate(goal_points_one_lap, start=1):
            success = False

            for attempt in range(1, max_retries + 1):
                
                logger.info(
                    f"Lap {lap}: sending {goal_name} "
                    f"({goal_idx}/{len(goal_points_one_lap)}) "
                    f"attempt {attempt}/{max_retries}"
                )

                navigator.goToPose(goal_values)
                result = wait_for_task_result(navigator, logger, lap, goal_idx)

                if result == TaskResult.SUCCEEDED:
                    logger.info(f"Lap {lap}: {goal_name} succeeded")
                    success = True
                    break

                if result == TaskResult.CANCELED:
                    logger.warn(
                        f"Lap {lap}: {goal_name} canceled "
                        f"({attempt}/{max_retries})"
                    )
                elif result == TaskResult.FAILED:
                    logger.warn(
                        f"Lap {lap}: {goal_name} failed "
                        f"({attempt}/{max_retries})"
                    )
                    run_recovery(navigator, logger)
                else:
                    logger.warn(
                        f"Lap {lap}: {goal_name} unknown result={result} "
                        f"({attempt}/{max_retries})"
                    )

            if not success:
                logger.error(
                    f"Lap {lap}: {goal_name} reached max retries "
                    f"({max_retries}), skipping"
                )

    logger.info("All laps completed.")
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
