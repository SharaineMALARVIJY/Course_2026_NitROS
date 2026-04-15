import math
import time as t
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


Point = Tuple[float, float, float]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(navigator: BasicNavigator, p: Point, frame_id: str = "map") -> PoseStamped:
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


def wait_for_task_result(navigator: BasicNavigator, logger, log_freq: int):
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % log_freq == 0:
            distance = getattr(feedback, "distance_to_goal", None)
            if distance is None:
                distance = getattr(feedback, "distance_remaining", None)

            if distance is not None:
                logger.info(f"Distance restante: {distance:.2f} m")

        t.sleep(0.05)

    return navigator.getResult()


def append_path(dst: Path, src: Path) -> None:
    """
    Concatène src dans dst en évitant de dupliquer
    le point de jonction entre deux segments.
    """
    if src is None or not src.poses:
        return

    if not dst.poses:
        dst.header = src.header
        dst.poses = list(src.poses)
        return

    dst.poses.extend(src.poses[1:])


def repeat_path(navigator: BasicNavigator, base_path: Path, laps: int) -> Path:
    """
    Répète un chemin de base 'laps' fois.
    On évite de dupliquer la première pose au raccord entre deux tours.
    """
    repeated = Path()
    repeated.header.frame_id = base_path.header.frame_id if base_path.header.frame_id else "map"
    repeated.header.stamp = navigator.get_clock().now().to_msg()

    if laps < 1 or not base_path.poses:
        return repeated

    for lap_idx in range(laps):
        if lap_idx == 0:
            repeated.poses.extend(base_path.poses)
        else:
            repeated.poses.extend(base_path.poses[1:])

    return repeated


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator(node_name="race_node")
    logger = navigator.get_logger()

    navigator.declare_parameter("laps", 1)
    navigator.declare_parameter("log_frequency", 5)

    navigator.declare_parameter("planner_id", "GridBased")
    navigator.declare_parameter("controller_id", "FollowPath")

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
    log_freq = int(navigator.get_parameter("log_frequency").value)
    planner_id = str(navigator.get_parameter("planner_id").value)
    controller_id = str(navigator.get_parameter("controller_id").value)

    p0 = tuple(navigator.get_parameter("p0").value)

    use_p1 = bool(navigator.get_parameter("use_p1").value)
    use_p2 = bool(navigator.get_parameter("use_p2").value)
    use_p3 = bool(navigator.get_parameter("use_p3").value)
    use_p4 = bool(navigator.get_parameter("use_p4").value)

    p1 = tuple(navigator.get_parameter("p1").value)
    p2 = tuple(navigator.get_parameter("p2").value)
    p3 = tuple(navigator.get_parameter("p3").value)
    p4 = tuple(navigator.get_parameter("p4").value)

    if not validate_point(logger, "p0", p0):
        navigator.destroy_node()
        rclpy.shutdown()
        return

    selected_points = {
        "p1": p1,
        "p2": p2,
        "p3": p3,
        "p4": p4,
    }

    for name, p in selected_points.items():
        if not validate_point(logger, name, p):
            navigator.destroy_node()
            rclpy.shutdown()
            return

    goal_points_one_lap: List[Tuple[str, Point]] = []
    if use_p1:
        goal_points_one_lap.append(("p1", p1))
    if use_p2:
        goal_points_one_lap.append(("p2", p2))
    if use_p3:
        goal_points_one_lap.append(("p3", p3))
    if use_p4:
        goal_points_one_lap.append(("p4", p4))

    if len(goal_points_one_lap) < 2:
        logger.error("Need at least 2 active points. Enable at least two of use_p1..use_p4")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if laps < 1:
        logger.error("laps must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if log_freq < 1:
        logger.error("log_frequency must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    logger.info(f"Initial pose p0: {p0}")
    logger.info(f"Planner: {planner_id}")
    logger.info(f"Controller: {controller_id}")
    logger.info(f"Laps: {laps}")
    logger.info(f"Ordered waypoints: {[name for name, _ in goal_points_one_lap]}")

    navigator.setInitialPose(make_pose(navigator, p0))

    logger.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    logger.info("Nav2 ready")

    wait_for_keyboard_start()

    lap_path = Path()
    lap_path.header.frame_id = "map"
    lap_path.header.stamp = navigator.get_clock().now().to_msg()

    first_name, first_values = goal_points_one_lap[0]

    logger.info(f"Planning initial segment p0 -> {first_name}")
    initial_segment = navigator.getPath(
        start=make_pose(navigator, p0),
        goal=make_pose(navigator, first_values),
        planner_id=planner_id,
        use_start=True,
    )

    if initial_segment is None or not initial_segment.poses:
        logger.error(f"Failed to plan initial segment p0 -> {first_name}")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    logger.info(f"Initial segment p0->{first_name}: {len(initial_segment.poses)} poses")
    append_path(lap_path, initial_segment)

    logger.info("Planning lap segments once at startup...")
    for i in range(len(goal_points_one_lap)):
        start_name, start_values = goal_points_one_lap[i]
        goal_name, goal_values = goal_points_one_lap[(i + 1) % len(goal_points_one_lap)]

        logger.info(f"Planning segment {start_name} -> {goal_name}")
        segment = navigator.getPath(
            start=make_pose(navigator, start_values),
            goal=make_pose(navigator, goal_values),
            planner_id=planner_id,
            use_start=True,
        )

        if segment is None or not segment.poses:
            logger.error(f"Failed to plan segment {start_name} -> {goal_name}")
            navigator.destroy_node()
            rclpy.shutdown()
            return

        logger.info(f"Segment {start_name}->{goal_name}: {len(segment.poses)} poses")
        append_path(lap_path, segment)

    if not lap_path.poses:
        logger.error("Generated lap path is empty")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    race_path = repeat_path(navigator, lap_path, laps)

    if not race_path.poses:
        logger.error("Generated race path is empty")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    logger.info(f"Lap path poses: {len(lap_path.poses)}")
    logger.info(f"Race path poses: {len(race_path.poses)}")

    logger.info("Starting FollowPath...")
    navigator.followPath(
        path=race_path,
        controller_id=controller_id,
    )

    result = wait_for_task_result(navigator, logger, log_freq)

    if result == TaskResult.SUCCEEDED:
        logger.info("Race path completed successfully")
    elif result == TaskResult.CANCELED:
        logger.warn("Race path canceled")
    elif result == TaskResult.FAILED:
        logger.error("Race path failed")
    else:
        logger.warn(f"Race path unknown result={result}")

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()