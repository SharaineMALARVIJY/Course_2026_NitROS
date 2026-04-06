import math
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


def make_pose(navigator, p, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = p[0]
    pose.pose.position.y = p[1]
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(p[2])
    return pose


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator(node_name="race_node")
    logger = navigator.get_logger()

    navigator.declare_parameter("reverse", False)
    navigator.declare_parameter("laps", 1)
    navigator.declare_parameter("log_frequency", 5)

    navigator.declare_parameter("p0", [0.0, 0.0, 0.0])

    navigator.declare_parameter("use_p1", True)
    navigator.declare_parameter("use_p2", True)
    navigator.declare_parameter("use_p3", True)
    navigator.declare_parameter("use_p4", False)

    navigator.declare_parameter("p1", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p2", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p3", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p4", [0.0, 0.0, 0.0])

    navigator.declare_parameter("p1_reverse", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p2_reverse", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p3_reverse", [0.0, 0.0, 0.0])
    navigator.declare_parameter("p4_reverse", [0.0, 0.0, 0.0])

    reverse = navigator.get_parameter("reverse").value
    laps = navigator.get_parameter("laps").value
    log_freq = navigator.get_parameter("log_frequency").value

    p0 = navigator.get_parameter("p0").value

    use_p1 = navigator.get_parameter("use_p1").value
    use_p2 = navigator.get_parameter("use_p2").value
    use_p3 = navigator.get_parameter("use_p3").value
    use_p4 = navigator.get_parameter("use_p4").value

    p1 = navigator.get_parameter("p1").value
    p2 = navigator.get_parameter("p2").value
    p3 = navigator.get_parameter("p3").value
    p4 = navigator.get_parameter("p4").value

    p1_reverse = navigator.get_parameter("p1_reverse").value
    p2_reverse = navigator.get_parameter("p2_reverse").value
    p3_reverse = navigator.get_parameter("p3_reverse").value
    p4_reverse = navigator.get_parameter("p4_reverse").value

    if reverse:
        logger.info("Reverse mode enabled")
        selected_points = {
            "p1": p1_reverse,
            "p2": p2_reverse,
            "p3": p3_reverse,
            "p4": p4_reverse,
        }
    else:
        selected_points = {
            "p1": p1,
            "p2": p2,
            "p3": p3,
            "p4": p4,
        }

    for name, p in selected_points.items():
        if len(p) != 3:
            logger.error(f"{name} must contain exactly 3 values: [x, y, yaw]")
            navigator.destroy_node()
            rclpy.shutdown()
            return

    goal_points_one_lap = []
    if use_p1:
        goal_points_one_lap.append(selected_points["p1"])
    if use_p2:
        goal_points_one_lap.append(selected_points["p2"])
    if use_p3:
        goal_points_one_lap.append(selected_points["p3"])
    if use_p4:
        goal_points_one_lap.append(selected_points["p4"])

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

    if log_freq < 1:
        logger.error("log_frequency must be >= 1")
        navigator.destroy_node()
        rclpy.shutdown()
        return


    logger.info(f"Initial pose: {p0}")
    logger.info(f"Active goals per lap: {len(goal_points_one_lap)}")
    logger.info(f"Total goals over all laps: {len(goal_points_one_lap) * laps}")

    navigator.setInitialPose(make_pose(navigator, p0))

    logger.info("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    logger.info("Nav2 ready")

    lap_goals = [make_pose(navigator, p) for p in goal_points_one_lap]

    for lap in range(laps):
        logger.info(f"Starting lap {lap+1}/{laps}")
        #lap_goals = [make_pose(navigator, p) for p in goal_points_one_lap]

        for goal in lap_goals:
            navigator.goToPose(goal)

            i = 0
            while not navigator.isTaskComplete():
                i += 1
                feedback = navigator.getFeedback()
                if feedback and i % log_freq == 0:
                    logger.info(f"Lap {lap+1}: remaining {feedback.distance_remaining:.2f} m")

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()