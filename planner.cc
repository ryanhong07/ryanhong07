#include "basic_waypoint_pkg/planner.hpp"

#include <utility>

BasicPlanner::BasicPlanner(const rclcpp::Node::SharedPtr & node)
: node_(node),
  current_pose_(Eigen::Affine3d::Identity()),
  current_velocity_(Eigen::Vector3d::Zero()),
  current_angular_velocity_(Eigen::Vector3d::Zero()),
  max_v_(0.2),
  max_a_(0.2),
  max_ang_v_(0.0),
  max_ang_a_(0.0)
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  To Do: Load Trajectory Parameters from parameter file (ROS2)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  //
  // ~~~~ begin solution
  // 1. Declare parameters with default values (safeguard against missing yaml keys)
  node_->declare_parameter("max_v", 1.0);
  node_->declare_parameter("max_a", 1.0);
  node_->declare_parameter("max_ang_v", 1.0);
  node_->declare_parameter("max_ang_a", 1.0);

  // 2. Get the values from the configuration file
  node_->get_parameter("max_v", max_v_);
  node_->get_parameter("max_a", max_a_);
  node_->get_parameter("max_ang_v", max_ang_v_);
  node_->get_parameter("max_ang_a", max_ang_a_);
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Publishers
  pub_markers_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

  pub_trajectory_ =
    node_->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "trajectory", 10);

  // Subscriber for Odometry
  sub_odom_ =
    node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&BasicPlanner::uavOdomCallback, this, std::placeholders::_1));
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // store current position in our planner
  tf2::fromMsg(odom->pose.pose, current_pose_);

  // store current velocity
  tf2::fromMsg(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v)
{
  max_v_ = max_v;
}

// Plans a trajectory from the current position to a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(
  const Eigen::VectorXd & goal_pos,
  const Eigen::VectorXd & goal_vel,
  mav_trajectory_generation::Trajectory * trajectory)
{
  // 3 Dimensional trajectory => through Cartesian space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
    mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // End   = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(
    current_pose_.translation(),
    derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY,
    current_velocity_);

  // add waypoint to list
  vertices.push_back(start);

  /******* Configure trajectory (intermediate waypoints) *******/
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  To Do: Set up trajectory waypoints
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // In this section, you need to
  // - load waypoint definition (pos, vel, acc) per dimension from params
  // - dynamically set constraints for each (and only where needed)
  // - push waypoints to vertices
  //
  // ~~~~ begin solution
  // 1. Define the Obstacles (Coordinates from Lab Handout)
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(Eigen::Vector3d(10.0, 0.0, 3.0));   // p1
  waypoints.push_back(Eigen::Vector3d(30.0, 10.0, 3.0));  // p2
  waypoints.push_back(Eigen::Vector3d(35.0, 25.0, 13.0)); // p3
  waypoints.push_back(Eigen::Vector3d(35.0, 35.0, 6.0));  // p4
  waypoints.push_back(Eigen::Vector3d(25.0, 39.0, 11.0)); // p5 (Tunnel Start)
  waypoints.push_back(Eigen::Vector3d(16.0, 39.0, 11.0)); // p6 (Tunnel End)

  // Helper to get the position of the last vertex added
  auto getLastPos = [&](const mav_trajectory_generation::Vertex::Vector& verts) -> Eigen::Vector3d {
      Eigen::VectorXd pos; 
      verts.back().getConstraint(mav_trajectory_generation::derivative_order::POSITION, &pos);
      return pos; 
  };

  // Helper to safely add a Position-Only vertex
  auto addPosVertex = [&](const Eigen::Vector3d& pos) {
      if (vertices.empty() || (pos - getLastPos(vertices)).norm() > 0.1) {
          mav_trajectory_generation::Vertex v(dimension);
          v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
          vertices.push_back(v);
      }
  };

  // --- LOOP 1: Fast Pass ---
  for (size_t i = 0; i < waypoints.size(); ++i) {
      addPosVertex(waypoints[i]);
  }

  // Connect Loop 1 -> Loop 2 (Return to Origin)
  addPosVertex(Eigen::Vector3d(0.0, 0.0, 0.0));

  // --- LOOP 2: Precision Run ---
  
  // Leg 1: Start -> p1
  addPosVertex(waypoints[0]); 

  // Leg 2: p1 -> p2 (Brake)
  Eigen::Vector3d p1_p2_mid = (waypoints[0] + waypoints[1]) * 0.5; 
  addPosVertex(p1_p2_mid);
  addPosVertex(waypoints[1]); // p2

  // Leg 3: p2 -> p3 (Smoother)
  Eigen::Vector3d p2_p3_mid = (waypoints[1] + waypoints[2]) * 0.5;
  addPosVertex(p2_p3_mid);
  addPosVertex(waypoints[2]); // p3

  // Leg 4: p3 -> p4
  addPosVertex(waypoints[3]); // p4

  // Leg 5: p4 -> Tunnel (The "Square Turn" Fix)
  // p4 is (35, 35, 6). Tunnel is (25, 39, 11).
  // We force alignment WAY back at x=30 to prevent corner cutting.
  
  // Step 1: Climb and align Y early (at x=30)
  // This makes the drone fly "North" to y=39 while it is still far from the tunnel (x=30).
  addPosVertex(Eigen::Vector3d(30.0, 39.0, 11.0)); 
  
  // Step 2: Enter Tunnel (p5)
  // Now we just fly straight West from x=30 to x=25. No turning involved.
  addPosVertex(waypoints[4]);

  // Step 3: Mid-Tunnel Pin (The "Anti-Wobble" Fix)
  // Force the drone to stay center at x=20.5.
  addPosVertex(Eigen::Vector3d(20.5, 39.0, 11.0));

  // *** STOP: Tunnel End (p6) ***
  if ((waypoints.back() - getLastPos(vertices)).norm() > 0.1) {
      mav_trajectory_generation::Vertex tunnel_stop(dimension);
      tunnel_stop.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints.back());
      tunnel_stop.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
      tunnel_stop.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d::Zero());
      vertices.push_back(tunnel_stop);
  }
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(
    goal_pos,
    derivative_to_optimize);

  // set end point's velocity to be constrained to desired velocity
  end.addConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY,
    goal_vel);

  // add waypoint to list
  vertices.push_back(end);

  // estimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
    dimension, parameters);
  opt.setupFromVertices(
    vertices, segment_times,
    derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(trajectory);

  return true;
}

// Overload using explicit start state and limits (currently just a stub, same as above)
bool BasicPlanner::planTrajectory(
  const Eigen::VectorXd & goal_pos,
  const Eigen::VectorXd & goal_vel,
  const Eigen::VectorXd & start_pos,
  const Eigen::VectorXd & start_vel,
  double v_max, double a_max,
  mav_trajectory_generation::Trajectory * trajectory)
{
  // You can either implement a different variant or simply reuse the other method.
  (void)start_pos;
  (void)start_vel;
  (void)v_max;
  (void)a_max;
  return planTrajectory(goal_pos, goal_vel, trajectory);
}

bool BasicPlanner::publishTrajectory(
  const mav_trajectory_generation::Trajectory & trajectory)
{
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::msg::MarkerArray markers;
  double distance = 0.2;  // distance between markers; 0.0 to disable
  std::string frame_id = "world";

  drawMavTrajectory(
    trajectory, distance, frame_id, &markers);
  pub_markers_->publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
    trajectory, &msg);
  msg.header.frame_id = "world";
  // optionally: msg.header.stamp = node_->now();
  pub_trajectory_->publish(msg);

  return true;
}

void BasicPlanner::drawMavTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory,
    double distance, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray* marker_array) {
    // sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, 0.1, &trajectory_points);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Could not sample trajectory.");
        return;
    }

    // draw trajectory
    marker_array->markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // orange
    std_msgs::msg::ColorRGBA line_strip_color;
    line_strip_color.r = 1.0;
    line_strip_color.g = 0.5;
    line_strip_color.b = 0.0;
    line_strip_color.a = 1.0;
    line_strip.color = line_strip_color;
    line_strip.scale.x = 0.01;
    line_strip.ns = "path";

    double accumulated_distance = 0.0;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    double scale = 0.3;
    double diameter = 0.3;
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const mav_msgs::EigenTrajectoryPoint& point = trajectory_points[i];

        accumulated_distance += (last_position - point.position_W).norm();
        if (accumulated_distance > distance) {
            accumulated_distance = 0.0;
            mav_msgs::EigenMavState mav_state;
            mav_msgs::EigenMavStateFromEigenTrajectoryPoint(point, &mav_state);
            mav_state.orientation_W_B = point.orientation_W_B;

            visualization_msgs::msg::MarkerArray axes_arrows;
            axes_arrows.markers.resize(3);

            // x axis
            visualization_msgs::msg::Marker arrow_marker = axes_arrows.markers[0];
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
            arrow_marker.color = color;  // x - red
            arrow_marker.points.resize(2);
            arrow_marker.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitX() * scale)
            );
            arrow_marker.scale.x = diameter * 0.1;
            arrow_marker.scale.y = diameter * 0.2;
            arrow_marker.scale.z = 0;

            // y axis
            visualization_msgs::msg::Marker arrow_marker_y = axes_arrows.markers[1];
            arrow_marker_y.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_y.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_y;
            color_y.r = 0.0; color_y.g = 1.0; color_y.b = 0.0; color_y.a = 1.0;
            arrow_marker_y.color = color_y;  // y - green
            arrow_marker_y.points.resize(2);
            arrow_marker_y.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_y.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitY() * scale)
            );
            arrow_marker_y.scale.x = diameter * 0.1;
            arrow_marker_y.scale.y = diameter * 0.2;
            arrow_marker_y.scale.z = 0;

            // z axis
            visualization_msgs::msg::Marker arrow_marker_z = axes_arrows.markers[2];
            arrow_marker_z.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_z.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_z;
            color_z.r = 0.0; color_z.g = 0.0; color_z.b = 1.0; color_z.a = 1.0;
            arrow_marker_z.color = color_z;  // z - blue
            arrow_marker_z.points.resize(2);
            arrow_marker_z.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_z.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitZ() * scale)
            );
            arrow_marker_z.scale.x = diameter * 0.1;
            arrow_marker_z.scale.y = diameter * 0.2;
            arrow_marker_z.scale.z = 0;

            // append to marker array
            for (size_t j = 0; j < axes_arrows.markers.size(); ++j) {
                axes_arrows.markers[j].header.frame_id = frame_id;
                axes_arrows.markers[j].ns = "pose";
                marker_array->markers.push_back(axes_arrows.markers[j]);
            }
        }
        last_position = point.position_W;
        geometry_msgs::msg::Point last_position_msg;
        last_position_msg = tf2::toMsg(last_position);
        line_strip.points.push_back(last_position_msg);
    }
    marker_array->markers.push_back(line_strip);

    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = node_->now();
    for (size_t i = 0; i < marker_array->markers.size(); ++i) {
        marker_array->markers[i].header = header;
        marker_array->markers[i].id = i;
        marker_array->markers[i].lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array->markers[i].action = visualization_msgs::msg::Marker::ADD;
    }
}
