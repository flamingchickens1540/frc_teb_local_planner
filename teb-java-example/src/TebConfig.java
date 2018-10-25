public class TebConfig {

    // ROBOT
    public double max_vel_x = 0.4; //!< Maximum translational velocity of the robot
    public double max_vel_x_backwards = 0.2; //!< Maximum translational velocity of the robot for driving backwards
    public double max_vel_y = 0.0; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    public double max_vel_theta = 0.3; //!< Maximum angular velocity of the robot
    public double acc_lim_x = 0.5; //!< Maximum translational acceleration of the robot
    public double acc_lim_y = 0.5; //!< Maximum strafing acceleration of the robot
    public double acc_lim_theta = 0.5; //!< Maximum angular acceleration of the robot
    public double min_turning_radius = 0; //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    public double wheelbase = 1.0; //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for back-wheeled robots!

    // GOAL TOLERANCE

    public double yaw_goal_tolerance = 0.2; //!< Allowed final orientation error
    public double xy_goal_tolerance = 0.2; //!< Allowed final euclidean distance to the goal position
}
