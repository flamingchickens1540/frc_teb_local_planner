// Liam Wang
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/optimal_planner.h>
#include <fake_geometry.h>
#include "teb_jni_interface.h"
#include "include/teb_local_planner/pose_se2.h"
#include "teb-java-example/headers/TebInterface.h"
#include <chrono>  // for high_resolution_clock

teb_local_planner::PoseSE2 poseSE2FromJobject(JNIEnv *env, const jobject &obj) {
    jclass PoseSE2Class = env->GetObjectClass(obj);
    teb_local_planner::PoseSE2 pose(
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "x", "D")),
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "y", "D")),
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "theta", "D"))
    );
    return pose;
}

void saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) {
    // Limit translational velocity for forward driving
    if (vx > max_vel_x)
        vx = max_vel_x;

    // limit strafing velocity
    if (vy > max_vel_y)
        vy = max_vel_y;
    else if (vy < -max_vel_y)
        vy = -max_vel_y;

    // Limit angular velocity
    if (omega > max_vel_theta)
        omega = max_vel_theta;
    else if (omega < -max_vel_theta)
        omega = -max_vel_theta;

    // Limit backwards velocity
    if (max_vel_x_backwards <= 0) {
        std::cout << "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving." << std::endl;
    } else if (vx < -max_vel_x_backwards)
        vx = -max_vel_x_backwards;
}

static teb_local_planner::PlannerInterfacePtr planner_;
static teb_local_planner::TebConfig cfg_;
static teb_local_planner::TebVisualizationPtr visual_;
static teb_local_planner::ObstContainer obstacles_{};
static teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
static teb_local_planner::ViaPointContainer via_points_{};

JNIEXPORT void JNICALL Java_TebInterface_initialize(JNIEnv *, jclass, jobject) {

    planner_ = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, visual_, &via_points_));
}

JNIEXPORT void JNICALL Java_TebInterface_updateConfig(JNIEnv *env, jclass, jobject jconf) {
    jclass TebConfigCLass = env->GetObjectClass(jconf);

    cfg_.robot.max_vel_x =              env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "max_vel_x", "D"));
    cfg_.robot.max_vel_x_backwards =    env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "max_vel_x_backwards", "D"));
    cfg_.robot.max_vel_y =              env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "max_vel_y", "D"));
    cfg_.robot.max_vel_theta =          env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "max_vel_theta", "D"));

    cfg_.robot.acc_lim_x =              env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "acc_lim_x", "D"));
    cfg_.robot.acc_lim_y =              env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "acc_lim_y", "D"));
    cfg_.robot.acc_lim_theta =          env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "acc_lim_theta", "D"));

    cfg_.robot.min_turning_radius =     env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "min_turning_radius", "D"));
    cfg_.robot.wheelbase =              env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "wheelbase", "D"));

    cfg_.goal_tolerance.yaw_goal_tolerance =    env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "yaw_goal_tolerance", "D"));
    cfg_.goal_tolerance.xy_goal_tolerance =     env->GetDoubleField(jconf, env->GetFieldID(TebConfigCLass, "xy_goal_tolerance", "D"));
}

JNIEXPORT jobject JNICALL
Java_TebInterface_plan(JNIEnv *env, jclass, jobject jstart, jobject jgoal, jobject jstart_vel, jboolean free_goal_vel) {
    auto startPose = poseSE2FromJobject(env, jstart);
    auto goalPose = poseSE2FromJobject(env, jgoal);
    auto startTwist = poseSE2FromJobject(env, jstart_vel);

    fake_geometry_msgs::Twist start_vel{};
    start_vel.linear.x = startTwist.x();
    start_vel.linear.y = startTwist.y();
    start_vel.angular.z = startTwist.theta();


    auto start = std::chrono::high_resolution_clock::now();
    bool success = planner_->plan(startPose, goalPose, &start_vel, free_goal_vel); // Calculate the plan
    if (!success) {
        planner_->clearPlanner(); // force reinitialization for next time
        std::cout << "teb_local_planner was not able to obtain a local plan for the current setting." << std::endl;
    }

    // Debugging
//    std::cout << "Start Pose: " << startPose << std::endl;
//    std::cout << "End Pose: " << goalPose << std::endl;
//    std::cout << "Start Vel: " << startTwist << std::endl;
//    std::cout << "Free Goal Vel: " << (static_cast<bool>(free_goal_vel) ? "True" : "False") << std::endl;

    double x,y,theta;
    planner_->getVelocityCommand(x, y, theta);
    auto finish = std::chrono::high_resolution_clock::now();

    saturateVelocity(x, y, theta,
            cfg_.robot.max_vel_x,
            cfg_.robot.max_vel_y,
            cfg_.robot.max_vel_theta,
            cfg_.robot.max_vel_x_backwards);

    jclass poseSE2Class = env->FindClass("LPoseSE2;");
    jmethodID poseSE2Constructor = env->GetMethodID(poseSE2Class, "<init>", "(DDD)V");
    jobject cmd_vel = env->NewObject(poseSE2Class, poseSE2Constructor, x, y, theta);
    return cmd_vel;
}
