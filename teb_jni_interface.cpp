// Liam Wang
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/optimal_planner.h>
#include <fake_geometry.h>
#include "teb_jni_interface.h"
#include "include/teb_local_planner/pose_se2.h"

#include "java/TebInterface.h"

teb_local_planner::PoseSE2 poseSE2FromJobject(JNIEnv *env, const jobject &obj) {
    jclass PoseSE2Class = env->GetObjectClass(obj);
    teb_local_planner::PoseSE2 pose(
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "x", "D")),
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "y", "D")),
            env->GetDoubleField(obj, env->GetFieldID(PoseSE2Class, "theta", "D"))
    );
    return pose;
}

JNIEXPORT jobject JNICALL
Java_TebInterface_plan(JNIEnv *env, jclass, jobject jstart, jobject jgoal, jobject jstart_vel, jboolean free_goal_vel) {
    auto start = poseSE2FromJobject(env, jstart);
    std::cout << "Start Pose: " << start << std::endl;
    auto goal = poseSE2FromJobject(env, jgoal);
    std::cout << "End Pose: " << goal << std::endl;
    auto start_vel_pose = poseSE2FromJobject(env, jstart_vel);
    std::cout << "Start Vel: " << start_vel_pose << std::endl;
    std::cout << "Free Goal Vel: " << (static_cast<bool>(free_goal_vel) ? "True" : "False") << std::endl;

    fake_geometry_msgs::Twist start_vel;
    start_vel.linear.x = start_vel_pose.x();
    start_vel.linear.y = start_vel_pose.y();
    start_vel.angular.z = start_vel_pose.theta();

    teb_local_planner::TebConfig cfg_{};
    teb_local_planner::ObstContainer obstacles_{};
    teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
    teb_local_planner::ViaPointContainer via_points_{};
    teb_local_planner::TebVisualizationPtr visual_;

    auto planner_ = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, visual_, &via_points_));
    bool success = planner_->plan(start, goal, &start_vel, free_goal_vel);

    double x,y,theta;
    planner_->getVelocityCommand(x, y, theta);
    jclass poseSE2Class = env->FindClass("LPoseSE2;");
    jmethodID poseSE2Constructor = env->GetMethodID(poseSE2Class, "<init>", "(DDD)V");
    jobject cmd_vel = env->NewObject(poseSE2Class, poseSE2Constructor, x, y, theta);
    return cmd_vel;
}
