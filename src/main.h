//
// Created by wangl on 11/8/18.
//

#ifndef FRC_TEB_LOCAL_PLANNER_MAIN_H
#define FRC_TEB_LOCAL_PLANNER_MAIN_H

#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/pose_se2.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "networktables/NetworkTable.h"
#include "tables/ITableListener.h"
#include "wpi/StringRef.h"

#include <fake_geometry.h>
#include "practical_socket.h"

static teb_local_planner::ObstContainer obstacles{};
static teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
static teb_local_planner::TebVisualizationPtr visual;
static teb_local_planner::ViaPointContainer via_points{};

static std::mutex cfg_mtx;
static teb_local_planner::TebConfig teb_cfg;
static fake_geometry_msgs::Pose goal_pose;
static bool newCfgReceived;

static std::mutex pose_twist_goal_mtx;
static fake_geometry_msgs::Pose current_pose;
static fake_geometry_msgs::Twist current_twist;
static bool newPoseTwistReceived;

static long long counter = 0;

void saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta,
                      double max_vel_x_backwards) {
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
        std::cout
                << "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalizing backwards driving."
                << std::endl;
    } else if (vx < -max_vel_x_backwards)
        vx = -max_vel_x_backwards;
}

double reverseDouble(const double inDouble)
{
    double retVal;
    auto *doubleToConvert = ( char* ) & inDouble;
    auto *returnDouble = ( char* ) & retVal;

    // swap the bytes into a temporary buffer
    returnDouble[0] = doubleToConvert[7];
    returnDouble[1] = doubleToConvert[6];
    returnDouble[2] = doubleToConvert[5];
    returnDouble[3] = doubleToConvert[4];
    returnDouble[4] = doubleToConvert[3];
    returnDouble[5] = doubleToConvert[2];
    returnDouble[6] = doubleToConvert[1];
    returnDouble[7] = doubleToConvert[0];

    return retVal;
}

double constrainAngle(double x) {
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

class UDPRunnable {
private:
    const unsigned short localPortIn = 5800;
    UDPSocket* socket;
public:
    UDPRunnable();
    void run();
};

class NTListener : public ITableListener {
private:
    std::map<std::string, double*> ntDoubleKeys {
            {"--------max_vel_x",                       &teb_cfg.robot.max_vel_x},
            {"--------max_vel_x_backwards",             &teb_cfg.robot.max_vel_x_backwards},
            {"--------acc_lim_x",                       &teb_cfg.robot.acc_lim_x},
            {"--------max_vel_theta",                   &teb_cfg.robot.max_vel_theta},
            {"--------acc_lim_theta",                   &teb_cfg.robot.acc_lim_theta},
//            {"weight_optimaltime", &teb_cfg.optim.weight_optimaltime},
//            {"--------goal_position_x",                 &goal_pose.position.x},
//            {"--------goal_position_y",                 &goal_pose.position.y},
//            {"--------goal_orientation_z",              &goal_pose.orientation.z},
            {"--------min_turning_radius",              &teb_cfg.robot.min_turning_radius},
            {"--------weight_kinematics_forward_drive", &teb_cfg.optim.weight_kinematics_forward_drive}
    };
    std::map<std::string, bool*> ntBoolKeys {
            {"--------free_goal_vel", &teb_cfg.goal_tolerance.free_goal_vel}
    };
public:
    NTListener(shared_ptr<NetworkTable> source);
    void ValueChanged(ITable *source, wpi::StringRef testKey, std::shared_ptr<nt::Value> value, bool isNew) override;
};

class PlannerRunnable {
private:
    // UDP
    const string serverAddress = "10.15.40.2";
    const unsigned short localPortOut = 5801;
    UDPSocket* sendSocket;

    // TEB
    teb_local_planner::PlannerInterfacePtr planner;
    teb_local_planner::TebConfig* temp_teb_cfg;

    fake_geometry_msgs::Twist plan(
            teb_local_planner::PoseSE2 start_pose,
            teb_local_planner::PoseSE2 goal_pose,
            fake_geometry_msgs::Twist start_twist,
            bool free_goal_vel);
public:
    PlannerRunnable();
    void run();
};

#endif //FRC_TEB_LOCAL_PLANNER_MAIN_H
