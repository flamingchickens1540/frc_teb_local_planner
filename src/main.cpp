//
// Created by wangl on 11/8/18.
//

#include <networktables/NetworkTableInstance.h>
#include "main.h"

using namespace std::chrono;

UDPRunnable::UDPRunnable() {
    try {
        socket = new UDPSocket(localPortIn);
        cout << "Successfully opened port " << localPortIn << "." << endl;
    } catch (SocketException &e) {
        cerr << "Error opening port " << localPortIn << ": " << e.what() << endl;
    }
}

void UDPRunnable::run() {
    try {
        double doubleBuffer[10];
        while (true) {
            if (socket->recv(doubleBuffer, sizeof(double) * 10) == sizeof(double) * 10) {
                pose_twist_goal_mtx.lock();
                current_pose.position.x = reverseDouble(doubleBuffer[0]);
                current_pose.position.y = reverseDouble(doubleBuffer[1]);
                current_pose.orientation.z = constrainAngle(reverseDouble(doubleBuffer[2]));
                current_twist.linear.x = reverseDouble(doubleBuffer[3]);
                current_twist.angular.z = reverseDouble(doubleBuffer[4]);
                goal_pose.position.x = reverseDouble(doubleBuffer[5]);
                goal_pose.position.y = reverseDouble(doubleBuffer[6]);
                goal_pose.orientation.z = constrainAngle(reverseDouble(doubleBuffer[7]));
                via_point[0] = reverseDouble(doubleBuffer[8]);
                via_point[1] = reverseDouble(doubleBuffer[9]);
                newPoseTwistReceived = true;
                pose_twist_goal_mtx.unlock();
            } else {
                cerr << "UDP ERROR: Unable to receive data!" << endl;
            }
        }
    } catch (SocketException &e) {
        cerr << e.what() << endl;
        exit(1);
    }
}

NTListener::NTListener(shared_ptr<NetworkTable> source) {
    // Defaults
    teb_cfg.robot.max_vel_theta = 3.0;
    teb_cfg.robot.acc_lim_theta = 3.0;

    teb_cfg.robot.max_vel_x = 1.5;
    teb_cfg.robot.max_vel_x_backwards = 1.4;
    teb_cfg.robot.acc_lim_x = 0.7;

    teb_cfg.robot.min_turning_radius = 0.0;

    teb_cfg.optim.weight_optimaltime = 4.0;

    teb_cfg.goal_tolerance.free_goal_vel = false;

    teb_cfg.optim.weight_kinematics_forward_drive = 1.0;

    for (auto const &symbol : ntDoubleKeys) {
        source->PutNumber(symbol.first, *symbol.second);
    }
    for (auto const &symbol : ntBoolKeys) {
        source->PutBoolean(symbol.first, *symbol.second);
    }
}

void NTListener::ValueChanged(ITable *source, wpi::StringRef testKey, shared_ptr<nt::Value> value, bool isNew) {
    cfg_mtx.lock();
    if (testKey.equals("TEBReset") && value->GetBoolean()) {
        newCfgReceived = true;
        shared_ptr<NetworkTable> table = NetworkTable::GetTable(NT_NAME);
        table->PutBoolean("TEBReset", false);
    }
    for (auto const &symbol : ntDoubleKeys) {
        if (testKey.equals(symbol.first)) {
            *symbol.second = value->GetDouble();
            newCfgReceived = true;
        }
    }
    for (auto const &symbol : ntBoolKeys) {
        if (testKey.equals(symbol.first)) {
            *symbol.second = value->GetBoolean();
            newCfgReceived = true;
        }
    }
    cfg_mtx.unlock();
};

PlannerRunnable::PlannerRunnable() {
    newPoseTwistReceived = false;
    newCfgReceived = true;
    try {
        sendSocket = new UDPSocket(localPortOut);
        cout << "Successfully opened port " << localPortOut << "." << endl;
    } catch (SocketException &e) {
        cerr << "Error opening port " << localPortOut << ": " << e.what() << endl;
    }

    temp_teb_cfg = new teb_local_planner::TebConfig();

    planner = teb_local_planner::PlannerInterfacePtr(
            new teb_local_planner::TebOptimalPlanner(*temp_teb_cfg,
                                                     &obstacles,
                                                     robot_model,
                                                     nullptr,
                                                     nullptr
            )
    );

}

void PlannerRunnable::run() {
    while (true) {
        // Copy input data into temporary variables so we don't hold up the other threads
        pose_twist_goal_mtx.lock();
        if (!newPoseTwistReceived) {
            pose_twist_goal_mtx.unlock();
            continue;
        }
        teb_local_planner::PoseSE2 temp_current_pose{
                current_pose.position.x,
                current_pose.position.y,
                current_pose.orientation.z
        };

        auto start_twist = new geometry_msgs::Twist();
        start_twist->linear.x = current_twist.linear.x;
        start_twist->linear.y = current_twist.linear.y;
        start_twist->angular.z = current_twist.angular.z;

        teb_local_planner::PoseSE2 temp_goal_pose{
                goal_pose.position.x,
                goal_pose.position.y,
                goal_pose.orientation.z
        };

        Eigen::Vector2d temp_via_point(
                via_point[0],
                via_point[1]
        );

        newPoseTwistReceived = false;
        pose_twist_goal_mtx.unlock();

        cfg_mtx.lock();
        if (newCfgReceived) { // TODO: Does the planner really need to be created again if the config is updated?
            temp_teb_cfg = new teb_local_planner::TebConfig(teb_cfg); // Copy teb config
            planner = teb_local_planner::PlannerInterfacePtr(
                    new teb_local_planner::TebOptimalPlanner(*temp_teb_cfg,
                                                             &obstacles,
                                                             robot_model,
                                                             nullptr,
                                                             nullptr
                    )
            );
            cout << "INFO: Updated TEB configuration!" << endl;
            newCfgReceived = false;
        }
        cfg_mtx.unlock();

        // Actually calculate plan
        geometry_msgs::Twist cmd_vel = plan(temp_current_pose, temp_goal_pose, *start_twist, temp_via_point, teb_cfg.goal_tolerance.free_goal_vel);

        double cmd_vel_packet[2];
//        cmd_vel_packet[0] = 0;  // TODO: Add timestamp (or at least incrementing counter)
        cmd_vel_packet[0] = cmd_vel.linear.x;
        cmd_vel_packet[1] = cmd_vel.angular.z;

        sendSocket->sendTo(cmd_vel_packet, sizeof(double) * 2, serverAddress, localPortOut);
    }
}

geometry_msgs::Twist PlannerRunnable::plan(
        teb_local_planner::PoseSE2 start_pose,
        teb_local_planner::PoseSE2 goal_pose,
        geometry_msgs::Twist start_twist,
        Eigen::Vector2d via_point,
        bool free_goal_vel) {



    teb_local_planner::ViaPointContainer via_points;
    via_points.push_back(via_point);
    via_points.push_back(via_point);
    boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>(planner)->setViaPoints(&via_points);
    auto start = chrono::high_resolution_clock::now();
    bool success = planner->plan(start_pose, goal_pose, &start_twist, free_goal_vel); // Calculate the plan
    if (!success) {
        planner->clearPlanner(); // force reinitialization for next time
        cout << "teb_local_planner was not able to obtain a local plan for the current setting." << endl;
    }

    double x, y, theta;
    planner->getVelocityCommand(x, y, theta);
    auto finish = chrono::high_resolution_clock::now();

    saturateVelocity(x, y, theta,
                     temp_teb_cfg->robot.max_vel_x,
                     temp_teb_cfg->robot.max_vel_y,
                     temp_teb_cfg->robot.max_vel_theta,
                     temp_teb_cfg->robot.max_vel_x_backwards);

    geometry_msgs::Twist cmd_vel{};
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.angular.z = theta;
    return cmd_vel;
}

int main() {
    thread udpThread(&UDPRunnable::run, UDPRunnable());
    thread plannerThread(&PlannerRunnable::run, PlannerRunnable());

    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(TEAM_NUMBER);

    shared_ptr<NetworkTable> table = NetworkTable::GetTable(NT_NAME);
    NTListener ntListener(table);
    table->AddTableListener(&ntListener);

    udpThread.join();
    plannerThread.join();
}