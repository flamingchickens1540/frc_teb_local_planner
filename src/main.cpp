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
        double doubleBuffer[5];
        while (true) {
            if (socket->recv(doubleBuffer, sizeof(double) * 5) == sizeof(double) * 5) {
                pose_twist_mtx.lock();
                current_pose.position.x = reverseDouble(doubleBuffer[0]);
                current_pose.position.y = reverseDouble(doubleBuffer[1]);
                current_pose.orientation.z = constrainAngle(reverseDouble(doubleBuffer[2])) * 3.141592 / 180;
                current_twist.linear.x = reverseDouble(doubleBuffer[3]);
                current_twist.angular.z = reverseDouble(doubleBuffer[4]);
                newPoseTwistReceived = true;
                pose_twist_mtx.unlock();
            } else {
                cerr << "Unable to receive data!" << endl;
                pose_twist_mtx.unlock();
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
    cfg_goal_mtx.lock();
    for (auto const &symbol : ntDoubleKeys) {
//        if (testKey.equals(symbol.first)) {
        if (testKey.substr(8).equals(wpi::StringRef(symbol.first).substr(8))) {
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
    cfg_goal_mtx.unlock();
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
                                                     visual,
                                                     &via_points
            )
    );

}

void PlannerRunnable::run() {
    while (true) {
        // Copy input data into temporary variables so we don't hold up the other threads
        pose_twist_mtx.lock();
        if (!newPoseTwistReceived) {
            pose_twist_mtx.unlock();
            continue;
        }
        teb_local_planner::PoseSE2 temp_current_pose{
                current_pose.position.x,
                current_pose.position.y,
                current_pose.orientation.z
        };
        fake_geometry_msgs::Twist start_twist{
                current_twist.linear.x,
                current_twist.linear.y,
                current_twist.angular.z
        };
        newPoseTwistReceived = false;
        pose_twist_mtx.unlock();

        cfg_goal_mtx.lock();
        teb_local_planner::PoseSE2 temp_goal_pose{
                goal_pose.position.x,
                goal_pose.position.y,
                goal_pose.orientation.z
        };
        if (newCfgReceived) {
            temp_teb_cfg = new teb_local_planner::TebConfig(teb_cfg); // Copy teb config
            planner = teb_local_planner::PlannerInterfacePtr(
                    new teb_local_planner::TebOptimalPlanner(*temp_teb_cfg,
                                                             &obstacles,
                                                             robot_model,
                                                             visual,
                                                             &via_points
                    )
            );
            cout << "Updated goal and cfg!" << endl;
            newCfgReceived = false;
        }
        cfg_goal_mtx.unlock();

        // Actually calculate plan
        fake_geometry_msgs::Twist cmd_vel = plan(temp_current_pose, temp_goal_pose, start_twist, teb_cfg.goal_tolerance.free_goal_vel);

        double cmd_vel_packet[2];
//        cmd_vel_packet[0] = 0;  // TODO: Add timestamp (or at least incrementing counter)
        cmd_vel_packet[0] = cmd_vel.linear.x;
        cmd_vel_packet[1] = -cmd_vel.angular.z;

        sendSocket->sendTo(cmd_vel_packet, sizeof(double) * 2, serverAddress, localPortOut);
    }
}

fake_geometry_msgs::Twist PlannerRunnable::plan(
        teb_local_planner::PoseSE2 start_pose,
        teb_local_planner::PoseSE2 goal_pose,
        fake_geometry_msgs::Twist start_twist,
        bool free_goal_vel) {

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

//    planner->

//    cout
//         << " Free_Goal_Vel: " << temp_teb_cfg->goal_tolerance.free_goal_vel
//         << " MVX: " << temp_teb_cfg->robot.max_vel_x
//         << " MVXB: " << temp_teb_cfg->robot.max_vel_x_backwards
//         << " AX: " << temp_teb_cfg->robot.acc_lim_x
//         << " MVT: " << temp_teb_cfg->robot.max_vel_theta
//         << " AT: " << temp_teb_cfg->robot.acc_lim_theta
//            << endl;
// cout
//         << " Linear X: " << start_pose.x()
//         << " Linear X: " << start_twist.angular.x
//         << " MVXB: " << temp_teb_cfg->robot.max_vel_x_backwards
//         << " AX: " << temp_teb_cfg->robot.acc_lim_x
//         << " MVT: " << temp_teb_cfg->robot.max_vel_theta
//         << " AT: " << temp_teb_cfg->robot.acc_lim_theta
//            << endl;
    fake_geometry_msgs::Twist cmd_vel{};
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.angular.z = theta;
    return cmd_vel;
}

int main() {
    thread udpThread(&UDPRunnable::run, UDPRunnable());
    thread plannerThread(&PlannerRunnable::run, PlannerRunnable());

//    auto ntinst = nt::NetworkTableInstance::GetDefault();
//    ntinst.StartClientTeam(1540);
    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(1540);

//    auto table = ntinst.GetTable("SmartDashboard");
    shared_ptr<NetworkTable> table = NetworkTable::GetTable("SmartDashboard");
    NTListener ntListener(table);
    table->AddTableListener(&ntListener);

    udpThread.join();
    plannerThread.join();
}