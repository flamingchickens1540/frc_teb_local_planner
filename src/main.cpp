//
// Created by wangl on 11/8/18.
//

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
                current_pose.orientation.z = reverseDouble(doubleBuffer[2]);
                current_twist.linear.x = reverseDouble(doubleBuffer[3]);
                current_twist.angular.z = reverseDouble(doubleBuffer[4]);
                pose_twist_mtx.unlock();
            } else {
                cerr << "Unable to receive data!" << endl;
            }
        }
    } catch (SocketException &e) {
        cerr << e.what() << endl;
        exit(1);
    }
}

void NTRunnable::ValueChanged(ITable *source, llvm::StringRef testKey, shared_ptr<nt::Value> value, bool isNew) {
    for( auto const& symbol : ntKeys ) {
        if (testKey.equals(symbol.first)) {
            *symbol.second = value->GetDouble();
        }
    }
};

PlannerRunnable::PlannerRunnable() {
    try {
        sendSocket = new UDPSocket(localPortOut);
        cout << "Successfully opened port " << localPortOut << "." << endl;
    } catch (SocketException &e) {
        cerr << "Error opening port " << localPortOut << ": " << e.what() << endl;
    }

    temp_teb_cfg = new teb_local_planner::TebConfig();
    teb_local_planner::ObstContainer obstacles{};
    teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
    teb_local_planner::TebVisualizationPtr visual;
    teb_local_planner::ViaPointContainer via_points{};

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
    // Copy input data into temporary variables so we don't hold up the other threads
    pose_twist_mtx.lock();
    teb_local_planner::PoseSE2 temp_current_pose {
        current_pose.position.x,
        current_pose.position.y,
        current_pose.orientation.z
    };
    fake_geometry_msgs::Twist start_twist {
        current_twist.linear.x,
        current_twist.linear.y,
        current_twist.angular.z
    };
    pose_twist_mtx.unlock();

    cfg_goal_mtx.lock();
    temp_teb_cfg = new teb_local_planner::TebConfig(teb_cfg); // Copy teb config
    teb_local_planner::PoseSE2 temp_goal_pose {
        goal_pose.position.x,
        goal_pose.position.y,
        goal_pose.orientation.z
    };
    cfg_goal_mtx.unlock();

    // Actually calculate plan
    fake_geometry_msgs::Twist cmd_vel = plan(temp_current_pose, temp_goal_pose, start_twist, false);

    double cmd_vel_packet[3];
    cmd_vel_packet[0] = 0;  // TODO: Add timestamp (or at least incrementing counter)
    cmd_vel_packet[1] = cmd_vel.linear.x;
    cmd_vel_packet[2] = -cmd_vel.angular.z;

    sendSocket->sendTo(cmd_vel_packet, sizeof(double)*3, serverAddress, localPortOut);
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

    fake_geometry_msgs::Twist cmd_vel{};
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.angular.z = theta;
    return cmd_vel;
}

int main() {
    thread UDPThread (&UDPRunnable::run);
}