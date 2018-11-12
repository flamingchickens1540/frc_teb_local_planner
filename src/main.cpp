//
// Created by wangl on 11/8/18.
//

#include "main.h"
#include <iostream>

#include <boost/endian/buffers.hpp>  // see Synopsis below
#include "networktables/NetworkTable.h"
#include "tables/ITableListener.h"

// Liam Wang
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/optimal_planner.h>
#include <fake_geometry.h>
#include "teb_local_planner/pose_se2.h"
#include <chrono>  // for high_resolution_clock

#include "PracticalSocket.h"

using namespace std::chrono;

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
                << "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving."
                << std::endl;
    } else if (vx < -max_vel_x_backwards)
        vx = -max_vel_x_backwards;
}

static teb_local_planner::PlannerInterfacePtr planner_;
static teb_local_planner::TebConfig cfg_;
static teb_local_planner::TebVisualizationPtr visual_;
static teb_local_planner::ObstContainer obstacles_{};
static teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
static teb_local_planner::ViaPointContainer via_points_{};

void initialize() {

    planner_ = teb_local_planner::PlannerInterfacePtr(
            new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, visual_, &via_points_));
}

void updateConfig() {
    cfg_.robot.max_vel_x = 1;
    cfg_.robot.max_vel_x_backwards = 1;
    cfg_.robot.max_vel_y = 0.0;
    cfg_.robot.max_vel_theta = 3;

    cfg_.robot.acc_lim_x = 0.5;
    cfg_.robot.acc_lim_y = 0.5;
    cfg_.robot.acc_lim_theta = 3;

    cfg_.robot.min_turning_radius = 0.0;
    cfg_.robot.wheelbase = 1.0; // ONLY for car-like robots

    cfg_.goal_tolerance.yaw_goal_tolerance = 0.2;
    cfg_.goal_tolerance.xy_goal_tolerance = 0.2;
}

fake_geometry_msgs::Twist plan(
        teb_local_planner::PoseSE2 start_pose,
        teb_local_planner::PoseSE2 goal_pose,
        fake_geometry_msgs::Twist start_twist,
        bool free_goal_vel) {

    auto start = std::chrono::high_resolution_clock::now();
    bool success = planner_->plan(start_pose, goal_pose, &start_twist, free_goal_vel); // Calculate the plan
    if (!success) {
        planner_->clearPlanner(); // force reinitialization for next time
        std::cout << "teb_local_planner was not able to obtain a local plan for the current setting." << std::endl;
    }

    double x, y, theta;
    planner_->getVelocityCommand(x, y, theta);
    auto finish = std::chrono::high_resolution_clock::now();

    saturateVelocity(x, y, theta,
                     cfg_.robot.max_vel_x,
                     cfg_.robot.max_vel_y,
                     cfg_.robot.max_vel_theta,
                     cfg_.robot.max_vel_x_backwards);

    fake_geometry_msgs::Twist cmd_vel{};
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.angular.z = theta;
    return cmd_vel;
}


class Derived : public ITableListener {
private:
    double pose_position_x = 0;
    double pose_position_y = 0;
    double pose_orientation_z = 0;
    double twist_linear_x = 0;
    double twist_angular_z = 0;
    double goal_position_x = 0;
    double goal_position_y = 0;
    double goal_orientation_z = 0;

    std::chrono::_V2::system_clock::time_point start;

    int count = 0;

    double constrainAngle(double x) {
        x = fmod(x + 180, 360);
        if (x < 0)
            x += 360;
        return x - 180;
    }

public:
    void ValueChanged(ITable *source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool isNew) {
//        if (key.equals("timertest")) {
//            source->PutBoolean("timertest", true);
//            count++;
//            if (count%100==0) {
//                auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
//                std::cout << "Count: " << count << " Time: " << duration.count() << std::endl;
//                start = high_resolution_clock::now();
//            }
//        }
        if (key.equals("pose-position-x")) {
            pose_position_x = value->GetDouble();
        } else if (key.equals("pose-position-y")) {
            pose_position_y = value->GetDouble();
        } else if (key.equals("pose-orientation-z")) {
            pose_orientation_z = constrainAngle(value->GetDouble()) * 3.141592 / 180;
        } else if (key.equals("twist-linear-x")) {
            twist_linear_x = value->GetDouble();
        } else if (key.equals("twist-angular-z")) {
            twist_angular_z = value->GetDouble();
        } else if (key.equals("goal-position-x")) {
            goal_position_x = value->GetDouble();
        } else if (key.equals("goal-position-y")) {
            goal_position_y = value->GetDouble();
        } else if (key.equals("goal-orientation-z")) {
            goal_orientation_z = value->GetDouble();
        }
    }

    void Loop(std::shared_ptr<NetworkTable> &table) {
        while (true) {
            teb_local_planner::PoseSE2 start_pose{pose_position_x, pose_position_y, pose_orientation_z};
            teb_local_planner::PoseSE2 goal_pose{goal_position_x, goal_position_y, goal_orientation_z};
            fake_geometry_msgs::Twist start_twist{twist_linear_x, 0, twist_angular_z};
            fake_geometry_msgs::Twist cmd_vel = plan(start_pose, goal_pose, start_twist, false);

            table->PutNumber("cmd_vel-linear-x", cmd_vel.linear.x);
            table->PutNumber("cmd_vel-angular-z", -cmd_vel.angular.z);

            std::cout << "X: " << pose_position_x
                      << " Y: " << pose_position_y
                      << " Theta: " << pose_orientation_z
                      << " XVel: " << twist_linear_x
                      << " ThetaVel: " << twist_angular_z
                      << " cmd_vel_x " << cmd_vel.linear.x
                      << " cmd_vel_theta " << cmd_vel.angular.z << std::endl;
        }
    };
};

#define swapByte(a, b, t) t=*(a); *(a)=*(b); *(b)=t

void byteSwapDouble(double *ptrValue)
{
    char temp;
    swapByte(ptrValue, ptrValue+7, temp);
    swapByte(ptrValue+1, ptrValue+6, temp);
    swapByte(ptrValue+2, ptrValue+5, temp);
    swapByte(ptrValue+3, ptrValue+4, temp);
}

union fiveDoubles {
    double d[5];
    char bytes[sizeof(std::double_t) * 5];
};

double reverseDouble(const double inDouble)
{
    double retVal;
    char *doubleToConvert = ( char* ) & inDouble;
    char *returnDOuble = ( char* ) & retVal;

    // swap the bytes into a temporary buffer
    returnDOuble[0] = doubleToConvert[7];
    returnDOuble[1] = doubleToConvert[6];
    returnDOuble[2] = doubleToConvert[5];
    returnDOuble[3] = doubleToConvert[4];
    returnDOuble[4] = doubleToConvert[3];
    returnDOuble[5] = doubleToConvert[2];
    returnDOuble[6] = doubleToConvert[1];
    returnDOuble[7] = doubleToConvert[0];

    return retVal;
}

int main() {
//    NetworkTable::SetClientMode();
//    NetworkTable::SetTeam(1540);


    updateConfig();
    initialize();


    string servAddress = "10.15.40.2";             // First arg: server address
    unsigned short echoServPort = Socket::resolveService("4445", "udp");
    unsigned short echoServPort2 = Socket::resolveService("9876", "udp");

    try {
        UDPSocket sock(4445);
        UDPSocket sendSock(9876);

        // Send the string to the server



        fiveDoubles doubleBuffer = {};
        while (true) {
//            std::cout << "Waiting..." << echoServPort << std::endl;

            if (sock.recvFrom(doubleBuffer.bytes, 8 * 5, servAddress, echoServPort) != 8 * 5) {
                cerr << "Unable to receive" << endl;
                exit(1);
            }

            std::cout << "X: " << reverseDouble(doubleBuffer.d[0])
                      << " Y: " << reverseDouble(doubleBuffer.d[1])
                      << " Theta: " << reverseDouble(doubleBuffer.d[2])
                      << " XVel: " << reverseDouble(doubleBuffer.d[3])
                      << " ThetaVel: " << reverseDouble(doubleBuffer.d[4]) << std::endl;

            teb_local_planner::PoseSE2 start_pose{reverseDouble(doubleBuffer.d[0]), reverseDouble(doubleBuffer.d[1]), constrainAngle(reverseDouble(doubleBuffer.d[2])) * 3.141592 / 180};
            teb_local_planner::PoseSE2 goal_pose{3, 2, 0};
            fake_geometry_msgs::Twist start_twist{reverseDouble(doubleBuffer.d[3]), 0, reverseDouble(doubleBuffer.d[4])};
            fake_geometry_msgs::Twist cmd_vel = plan(start_pose, goal_pose, start_twist, false);

            twoDoubles cmd_vel_packet;
            cmd_vel_packet.d[0] = cmd_vel.linear.x;
            cmd_vel_packet.d[1] = -cmd_vel.angular.z;

            sendSock.sendTo(cmd_vel_packet.bytes, 8*2, servAddress, echoServPort2);


        }
        // Destructor closes the socket

    } catch (SocketException &e) {
        cerr << e.what() << endl;
        exit(1);
    }
}



