//
// Created by wangl on 11/8/18.
//

#include "main.h"
#include <iostream>

#include "networktables/NetworkTable.h"
#include "tables/ITableListener.h"

// Liam Wang
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/optimal_planner.h>
#include <fake_geometry.h>
#include "teb_local_planner/pose_se2.h"
#include <chrono>  // for high_resolution_clock
using namespace std::chrono;

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

void initialize() {

    planner_ = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, visual_, &via_points_));
}

void updateConfig() {
    cfg_.robot.max_vel_x =              1;
    cfg_.robot.max_vel_x_backwards =    1;
    cfg_.robot.max_vel_y =              0.0;
    cfg_.robot.max_vel_theta =          3;

    cfg_.robot.acc_lim_x =              0.5;
    cfg_.robot.acc_lim_y =              0.5;
    cfg_.robot.acc_lim_theta =          3;

    cfg_.robot.min_turning_radius =     0.0;
    cfg_.robot.wheelbase =              1.0; // ONLY for car-like robots

    cfg_.goal_tolerance.yaw_goal_tolerance = 0.2;
    cfg_.goal_tolerance.xy_goal_tolerance =  0.2;
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

    double x,y,theta;
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
    double xpos = 0;
    double ypos = 0;
    double thetapos = 0;
    double xvel = 0;
    double thetavel = 0;
    double goalx = 0;
    double goaly = 0;
    double goaltheta = 0;

    std::chrono::_V2::system_clock::time_point start;

    int count = 0;

    double constrainAngle(double x){
        x = fmod(x + 180,360);
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
            xpos = value->GetDouble();
        } else if (key.equals("pose-position-y")) {
            ypos = value->GetDouble();
        } else if (key.equals("pose-orientation-z")) {
            thetapos = constrainAngle(value->GetDouble()) * 3.141592/180;
        } else if (key.equals("twist-linear-x")) {
            xvel = value->GetDouble();
        } else if (key.equals("twist-angular-z")) {
            thetavel = value->GetDouble();
        } else if (key.equals("goal-position-x")) {
            goalx = value->GetDouble();
        } else if (key.equals("goal-position-y")) {
            goaly = value->GetDouble();
        } else if (key.equals("goal-orientation-z")) {
            goaltheta = value->GetDouble();
        }
    }

    void Loop(std::shared_ptr<NetworkTable> &table) {
        while (true) {
            teb_local_planner::PoseSE2 start_pose{xpos, ypos, thetapos};
            teb_local_planner::PoseSE2 goal_pose{goalx, goaly, goaltheta};
            fake_geometry_msgs::Twist start_twist{xvel, 0, thetavel};
            fake_geometry_msgs::Twist cmd_vel = plan(start_pose, goal_pose, start_twist, false);

            table->PutNumber("cmd_vel-linear-x", cmd_vel.linear.x);
            table->PutNumber("cmd_vel-angular-z", -cmd_vel.angular.z);

            std::cout << "X: " << xpos << " Y: " << ypos << " Theta: " << thetapos << " XVel: " << xvel << " ThetaVel: " << thetavel << " cmd_vel_x " << cmd_vel.linear.x << " cmd_vel_theta " << cmd_vel.angular.z << std::endl;
        }
    };
};

int main() {
    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(1540);


    auto table = NetworkTable::GetTable("SmartDashboard");
    Derived dir;
    table->AddTableListener(&dir);
    updateConfig();
    initialize();
    dir.Loop(table);
}



