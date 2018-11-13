//
// Created by wangl on 7/28/2018.
//

#ifndef FRC_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H
#define FRC_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H

namespace fake_geometry_msgs {
    struct Vector3 {
        double x;
        double y;
        double z;
    };
    struct Quaternion {
        double w;
        double x;
        double y;
        double z;
    };
    class Twist {
    public:
        Vector3 linear;
        Vector3 angular;
    };
    class Pose {
    public:
        Vector3 position;
        Quaternion orientation;
    };
}

#endif //FRC_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H
