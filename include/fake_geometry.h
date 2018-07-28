//
// Created by wangl on 7/28/2018.
//

#ifndef JNI_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H
#define JNI_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H

namespace fake_geometry_msgs {
    struct Vector3 {
        double x;
        double y;
        double z;
    };
    class Twist {
    public:
        Vector3 linear;
        Vector3 angular;
    };
}

#endif //JNI_TEB_LOCAL_PLANNER_FAKEGEOMETRY_H