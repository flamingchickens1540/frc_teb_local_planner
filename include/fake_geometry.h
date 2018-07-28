//
// Created by wangl on 7/28/2018.
//

#ifndef JNITESTS_FAKEGEOMETRY_H
#define JNITESTS_FAKEGEOMETRY_H

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

#endif //JNITESTS_FAKEGEOMETRY_H
