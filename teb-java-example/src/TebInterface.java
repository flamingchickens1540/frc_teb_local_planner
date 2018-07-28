class TebInterface {

    static {
        System.load("/mnt/c/Users/wangl/CloudStation/Liam/ROS/teb-jni/teb-source/libjnitests.so");
    }

    public static native PoseSE2 plan(PoseSE2 start, PoseSE2 goal, PoseSE2 start_vel, boolean free_goal_vel);
}
