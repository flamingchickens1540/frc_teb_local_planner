class Main {

    public static void main(String[] args) {
        PoseSE2 cmd_vel = TebInterface.plan(new PoseSE2(5.1, 3.1415, 9000.1), new PoseSE2(0, 5, 0), new PoseSE2(), false);
        System.out.println(cmd_vel);
    }
}
