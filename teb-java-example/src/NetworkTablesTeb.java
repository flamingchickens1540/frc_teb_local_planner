import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesTeb {

    private static final double TWO_PI = Math.PI*2;
    private static Robot bot = new Robot();
    private static TebConfig conf = new TebConfig();

    private static void handle() {
        updateBot();
    }


    public static void main(String[] args) {
        TebInterface.updateConfig(conf);
        TebInterface.initialize(new RobotFootprint());

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable smartDashboard = inst.getTable("SmartDashboard");

        inst.startClient("10.15.40.2");

        smartDashboard.addEntryListener("X-Position", (table, key, entry, value, flags) -> {
            System.out.println("X Changed Value:"+value.getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

//        bot.setPosition(new PoseSE2(0,0,0));


        try {
            Thread.sleep(100000);
        } catch (InterruptedException ex){
            System.out.println("Interrupted!");
        }

//        launch(args);
    }

    static boolean free_goal_vel = true;

    static PoseSE2 goala = new PoseSE2(5, 0, 0);
    //        PoseSE2 goal = new PoseSE2(10, -10, 0);
    static PoseSE2 goal = new PoseSE2(10, 0, 0);
    //        PoseSE2 goal = new PoseSE2(10, -10, -3.14/2);

    private static void updateBot() {


        PoseSE2 cmd_vel = TebInterface.plan(bot.getPosition(), goala, bot.getVelocity(), free_goal_vel);
        bot.setVelocity(cmd_vel);
        bot.update();

        System.out.print("Vel: ");
        System.out.print(cmd_vel);

        System.out.print("  Pos: ");
        System.out.println(bot.getPosition());

        double dx = goala.x - bot.getPosition().x;
        double dy = goala.y - bot.getPosition().y;
        double delta_orient = normalizeAngle(goala.theta - bot.getPosition().theta, 0.0);
        if (Math.abs(Math.sqrt(dx * dx + dy * dy)) < conf.xy_goal_tolerance
            && Math.abs(delta_orient) < conf.yaw_goal_tolerance) {
            goala = goal;
            free_goal_vel = false;
        }

        dx = goal.x - bot.getPosition().x;
        dy = goal.y - bot.getPosition().y;
        delta_orient = normalizeAngle(goal.theta - bot.getPosition().theta, 0.0);
        if (Math.abs(Math.sqrt(dx * dx + dy * dy)) < conf.xy_goal_tolerance
            && Math.abs(delta_orient) < conf.yaw_goal_tolerance) {
        }
    }

    private static double normalizeAngle(double a, double center) {
        return a - TWO_PI * Math.floor((a + Math.PI - center) / TWO_PI);
    }
}