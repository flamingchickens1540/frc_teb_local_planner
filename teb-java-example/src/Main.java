import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

class Main {

    private static final double TWO_PI = Math.PI*2;
    private static Robot bot = new Robot();
    private static PathDisplayCanvas editorCanvas = new PathDisplayCanvas(600, 400);
    private static TebConfig conf = new TebConfig();
    private static Timer updateTimer = new Timer();

    private static List<Double> pathListX = new ArrayList<>();
    private static List<Double> pathListY = new ArrayList<>();




    public static void main(String[] args) {
        conf.max_vel_x = 1;
        TebInterface.updateConfig(conf);
        TebInterface.initialize(new RobotFootprint());

        bot.setPosition(new PoseSE2(0,0,0));


//        JFrame myJFrame = new JFrame("TEB Local Planner Sim");
//        myJFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//        myJFrame.setMinimumSize(new Dimension(1200, 900));
//
//        JPanel mainPanel = new JPanel();
//        mainPanel.setLayout(new BorderLayout());
//
//
//        mainPanel.add(editorCanvas, BorderLayout.CENTER);
//
//        myJFrame.getContentPane().add(mainPanel);
//        myJFrame.pack();
//        myJFrame.setLocationRelativeTo(null);
//        myJFrame.setVisible(true);

        ScatterChartSample.launch();



//        updateTimer.schedule(new UpdateRoutine(), 0, 1);
//        Timer drawTimer = new Timer();
//        drawTimer.schedule(new DrawRoutine(), 0, 200);

    }
    static class UpdateRoutine extends TimerTask {
        public void run() {
            PoseSE2 goal = new PoseSE2(10, -10, 0);
            PoseSE2 cmd_vel = TebInterface.plan(bot.getPosition(), goal, bot.getVelocity(), false);
            bot.setVelocity(cmd_vel);
            bot.update();
//            System.out.println(bot.getPosition());
//            System.out.println(bot.getVelocity());
            pathListX.add(bot.getPosition().x);
            pathListY.add(bot.getPosition().y);

            double dx = goal.x - bot.getPosition().x;
            double dy = goal.y - bot.getPosition().y;
            double delta_orient = normalizeAngle(goal.theta - bot.getPosition().theta, 0.0);
            if (Math.abs(Math.sqrt(dx * dx + dy * dy)) < conf.xy_goal_tolerance
                && Math.abs(delta_orient) < conf.yaw_goal_tolerance) {
                updateTimer.cancel();
            }
        }
    }

    public static double normalizeAngle(double a, double center) {
        return a - TWO_PI * Math.floor((a + Math.PI - center) / TWO_PI);
    }

    static class DrawRoutine extends TimerTask {
        public void run() {
//            System.out.println(bot.getVelocity());
//            System.out.println(bot.getPosition());
//            editorCanvas.drawPoseSE2(bot.getPosition());
        }
    }
}
