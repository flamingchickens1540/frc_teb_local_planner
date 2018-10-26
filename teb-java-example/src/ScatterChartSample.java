import javafx.animation.Animation;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.Scene;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.ScatterChart;
import javafx.scene.chart.XYChart;
import javafx.scene.chart.XYChart.Data;
import javafx.stage.Stage;
import javafx.util.Duration;


public class ScatterChartSample extends Application {

    private static final double TWO_PI = Math.PI*2;
    private static Robot bot = new Robot();
    private static TebConfig conf = new TebConfig();
    private static XYChart.Series series1 = new XYChart.Series();

    private static Timeline timeline = new Timeline(
        new KeyFrame(Duration.millis(10), ScatterChartSample::handle)
    );

    private static void handle(ActionEvent e) {
        series1.getData().add(new Data(bot.getPosition().x, bot.getPosition().y));
        updateBot();
    }


    @Override public void start(Stage stage) {

        stage.setTitle("Scatter Chart Sample");
        final NumberAxis xAxis = new NumberAxis(-15, 15, 1);
        final NumberAxis yAxis = new NumberAxis(-15, 15, 1);
        final ScatterChart<Number,Number> sc = new
            ScatterChart<>(xAxis,yAxis);

        sc.getData().addAll(series1);
        Scene scene  = new Scene(sc, 500, 400);
        stage.setScene(scene);
        stage.show();

        EventHandler<ActionEvent> actionEventEventHandler = ScatterChartSample::handle;

        timeline.setCycleCount(Animation.INDEFINITE);
        timeline.play();
    }

    public static void main(String[] args) {
        TebInterface.updateConfig(conf);
        TebInterface.initialize(new RobotFootprint());

        bot.setPosition(new PoseSE2(0,0,0));




        launch(args);
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
            timeline.stop();
        }
    }

    private static double normalizeAngle(double a, double center) {
        return a - TWO_PI * Math.floor((a + Math.PI - center) / TWO_PI);
    }
}