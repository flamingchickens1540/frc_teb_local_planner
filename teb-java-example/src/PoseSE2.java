import java.text.DecimalFormat;

@SuppressWarnings("WeakerAccess")
public class PoseSE2 extends Object {
    public double x;
    public double y;
    public double theta;

    public PoseSE2() {
        this.x = 0;
        this.y = 0;
        this.theta = 0;
    }

    public PoseSE2(double x, double y) {
        this.x = x;
        this.y = y;
        this.theta = 0;
    }

    public PoseSE2(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public String toString() {
        DecimalFormat df = new DecimalFormat("000.00");
        return "(X "+df.format(this.x)+" Y "+df.format(this.y)+" Theta "+df.format(this.theta)+")";
    }

    public static PoseSE2 add(PoseSE2 a, PoseSE2 b) {
        return new PoseSE2(
        a.x + b.x,
        a.y + b.y,
        a.theta + b.theta);
    }

    public static PoseSE2 multiply(PoseSE2 a, double b) {
        return new PoseSE2(
        a.x * b,
        a.y * b,
        a.theta * b);
    }
}
