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
        return "Java PoseSE2 (X "+this.x+" Y "+this.y+" Theta "+this.theta;
    }
}
