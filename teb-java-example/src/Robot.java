public class Robot {

    private PoseSE2 position = new PoseSE2(0,0,0);
    private PoseSE2 velocity = new PoseSE2(0,0,0);

    public void update() {
        double delta = getDeltaSeconds();

        position.theta += delta * velocity.theta;
        position.x += delta * velocity.x * Math.cos(position.theta);
        position.y += delta * velocity.x * Math.sin(position.theta);
    }


    private long lastTime = System.currentTimeMillis();

    private double getDeltaSeconds() {
        return 100d/1000d;
    }

    public PoseSE2 getPosition() {
        return position;
    }

    public PoseSE2 getVelocity() {
        return velocity;
    }

    public void setPosition(PoseSE2 position) {
        this.position = position;
    }

    public void setVelocity(PoseSE2 velocity) {
        this.velocity = velocity;
    }
}
