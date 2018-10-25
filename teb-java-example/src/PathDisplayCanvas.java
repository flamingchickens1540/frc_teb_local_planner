import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

class PathDisplayCanvas extends ImageCanvas
{

    private BufferedImage buffer;

    PathDisplayCanvas(int width, int height) {
        super(width,height);
        buffer = new BufferedImage(900, 1200, BufferedImage.TYPE_INT_RGB);
        Graphics bufferedPen = buffer.getGraphics();
        bufferedPen.setColor(Color.WHITE);
        bufferedPen.fillRect(0, 0, buffer.getWidth(), buffer.getHeight());
    }

    public void created() {
        resized();
        draw();
    }

    // Drawing

    public void draw()
    {
        Graphics2D pen = getPen();
        pen.drawImage(buffer, 0, 0, (int) ((double) buffer.getWidth()), (int) ((double) buffer.getHeight()), this);
        display();
    }

    public void resized() {
        Graphics2D pen = getPen();
        pen.setColor(new Color(255, 255, 255));
        pen.fillRect(0, 0, this.getWidth(), this.getHeight());
        draw();
    }

    public void drawPoseSE2(PoseSE2 pose) {
        Graphics2D pen = getPen();
        pen.setColor(new Color(100, 0, 0));
        int length = 40;
        int arrowlength = 15;
        int scale = 30;
        int xoffset = 10;
        int yoffset = this.getHeight() / 2;
        pen.drawLine(
            (int) (pose.x*scale+ xoffset),
            (int) (pose.y*scale+ yoffset),
            (int) (length*Math.cos(pose.theta)+pose.x*scale+ xoffset),
            (int) (length*Math.sin(pose.theta)+pose.y*scale+ yoffset));
        pen.drawLine(
            (int) (arrowlength*Math.cos(pose.theta+1)+pose.x*scale+ xoffset),
            (int) (arrowlength*Math.sin(pose.theta+1)+pose.y*scale+ yoffset),
            (int) (length*Math.cos(pose.theta)+pose.x*scale+ xoffset),
            (int) (length*Math.sin(pose.theta)+pose.y*scale+ yoffset));
        pen.drawLine(
            (int) (arrowlength*Math.cos(pose.theta-1)+pose.x*scale+ xoffset),
            (int) (arrowlength*Math.sin(pose.theta-1)+pose.y*scale+ yoffset),
            (int) (length*Math.cos(pose.theta)+pose.x*scale+ xoffset),
            (int) (length*Math.sin(pose.theta)+pose.y*scale+ yoffset));
        draw();
    }
}