import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.awt.geom.*;
import java.util.*;

/****************************************
 * 
 * A Canvas for double-buffered graphics.

  Useful methods in this class:

  <pre>
  public Graphics2D getPen();  // gets a pen for drawing on off-screen buffer
  public void display();       // displays the off-screen buffer on the screen
  public void clear();         // clears the off-screen buffer

  public int getWidth();
  public int getHeight();
  </pre>
 */

public class ImageCanvas extends Canvas
{   
    private BufferStrategy strategy = null;
    private Color backgroundColor = Color.WHITE;

    //**********************************************************************
    // Constructor: pass the width and height you want for this canvas.

    public ImageCanvas(int width, int height) 
    {
        setSize(width, height);
        setIgnoreRepaint(true);
        addComponentListener(new ImageCanvasComponentListener());
    }

    //*****************************************************************
    // This is called automatically to notify the Canvas that it has been added to the GUI
    public void addNotify()
    {
        super.addNotify();
        createBufferStrategy(2);
        strategy = getBufferStrategy();
        created();
 
    }  

    /************************
     * Returns the Graphics2D pen used to draw on the off-screen buffer.
     */

    public Graphics2D getPen()
    {
        return (Graphics2D) strategy.getDrawGraphics();
    }

    /************************
     * Call display() when you are done drawing a frame on the off-screen buffer.
     * This displays the contents of the buffer on the screen.
     */

    public void display()
    {
        strategy.show();
    }

    //**********************************************************************
    // clears the image panel

    public void clear()
    {
        Graphics2D pen = getPen();
        pen.setColor(backgroundColor);
        pen.fillRect(0, 0, getWidth(), getHeight());
    }

    //**********************************************************************
    // this function is called when the canvas is resized
    // override it to redraw your content

    public void resized() {
    }
    
    public void created() {
    }

    class ImageCanvasComponentListener extends ComponentAdapter
    {
        public void componentResized(ComponentEvent event) 
        {
            resized();
        }
    }

}