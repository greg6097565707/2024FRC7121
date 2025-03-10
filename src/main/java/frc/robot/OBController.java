package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * A custom wrapper class for one of our cheap off-brand controllers which has the button name/ID map built in for ease of use and reduced clutter
 * Make sure the controller is in Digital Mode (flip the switch on the bottom from X to D)
 */
public class OBController extends Joystick
{
    // public final InputMode inputMode;

    //𝗟𝗲𝘁𝘁𝗲𝗿 𝗯𝘂𝘁𝘁𝗼𝗻𝘀
    public final JoystickButton xButton = new JoystickButton(this, 1);
    public final JoystickButton aButton = new JoystickButton(this, 2);
    public final JoystickButton bButton = new JoystickButton(this, 3);
    public final JoystickButton yButton = new JoystickButton(this, 4);

    //𝗕𝘂𝗺𝗽𝗲𝗿𝘀 𝗮𝗻𝗱 𝗧𝗿𝗶𝗴𝗴𝗲𝗿𝘀
    public final JoystickButton lbButton = new JoystickButton(this, 5);
    public final JoystickButton rbButton = new JoystickButton(this, 6);
    public final JoystickButton ltButton = new JoystickButton(this, 7);
    public final JoystickButton rtButton = new JoystickButton(this, 8);

    /** The Back button  */
    public final JoystickButton bkButton = new JoystickButton(this,  9);
    /** The Start button */ 
    public final JoystickButton stButton = new JoystickButton(this, 10);
    
    //𝗧𝗵𝘂𝗺𝗯𝘀𝘁𝗶𝗰𝗸𝘀
    public final Thumbstick lStick = new Thumbstick(0, 1, 11);
    public final Thumbstick rStick = new Thumbstick(2, 3, 12);


    //D-pad (creates new Triggers which use getPOV() as BooleanSuppliers)
    public final Trigger dpadDf = new Trigger(() -> (getPOV() ==  -1));
    public final Trigger dpadNN = new Trigger(() -> (getPOV() ==   0));
    public final Trigger dpadNE = new Trigger(() -> (getPOV() ==  45));//also triggered when the middle button is pressed
    public final Trigger dpadEE = new Trigger(() -> (getPOV() ==  90));
    public final Trigger dpadSE = new Trigger(() -> (getPOV() == 135));
    public final Trigger dpadSS = new Trigger(() -> (getPOV() == 180));
    public final Trigger dpadSW = new Trigger(() -> (getPOV() == 225));
    public final Trigger dpadWW = new Trigger(() -> (getPOV() == 270));
    public final Trigger dpadNW = new Trigger(() -> (getPOV() == 315));
    
    /* old way of doing dpad which didn't work
    public final JoystickButton dpadDf = new JoystickButton(this, ControllerMap.dpDf);
    public final JoystickButton dpadNN = new JoystickButton(this, ControllerMap.dpNN);
    public final JoystickButton dpadNW = new JoystickButton(this, ControllerMap.dpNW);
    public final JoystickButton dpadWW = new JoystickButton(this, ControllerMap.dpWW);
    public final JoystickButton dpadSW = new JoystickButton(this, ControllerMap.dpSW);
    public final JoystickButton dpadSS = new JoystickButton(this, ControllerMap.dpSS);
    public final JoystickButton dpadSE = new JoystickButton(this, ControllerMap.dpSE);
    public final JoystickButton dpadEE = new JoystickButton(this, ControllerMap.dpEE);
    public final JoystickButton dpadNE = new JoystickButton(this, ControllerMap.dpNE);
    */
   
    /** Creates a new {@link OBController} */
    public OBController(int port) {
        super(port);
        // inputMode  = (getAxisCount() == 4)? InputMode.DIGITAL : InputMode.ANALOG;
    }

    //helper to reduce redundancy and clutter
    public final class Thumbstick 
    {
        private final int xID, yID, buttonID;
        private final JoystickButton button;

        public Thumbstick(int xAxis, int yAxis, int btn)
        {
            xID = xAxis;
            yID = yAxis;
            buttonID = btn;
            button = new JoystickButton(OBController.this, buttonID);
        }

        public final double x() {
            return getRawAxis(xID); 
        }
        public final double y() {
            return getRawAxis(yID); 
        }
        public final JoystickButton button() {
            return button;
        }
    }

    //TODO input modes
    /* 
     * Enum for input modes and how many analog axes each mode has
     * Each mode has different number of analog axes (ex: triggers being axes instead of buttons)
    */
    // private static enum InputMode {  ANALOG, DIGITAL } 
}