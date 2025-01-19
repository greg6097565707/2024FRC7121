package frc.robot.subsystems;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.controllers.RGBController;

public class RGBSubsystem extends SubsystemBase {
    private final RGBController m_blinkinDriver = new RGBController();
    public RGBSubsystem() {
        this.idle();
    }
    public void idle() {
        m_blinkinDriver.set(.15);
    }
    public void noNote() {
        //red
        m_blinkinDriver.set(.61);
    }
    public void noteHandoff() {
        //yellow
        m_blinkinDriver.set(.69);
    }
    public void noteInShooter() {
        //green
        m_blinkinDriver.set(.57);
    }
    public void limelightDetect() {
        m_blinkinDriver.set(.73);
    }
    public void periodic() {
        if (RobotContainer.D_INTAKE_IR.supplier.getAsBoolean()) {
            this.noteHandoff();
        } else if (RobotContainer.D_SHOOTER_IR.supplier.getAsBoolean()) {
            this.noteInShooter();
        } else if (!RobotContainer.D_INTAKE_IR.supplier.getAsBoolean()) {
            this.noNote();
        }
    }
}
