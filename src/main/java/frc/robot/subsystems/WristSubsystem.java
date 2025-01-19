package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.controllers.WristController;

public class WristSubsystem extends SubsystemBase {
    public WristSubsystem() {}
    public static WristController m_wrist = new WristController(WristConstants.wristCanId);

    public Command restWrist() {
        return startEnd(this::goToBumper, this::stopWrist).until(RobotContainer.D_WRIST_BUMPER_LI.supplier);
    }
    public Command dockWrist() {
        return startEnd(this::goToDock, this::stopWrist).until(RobotContainer.D_WRIST_DOCKING_LI.supplier);
    }
    public Command sourceFling() {
        return startEnd(this::goToSource, this::goToDock).until(RobotContainer.D_INTAKE_IR.supplier);
    }
    
    public void goToBumper() {
        m_wrist.set(-0.5);
        System.out.println("waiting for bumper");
    }
    public void goToDock() {
        System.out.println("waiting for dock");
        m_wrist.set(0.5);
    }
    public void goToSource() {
        m_wrist.goToPosition(.4);
    }
    public void raiseWrist() {
        if (this.isWristDocked().getAsBoolean()) this.stopWrist();
        else m_wrist.set(-.5);
    }
    public void lowerWrist() {      
        if (this.isWristOnBumper().getAsBoolean()) this.stopWrist();
        else m_wrist.set(.4);
    }
    public void stopWrist() {
        m_wrist.set(0);
    }

    public BooleanSupplier isWristOnBumper() {
        return RobotContainer.D_WRIST_BUMPER_LI.supplier;
    }
    public BooleanSupplier isWristDocked() {
        return RobotContainer.D_WRIST_DOCKING_LI.supplier;
    }

    public void periodic() {
        // System.out.println(RobotContainer.D_WRIST_DOCKING_LI.supplier.getAsBoolean());
        if (RobotContainer.D_WRIST_BUMPER_LI.isWristOnBumper()) {
            m_wrist.encoder().setPosition(0.0);
        } else if (RobotContainer.D_WRIST_DOCKING_LI.isWristDocked()) {
            m_wrist.encoder().setPosition(0.5);
        }
    }

}
