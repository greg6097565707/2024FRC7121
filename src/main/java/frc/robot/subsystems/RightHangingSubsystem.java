package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangingConstants;
import frc.robot.controllers.HangerController;

public class RightHangingSubsystem extends SubsystemBase {
    public RightHangingSubsystem() {}

    private final HangerController m_rightHanger = new HangerController(HangingConstants.rightID);
    // private final HangerController m_leftHanger = new HangerController(HangingConstants.leftID);

    public Command stopRight() {
        return runOnce(this::stopRightHanger);
    }
    public Command manualLowerRight() {
        return runOnce(this::lowerRightHanger);
    }
    public Command manualRaiseRight() {
        return runOnce(this::raiseRightHanger);
    }
    public Command raiseRight() {
        return startEnd(this::raiseRightHanger, this::stopRight);
    }
    public Command lowerRight() {
        return startEnd(this::lowerRightHanger, this::stopRight);
    }
    public void holdRight() {
        m_rightHanger.goToPosition(m_rightHanger.encoder().getPosition());
    }
    public void raiseRightHanger() {
        m_rightHanger.set(.3);
    }
    public void lowerRightHanger() {
        m_rightHanger.set(-.3);
    }
    public void stopRightHanger() {
        m_rightHanger.set(0);
    }
}
