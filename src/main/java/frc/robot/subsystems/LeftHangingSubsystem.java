package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangingConstants;
import frc.robot.controllers.HangerController;

public class LeftHangingSubsystem extends SubsystemBase {
    public LeftHangingSubsystem() {}

    // private final HangerController m_leftHanger = new HangerController(HangingConstants.leftID);
    private final HangerController m_leftHanger = new HangerController(HangingConstants.leftID);

    public Command manualRaiseLeft() {
        return runOnce(this::raiseLeftHanger);
    }
    public Command manualLowerLeft() {
        return runOnce(this::lowerLeftHanger);
    }
    public Command raiseLeft() {
        return startEnd(this::raiseLeftHanger, this::stopLeft);
    }
    public Command lowerLeft() {
        return startEnd(this::lowerLeftHanger, this::stopLeft);
    }
    public Command stopLeft() {
        return runOnce(this::stopLeftHanger);
    }
    public void holdLeft() {
        m_leftHanger.goToPosition(m_leftHanger.encoder().getPosition());
    }
    public void raiseLeftHanger() {
        m_leftHanger.set(-.3);
    }
    public void lowerLeftHanger() {
        m_leftHanger.set(.3);
    }
    public void stopLeftHanger() {
        m_leftHanger.set(0);
    }
}
