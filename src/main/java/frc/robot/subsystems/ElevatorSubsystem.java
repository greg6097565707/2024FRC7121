package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.controllers.ElevatorController;

public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorSubsystem() {};

    private final ElevatorController m_elevator = new ElevatorController(ElevatorConstants.elevatorCanId);

    public Command raiseToAmp() {
        return startEnd(this::positionToAMP, this::resetElevator).onlyIf(RobotContainer.D_SHOOTER_IR.supplier);
    }

    public void positionToAMP() {
        m_elevator.goToPosition(60);
    }
    public void resetElevator() {
        m_elevator.goToPosition(0);
    }
    public void raiseElevator() {
        m_elevator.set(.2);
    }
    public void lowerElevator() {
        m_elevator.set(-.2);
    }
}
