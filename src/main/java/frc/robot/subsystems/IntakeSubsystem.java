package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
    public IntakeSubsystem() {
        m_groundIntake.setNeutralMode(NeutralMode.Brake);
        m_shooterIntake.setNeutralMode(NeutralModeValue.Brake);
    }

    public static WPI_VictorSPX m_groundIntake = new WPI_VictorSPX(IntakeConstants.intakeCanId);
    public TalonFX m_shooterIntake = new TalonFX(ShooterConstants.handoffCanId);

    public Command autoReleaseNote() {
        return startEnd(this::handoffIntake, this::stopHandoff).onlyWhile(RobotContainer.D_SHOOTER_IR.supplier);
    }
    public Command releaseNote() {
        return startEnd(this::handoffIntake, this::stopHandoff);
    }
    public Command groundPickup() {
        return startEnd(this::inwardGroundIntake, this::stopGroundIntake).until(RobotContainer.D_INTAKE_IR.supplier);
    }
    public Command holdNote() {
        return startEnd(this::keepNote, this::stopGroundIntake).until(RobotContainer.D_WRIST_DOCKING_LI.supplier);
    }
    public Command handoff() {
        return startEnd(this::transferNote, this::stopTransfer).until(RobotContainer.D_SHOOTER_IR.supplier);
    }
    public Command clearNote() {
        return startEnd(this::handoffOuttake, this::stopHandoff).raceWith(new WaitCommand(.07).onlyIf(RobotContainer.D_SHOOTER_IR.supplier));
    }
    public Command reverseHandoff() {
        return startEnd(this::reverseTransferNote, this::stopTransfer).until(RobotContainer.D_INTAKE_IR.supplier);
    }
    public Command correctTransfer() {
        return runEnd(this::resetNote, this::stopTransfer).until(this.noteIsDetected());
    }

    public void resetNote() {
        m_groundIntake.set(.2);
        m_shooterIntake.set(.2);
    }
    public void handoffIntake() {
        m_shooterIntake.set(-.5);
        m_groundIntake.set(-.5);
    }
    public void handoffOuttake() {
        m_shooterIntake.set(.1);
    }
    public void stopHandoff() {
        m_shooterIntake.set(0);
    }
    public void inwardGroundIntake() {
        m_groundIntake.set(.75);
        System.out.println(RobotContainer.D_INTAKE_IR.isNoteInIntake());
    }
    public void outwardGroundIntake() {
        m_groundIntake.set(-.5);
    }
    public void stopGroundIntake() {
        m_groundIntake.set(0);
    }
    public void keepNote() {
        m_groundIntake.set(.2);
    }
    public void transferNote() {
        m_groundIntake.set(-.35);
        m_shooterIntake.set(-.18);
    }
    public void reverseTransferNote() {
        m_groundIntake.set(.45);
        m_shooterIntake.set(.45);
    }
    public void stopTransfer() {
        m_groundIntake.set(0);
        m_shooterIntake.set(0);
    }

    public BooleanSupplier handoffReady() {
        return (BooleanSupplier) () -> {
            if (DriverStation.isTeleopEnabled() && RobotContainer.D_INTAKE_IR.isNoteInIntake() && RobotContainer.D_WRIST_DOCKING_LI.isWristDocked()) return true;
            return false;
        };
    }
    public BooleanSupplier noteUnknown() {
        return (BooleanSupplier) () -> {
            if (RobotContainer.D_PIVOT_DOCKING_LI.supplier.getAsBoolean() && RobotContainer.D_WRIST_DOCKING_LI.isWristDocked() && !RobotContainer.D_INTAKE_IR.supplier.getAsBoolean() && !RobotContainer.D_SHOOTER_IR.supplier.getAsBoolean()) return true;
            return false;
        };
    }
    public BooleanSupplier noteIsDetected() {
        return (BooleanSupplier) () -> {
            if (RobotContainer.D_INTAKE_IR.supplier.getAsBoolean() || RobotContainer.D_SHOOTER_IR.supplier.getAsBoolean()) return true;
            return false;
        };
    }


}
