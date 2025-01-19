package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.controllers.ShooterController;

public class ShooterSubsystem extends SubsystemBase {

    public ShooterSubsystem() {
    }

    public ShooterController m_topShooter = new ShooterController(ShooterConstants.topShooterCanId);
    public ShooterController m_bottomShooter = new ShooterController(ShooterConstants.bottomShooterCanId);

    // public Command aimToLimelight() {
    //     return startEnd()
    // }
    // public BooleanSupplier
    public Command feedShot() {
        return startEnd(this::midFeed, this::stopShooters);
    }
    public Command yeetShot() {
        return startEnd(this::slowShootNote, this::stopShooters);
    }
    public Command manualStopShooters() {
        return runOnce(this::stopShooters);
    }
    public Command controlledShot() {
        return startEnd(this::shootNote, this::stopShooters);
    }
    public Command manualSpinShooters() {
        return runOnce(this::shootNote);
    }
    public Command spinShooters() {
        return startEnd(this::shootNote, this::stopShooters).onlyWhile(RobotContainer.readyToShoot());
    }
    public Command achieveClearance() {
        return startEnd(this::ejectNote, this::stopShooters).raceWith(new WaitCommand(.2));
    }
    public Command shootIntoAMP() {
        return startEnd(this::shootClose, this::stopShooters).onlyIf(RobotContainer.D_SHOOTER_IR.supplier);
    }
    public void slowShootNote() {
        m_topShooter.set(.6);
        m_bottomShooter.set(.6);
    }
    public void shootNote() {
        m_topShooter.set(.8);    //.8
        m_bottomShooter.set(.95);   //.95
    }
    public void midFeed() {
        m_topShooter.set(.50);
        m_bottomShooter.set(.55);
    }
    public void shootClose() {
        m_topShooter.set(.45);
        m_bottomShooter.set(.55);
    }
    public void ejectNote() {
        m_topShooter.set(-.2);
        m_bottomShooter.set(-.2);
    }
    public void stopShooters() {
        m_topShooter.set(0);
        m_bottomShooter.set(0);
    }
    public void distanceShot () {
        m_topShooter .set(.45); 
        m_bottomShooter .set(.55);
    }

    // public void periodic() {
    //     System.out.println(this.readyToShoot().getAsBoolean());
    // }
}
