package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.controllers.PivotController;

public class PivotSubsystem extends SubsystemBase {
    private static final InterpolatingTreeMap<Double, Double> shotCurve = new InterpolatingDoubleTreeMap();
    private double curveEstimateRaw;
    public double curveEstimateAdjusted;

    public PivotSubsystem() {
        shotCurve.put(9.884156, .072265625);
        shotCurve.put(20.074647903, .1053183);
        shotCurve.put(7.01899814, .07836914);
        shotCurve.put(17.18157386, .092796875);
        shotCurve.put(11.004891, 0.0718994140625);
        shotCurve.put(17.96202278137207, 0.1103515625);
        shotCurve.put(-14.978446, 0.00);
        shotCurve.put(16.597925186157227, 0.0980224609375);
        shotCurve.put(15.674973487854004, 0.09619140625);
        // shotCurve.put(5.055, .1031);
        // shotCurve.put(6.26, .1115);
    }

    public final PivotController m_shooterPivot = new PivotController(ShooterConstants.pivotCanId);
    private final PivotController m_slavePivot = new PivotController(ShooterConstants.slavePivotCanId);

    public Command manualRaisePivot() {
        return startEnd(this::manualRaisePivot, this::holdPivot);
    }
    public Command manualLowerPivot() {
        return startEnd(this::manualLowerPivot, this::holdPivot);
    }
    public Command dockPivot() {
        return runOnce(this::dockPivot);
    }
    public Command pivotAtStage() {
        return startEnd(this::positionAtStage, this::resetPivot).onlyWhile(RobotContainer.D_SHOOTER_IR.supplier);
    }
    public Command pivotAtAMP() {
        return startEnd(this::positionAtAMP, this::resetPivot).onlyIf(RobotContainer.D_SHOOTER_IR.supplier);
    }
    public Command aimWithLimelight() {
        return runEnd(this::trackAprilTag, this::resetPivot).onlyWhile(RobotContainer.readyToShoot());
    }
    public void holdPivot() {
        m_shooterPivot.goToPosition(m_shooterPivot.encoder().getPosition());
    }
    public void positionAtStage() {
        m_shooterPivot.goToPosition(.085);
    }
    public void positionAtAMP() {
        m_shooterPivot.goToPosition(.25);
    }
    public void resetPivot() {
        m_shooterPivot.goToPosition(0.0);
    }
    public void trackAprilTag() {
        m_shooterPivot.goToPosition(curveEstimateAdjusted);
    }
    public void raisePivot() {
        m_shooterPivot.set(.2);
    }
    public void lowerPivot() {
        m_shooterPivot.set(-.2);
    }

    public void periodic() {
        // System.out.println(isAimedAtSpeaker().getAsBoolean());
        curveEstimateRaw = shotCurve.get(-LimelightHelpers.getTY("limelight"));
        curveEstimateAdjusted = Math.min(.19, Math.max(0, curveEstimateRaw + SmartDashboard.getNumber("Aiming Adjustment", 0)));

        double[] data = {curveEstimateRaw, curveEstimateAdjusted, m_shooterPivot.encoder().getPosition()};
        SmartDashboard.putNumberArray("Aiming", data);
        SmartDashboard.putNumber("Curve Estimate Raw", curveEstimateRaw);
        // SmartDashboard.putNumber("Curve Estimated Adjusted", curveEstimateAdjusted);
        SmartDashboard.putNumber("Curve Actual Output", m_shooterPivot.encoder().getPosition());

        if (RobotContainer.D_PIVOT_DOCKING_LI.supplier.getAsBoolean()) {
            m_shooterPivot.encoder().setPosition(0.0);
        }
    }
}
