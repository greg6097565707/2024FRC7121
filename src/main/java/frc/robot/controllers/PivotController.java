package frc.robot.controllers;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class PivotController extends CANSparkMax {
    private final RelativeEncoder m_sparkMaxEncoder;
    private final SparkPIDController m_sparkMaxPIDController;
    public PivotController(int port) {
        super(port, MotorType.kBrushless);
        this.setIdleMode(IdleMode.kBrake);

        m_sparkMaxEncoder = this.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        
        m_sparkMaxPIDController = this.getPIDController();
        m_sparkMaxPIDController.setFeedbackDevice(this.encoder());

        m_sparkMaxPIDController.setP(6);
        m_sparkMaxPIDController.setI(0);
        m_sparkMaxPIDController.setD(0);
        m_sparkMaxPIDController.setIZone(0);
        m_sparkMaxPIDController.setFF(0);

        m_sparkMaxPIDController.setOutputRange(-.7,.7);

        m_sparkMaxEncoder.setPosition(0);

        this.burnFlash();
        
    }
    public RelativeEncoder encoder() {
        return m_sparkMaxEncoder;
    }
    public void goToPosition(double target) {
        m_sparkMaxPIDController.setReference(target, CANSparkMax.ControlType.kPosition);
    }
    
}
