package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class WristController extends CANSparkMax {
    // private final CANSparkMax m_sparkMax;
    private final RelativeEncoder m_sparkMaxEncoder;
    private final SparkPIDController m_sparkMaxPIDController;
    public WristController(int port) {
        super(port, MotorType.kBrushless);
        
        // m_sparkMax.restoreFactoryDefaults();
        m_sparkMaxEncoder = this.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        m_sparkMaxPIDController = this.getPIDController();
        m_sparkMaxPIDController.setFeedbackDevice(m_sparkMaxEncoder);

        m_sparkMaxPIDController.setPositionPIDWrappingEnabled(true);

        m_sparkMaxPIDController.setP(4);
        m_sparkMaxPIDController.setI(0.001);
        m_sparkMaxPIDController.setD(2);
        m_sparkMaxPIDController.setIZone(0);
        m_sparkMaxPIDController.setFF(0);

        m_sparkMaxPIDController.setOutputRange(-.5,.5);

        this.setIdleMode(IdleMode.kBrake);

        m_sparkMaxEncoder.setPosition(0);
        m_sparkMaxEncoder.setInverted(true);
        // m_sparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

        this.burnFlash();
    }

    // public void set(double power)
    // {
    //     this.set(power);
    // }

    public RelativeEncoder encoder()
    {
        return m_sparkMaxEncoder;
    }
    public void goToPosition(double target) {
        m_sparkMaxPIDController.setReference(target, CANSparkMax.ControlType.kPosition);
    }

}
