package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ElevatorController extends CANSparkMax {
    // private final CANSparkMax m_sparkMax;
    private final RelativeEncoder m_sparkMaxEncoder;
    private final SparkPIDController m_sparkMaxPIDController;
    public ElevatorController(int port) {
        super(port, MotorType.kBrushless);

        this.setInverted(true);
        
        // m_sparkMax.restoreFactoryDefaults();
        m_sparkMaxEncoder = this.getEncoder();
        m_sparkMaxPIDController = this.getPIDController();
        m_sparkMaxPIDController.setFeedbackDevice(m_sparkMaxEncoder);

        m_sparkMaxPIDController.setPositionPIDWrappingEnabled(true);

        m_sparkMaxPIDController.setP(.6);
        m_sparkMaxPIDController.setI(0);
        m_sparkMaxPIDController.setD(0);
        m_sparkMaxPIDController.setIZone(0);

        m_sparkMaxPIDController.setFF(0);
        m_sparkMaxPIDController.setOutputRange(-1,1);

        this.setIdleMode(IdleMode.kBrake);

        m_sparkMaxEncoder.setPosition(0);

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
