package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class HangerController extends CANSparkMax {
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;
    public HangerController(int port) {
        super(port, MotorType.kBrushless);

        m_encoder = this.getEncoder();
        m_pidController = this.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(0.7);
        m_pidController.setI(0.0);
        m_pidController.setD(0.0);
        m_pidController.setIZone(0);

        m_pidController.setFF(0.0);
        m_pidController.setOutputRange(-.5, .5);

        this.setIdleMode(IdleMode.kBrake);
        
        m_encoder.setPosition(0);
    }
    public RelativeEncoder encoder() {
        return m_encoder;
    }
    public void goToPosition(double position) {

    }
}
