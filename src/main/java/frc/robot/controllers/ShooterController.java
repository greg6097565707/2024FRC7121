package frc.robot.controllers;

// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterController extends TalonFX {
    public ShooterController(int port) {
        super(port);
        // this.setInverted(true);
        this.setNeutralMode(NeutralModeValue.Coast);
    }

}
