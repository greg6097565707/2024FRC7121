package frc.robot.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class WristBumperLI extends DigitalInput {
    public BooleanSupplier supplier;
    public WristBumperLI(int channel) {
        super(channel);
        supplier = () -> {
            if (!this.get()) return true;
            return false;
        };
    }
    
    public boolean isWristOnBumper() {
        return !this.get();
    }
}
