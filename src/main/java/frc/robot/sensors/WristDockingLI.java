package frc.robot.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class WristDockingLI extends DigitalInput {
    public BooleanSupplier supplier;
    public WristDockingLI(int channel) {
        super(channel);
        supplier = () -> {
            if (!this.get()) return true;
            return false;
        };
    }
    public boolean isWristDocked() {
        return !this.get();
    }

    
}
