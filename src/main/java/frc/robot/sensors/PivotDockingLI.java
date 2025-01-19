package frc.robot.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class PivotDockingLI extends DigitalInput {
    public BooleanSupplier supplier;
    public PivotDockingLI(int channel) {
        super(channel);
        supplier = () -> {
            if (!this.get()) return true;
            return false;
        };
    }
    
}
