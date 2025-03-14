package frc.robot.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIR extends DigitalInput {
    public BooleanSupplier supplier;
    public IntakeIR(int channel) {
        super(channel);
        supplier = () -> {
            if (!this.get()) return true;
            return false;
        };
    }
    public boolean isNoteInIntake() {
        return !this.get();
    }
    
}
