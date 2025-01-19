package frc.robot.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIR extends DigitalInput {
    public BooleanSupplier supplier;
    public BooleanSupplier invertedSupplier;
    public ShooterIR(int channel) {
        super(channel);
        supplier = () -> {
            if (!this.get()) return true;
            return false;
        };
        invertedSupplier = () -> {
            return this.get();
        };
    }
    public boolean isNoteInShooter() {
        return !this.get();
    }
    
}
