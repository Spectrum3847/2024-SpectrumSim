package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import lombok.Getter;

// Use this class to create a SmartDashboard tunable value
// You can put this in a Command to get the value from the SmartDashboard
public class TuneValue {
    @Getter private double value;
    @Getter private String name;

    public TuneValue(String Name, double defaultValue) {
        SmartDashboard.putNumber(Name, defaultValue);
        value = defaultValue;
        name = Name;
    }

    public Double update() {
        value = SmartDashboard.getNumber(name, value);
        return value;
    }

    public DoubleSupplier getSupplier() {
        return () -> update();
    }
}
