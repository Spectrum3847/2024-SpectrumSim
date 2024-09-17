package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LinearConfig {
    public int numMotors = 1;
    public double kElevatorGearing = 5;
    public double kCarriageMass = 1;
    public double kElevatorDrumRadius = Units.inchesToMeters(0.955 / 2);
    public double kMinElevatorHeight = 0;
    public double kMaxElevatorHeight =
            10000; // Units.inchesToMeters(Robot.config.elevator.maxHeight);

    // Display Config
    public double angle = 90;
    public Color8Bit color = new Color8Bit(Color.kPurple);
    public double lineWidth = 10;

    public LinearConfig(double gearing, double drumRadius, double mass) {
        kElevatorGearing = gearing;
        kCarriageMass = mass;
        kElevatorDrumRadius = drumRadius;
    }
}
