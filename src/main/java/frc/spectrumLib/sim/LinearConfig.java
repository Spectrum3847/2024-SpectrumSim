package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LinearConfig {
    public int numMotors = 1;
    public double kElevatorGearing = 5;
    public double kCarriageMassKg = 1;
    public double kElevatorDrumRadius = Units.inchesToMeters(0.955 / 2);
    public double kMinElevatorHeight = 0;
    public double kMaxElevatorHeight =
            10000; // Units.inchesToMeters(Robot.config.elevator.maxHeight);

    // Display Config
    public double angle = 90; // O is horizontal, 90 is vertical, CCW is positive
    public Color8Bit color = new Color8Bit(Color.kPurple);
    public double lineWidth = 10;
    public double initialX = 0.5;
    public double initialY = 0;
    public double staticLength = 20;
    public double movingLength = 20;

    public LinearConfig(double x, double y, double gearing, double drumRadius) {
        this.initialX = x;
        this.initialY = y;
        kElevatorGearing = gearing;
        kElevatorDrumRadius = drumRadius;
    }

    public LinearConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    public LinearConfig setCarriageMass(double carriageMassKg) {
        this.kCarriageMassKg = carriageMassKg;
        return this;
    }

    public LinearConfig setAngle(double angle) {
        this.angle = angle;
        return this;
    }

    public LinearConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    public LinearConfig setLineWidth(double lineWidth) {
        this.lineWidth = lineWidth;
        return this;
    }

    public LinearConfig setStaticLength(double lengthInches) {
        this.staticLength = Units.inchesToMeters(lengthInches);
        ;
        return this;
    }

    public LinearConfig setMovingLength(double lengthInches) {
        this.movingLength = Units.inchesToMeters(lengthInches);
        return this;
    }
}
