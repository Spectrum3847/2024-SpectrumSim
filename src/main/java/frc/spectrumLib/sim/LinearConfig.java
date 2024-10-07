package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

public class LinearConfig {
    @Getter private int numMotors = 1;
    @Getter private double elevatorGearing = 5;
    @Getter private double carriageMassKg = 1;
    @Getter private double drumRadius = Units.inchesToMeters(0.955 / 2);
    @Getter private double minHeight = 0;

    @Getter
    private double maxHeight = 10000; // Units.inchesToMeters(Robot.config.elevator.maxHeight);

    // Display Config
    @Getter private double angle = 90; // O is horizontal, 90 is vertical, CCW is positive
    @Getter private Color8Bit color = new Color8Bit(Color.kPurple);
    @Getter private double lineWidth = 10;
    @Getter private double initialX = 0.5;
    @Getter private double initialY = 0;
    @Getter private double staticLength = 20;
    @Getter private double movingLength = 20;

    public LinearConfig(double x, double y, double gearing, double drumRadius) {
        this.initialX = x;
        this.initialY = y;
        elevatorGearing = gearing;
        this.drumRadius = drumRadius;
    }

    public LinearConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    public LinearConfig setCarriageMass(double carriageMassKg) {
        this.carriageMassKg = carriageMassKg;
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
