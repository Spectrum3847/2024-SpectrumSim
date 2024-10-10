package frc.spectrumLib.sim;

import lombok.Getter;
import lombok.Setter;

public class ArmConfig {

    @Getter @Setter private int numMotors = 1;
    @Getter @Setter private double pivotX = 0.7;
    @Getter @Setter private double pivotY = 0.3;
    @Getter @Setter private double ratio = 50;
    @Getter @Setter private double length = 0.5;
    @Getter @Setter private double simMOI = 1.2;
    @Getter @Setter private double simCGLength = 0.3;
    @Getter @Setter private double minAngle = Math.toRadians(-60);
    @Getter @Setter private double maxAngle = Math.toRadians(90);
    @Getter @Setter private double startingAngle = Math.toRadians(90);
    @Getter @Setter private boolean simulateGravity = true;

    public ArmConfig(
            double pivotX,
            double pivotY,
            double ratio,
            double length,
            double minAngle,
            double maxAngle,
            double startingAngle) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngle);
        this.maxAngle = Math.toRadians(maxAngle);
        this.startingAngle = Math.toRadians(startingAngle);
        this.pivotX = pivotX;
        this.pivotY = pivotY;
    }
}
