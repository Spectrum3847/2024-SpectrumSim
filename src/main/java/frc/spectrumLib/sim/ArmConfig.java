package frc.spectrumLib.sim;

import lombok.Getter;

public class ArmConfig {

    @Getter private int numMotors = 1;
    @Getter private double pivotX = 0.7;
    @Getter private double pivotY = 0.3;
    @Getter private double ratio = 50;
    @Getter private double length = 0.5;
    @Getter private double simMOI = 0.2;
    @Getter private double simCGLength = 0.3;
    @Getter private double minAngle = Math.toRadians(-60);
    @Getter private double maxAngle = Math.toRadians(90);
    @Getter private double startingAngle = Math.toRadians(90);
    @Getter private boolean simulateGravity = true;

    public ArmConfig(
            double pivotX,
            double pivotY,
            double ratio,
            double length,
            double minAngle,
            double maxAngle) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngle);
        this.maxAngle = Math.toRadians(maxAngle);
        this.pivotX = pivotX;
        this.pivotY = pivotY;
    }

    public ArmConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    public ArmConfig setSimulateGravity(boolean simulateGravity) {
        this.simulateGravity = simulateGravity;
        return this;
    }

    public ArmConfig setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
        return this;
    }

    public ArmConfig setSimMOI(double simMOI) {
        this.simMOI = simMOI;
        return this;
    }

    public ArmConfig setSimCGLength(double simCGLength) {
        this.simCGLength = simCGLength;
        return this;
    }
}
