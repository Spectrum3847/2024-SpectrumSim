package frc.spectrumLib.sim;

public class ArmConfig {

    public int numMotors = 1;
    public double pivotX = 0.7;
    public double pivotY = 0.3;
    public double ratio = 50;
    public double length = 0.5;
    public double simMOI = 0.2;
    public double simCGLength = 0.3;
    public double minAngle = Math.toRadians(-60);
    public double maxAngle = Math.toRadians(90);
    public double startingAngle = Math.toRadians(90);
    public boolean simulateGravity = true;

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
