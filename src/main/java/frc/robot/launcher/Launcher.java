package frc.robot.launcher;

import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.RollerSim;
import frc.spectrumLib.mechanism.RollerSim.RollerConfig;
import frc.spectrumLib.mechanism.TalonFXFactory;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {
        public double maxVelocity = 5600;

        /* LeftLauncher config values */
        public double currentLimit = 60;
        public double torqueCurrentLimit = 300;
        public double threshold = 80;
        public double velocityKp = 6;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public LauncherConfig() {
            super("Launcher", 42, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public LauncherConfig config;
    public RollerSim sim;

    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

        // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
        sim =
                new RollerSim(
                        RobotSim.leftView,
                        motor.getSimState(),
                        new RollerConfig().setDiameter(6),
                        getName());
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    public void simulationPeriodic() {
        sim.simulationPeriodic(0.5, 0.5);
    }
}
