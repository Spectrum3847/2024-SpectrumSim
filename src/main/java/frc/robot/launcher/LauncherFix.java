package frc.robot.launcher;

import frc.robot.RobotConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;

public class LauncherFix extends Mechanism {

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

    public LauncherFix(LauncherConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }
}
