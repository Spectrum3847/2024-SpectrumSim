package frc.robot.launcher;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;

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

        /* Sim Configs */
        public double wheelDiameter = 6.0;

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
            configCounterClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    private LauncherConfig config;
    private RollerSim sim;

    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;
        simulationInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized: ");
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getMotorPosition, null);
            builder.addDoubleProperty("Velocity", this::getVelocity, null);
        }
    }

    private double getVelocity() {
        return motor.getVelocity().getValue();
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            sim = new LauncherSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic(0.5, 0.5);
        }
    }

    class LauncherSim extends RollerSim {
        public LauncherSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(new RollerConfig(config.wheelDiameter), mech, rollerMotorSim, config.name);
        }
    }
}
