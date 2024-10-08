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
import lombok.Getter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {
        @Getter private double maxVelocity = 5600;

        /* LeftLauncher config values */
        @Getter private double currentLimit = 60;
        @Getter private double torqueCurrentLimit = 300;
        @Getter private double threshold = 80;
        @Getter private double velocityKp = 6;
        @Getter private double velocityKv = 0.12;
        @Getter private double velocityKs = 0.24;

        /* Sim Configs */
        @Getter private double wheelDiameter = 6.0;

        public LauncherConfig() {
            super("Launcher", 42, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0.0, 0.0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configMotionMagic(51, 205, 0);
        }
    }

    private LauncherConfig config;
    private RollerSim sim;

    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;
        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getMotorPosition, null);
            builder.addDoubleProperty("Velocity", this::getMotorVelocityRPS, null);
        }
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
            super(new RollerConfig(config.wheelDiameter), mech, rollerMotorSim, config.getName());
        }
    }
}
