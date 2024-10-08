package frc.robot.pilot;

import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.elevator.ElevatorCommands;
import frc.robot.launcher.LauncherCommands;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {
    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double turboModeScalor = 1;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(0);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setTriggersDeadzone(0);
            setTriggersExp(2.0);
            setTriggersScalor(3);
        }
    }

    private PilotConfig config;
    @Getter @Setter private boolean isSlowMode = false;
    @Getter @Setter private boolean isTurboMode = false;
    @Getter @Setter private boolean isFieldOriented = true;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;
        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopTriggers() {
        b().whileTrue(ElevatorCommands.fullExtend());
        x().whileTrue(ElevatorCommands.home());
        y().whileTrue(ElevatorCommands.runElevator(() -> getLeftY()));

        b().whileTrue(LauncherCommands.runVelocity(Robot.getConfig().launcher::getMaxVelocity));
        x().whileTrue(
                        LauncherCommands.runVelocity(
                                () -> -1 * Robot.getConfig().launcher.getMaxVelocity()));
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledTriggers() {};

    /** Setup the Buttons for Test mode. */
    public void setupTestTriggers() {
        // This is just for training, robots may have different buttons during test
        // setupTeleopButtons();
        b().whileTrue(ElevatorCommands.tuneElevator());
    };

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        triggersCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = triggersCurve.calculate(getTwist());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        }
        return ccwPositive;
    }
}
