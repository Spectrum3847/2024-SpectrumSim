package frc.robot.pilot;

import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.elevator.ElevatorCommands;
import frc.robot.intake.IntakeCommands;
import frc.robot.launcher.LauncherCommands;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.util.ExpCurve;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {
    public static class PilotConfig {
        @Getter final String name = "Pilot";
        @Getter final int port = 0;
        /**
         * in order to run a PS5 controller, you must use DS4Windows to emulate a XBOX controller as
         * well and move the controller to emulatedPS5Port
         */
        @Getter final boolean isXbox = true;

        @Getter final int emulatedPS5Port = 4;

        @Getter @Setter double slowModeScalor = 0.45;
        @Getter @Setter double turboModeScalor = 1;

        @Getter @Setter double leftStickDeadzone = 0; // TODO: reivew
        @Getter @Setter double leftStickExp = 2.0;
        @Getter @Setter double leftStickScalor = 6;

        @Getter @Setter double triggersDeadzone = 0; // TODO: review
        @Getter @Setter double triggersExp = 2.0;
        @Getter @Setter double triggersScalor = leftStickScalor * 0.5;
        @Getter @Setter double rotationScalor = 0.8; // original was 0.8
    }

    private PilotConfig config;
    @Getter @Setter private boolean isSlowMode = false;
    @Getter @Setter private boolean isTurboMode = false;
    @Getter @Setter private boolean isFieldOriented = true;
    private ExpCurve LeftStickCurve;
    private ExpCurve TriggersCurve;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config.getName(), config.getPort(), config.isXbox(), config.getEmulatedPS5Port());
        this.config = config;

        // Curve objects that we use to configure the controller axis ojbects
        LeftStickCurve =
                new ExpCurve(
                        config.leftStickExp, 0, config.leftStickScalor, config.leftStickDeadzone);
        TriggersCurve =
                new ExpCurve(config.triggersExp, 0, config.triggersScalor, config.triggersDeadzone);

        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopButtons() {
        xbox.a().whileTrue(IntakeCommands.intake());

        xbox.b().whileTrue(ElevatorCommands.fullExtend());
        xbox.x().whileTrue(ElevatorCommands.home());
        xbox.y().whileTrue(ElevatorCommands.runElevator(() -> xbox.getLeftY()));

        xbox.b().whileTrue(LauncherCommands.runVelocity(Robot.config.launcher::getMaxVelocity));
        xbox.x()
                .whileTrue(
                        LauncherCommands.runVelocity(
                                () -> -1 * Robot.config.launcher.getMaxVelocity()));
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {};

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        // setupTeleopButtons();
        xbox.b().whileTrue(ElevatorCommands.tuneElevator());
    };

    public void setMaxVelocity(double maxVelocity) {
        LeftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        TriggersCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = LeftStickCurve.calculate(-1 * xbox.getLeftY());
        // if (isSlowMode) {
        //     fwdPositive *= Math.abs(PilotConfig.slowModeScalor);
        // }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * LeftStickCurve.calculate(xbox.getLeftX());
        // if (isSlowMode) {
        //     leftPositive *= Math.abs(PilotConfig.slowModeScalor);
        // }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = TriggersCurve.calculate(getTwist());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= config.rotationScalor;
        }
        return ccwPositive;
    }
}
