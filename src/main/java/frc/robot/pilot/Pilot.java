package frc.robot.pilot;

import frc.robot.RobotTelemetry;
import frc.robot.elevator.ElevatorCommands;
import frc.robot.intake.IntakeCommands;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.util.ExpCurve;

public class Pilot extends Gamepad {
    public class PilotConfig {
        public static final String name = "Pilot";
        public static final int port = 0;
        /**
         * in order to run a PS5 controller, you must use DS4Windows to emulate a XBOX controller as
         * well and move the controller to emulatedPS5Port
         */
        public static final boolean isXbox = true;

        public static final int emulatedPS5Port = 4;

        public static final double slowModeScalor = 0.45;
        public static final double turboModeScalor = 1;

        public final double leftStickDeadzone = 0; // TODO: reivew
        public final double leftStickExp = 2.0;
        public final double leftStickScalor = 6;

        public final double triggersDeadzone = 0; // TODO: review
        public final double triggersExp = 2.0;
        public final double triggersScalor = leftStickScalor * 0.5;
        public final double rotationScalor = 0.8; // original was 0.8
    }

    public PilotConfig config;
    private boolean isSlowMode = false;
    private boolean isTurboMode = false;
    private boolean isFieldOriented = true;
    private ExpCurve LeftStickCurve;
    private ExpCurve TriggersCurve;

    /** Create a new Pilot with the default name and port. */
    public Pilot() {
        super(PilotConfig.name, PilotConfig.port, PilotConfig.isXbox, PilotConfig.emulatedPS5Port);
        config = new PilotConfig();

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
        controller.a().whileTrue(IntakeCommands.intake());

        controller.b().whileTrue(ElevatorCommands.fullExtend());
        controller.x().whileTrue(ElevatorCommands.home());

        controller.y().whileTrue(ElevatorCommands.runElevator(() -> controller.getLeftY()));
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {};

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };

    public void setMaxVelocity(double maxVelocity) {
        LeftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        TriggersCurve.setScalar(maxRotationalVelocity);
    }

    public void setSlowMode(boolean isSlowMode) {
        this.isSlowMode = isSlowMode;
    }

    public void setTurboMode(boolean isTurboMode) {
        this.isTurboMode = isTurboMode;
    }

    public void setFieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }

    public boolean getFieldOriented() {
        return isFieldOriented;
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = LeftStickCurve.calculate(-1 * controller.getLeftY());
        // if (isSlowMode) {
        //     fwdPositive *= Math.abs(PilotConfig.slowModeScalor);
        // }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * LeftStickCurve.calculate(controller.getLeftX());
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
            ccwPositive *= Math.abs(PilotConfig.slowModeScalor);
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(PilotConfig.turboModeScalor);
        } else {
            ccwPositive *= config.rotationScalor;
        }
        return ccwPositive;
    }
}
