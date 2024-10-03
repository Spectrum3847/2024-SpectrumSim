package frc.spectrumLib.gamepads;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.ExpCurve;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public abstract class Gamepad extends SpectrumController implements Subsystem {

    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();
    private boolean configured =
            false; // Used to determine if we detected the gamepad is plugged and we have configured
    // it
    private boolean printed = false; // Used to only print Gamepad Not Deteceted once

    @Getter protected ExpCurve LeftStickCurve;
    @Getter protected ExpCurve RightStickCurve;
    @Getter protected ExpCurve TriggersCurve;

    public static class Config {
        @Getter private String name;
        @Getter private int port; // USB port on the DriverStation app

        // A configured value to say if we should use this controller on this robot
        @Getter @Setter private Boolean attached;

        /**
         * in order to run a PS5 controller, you must use DS4Windows to emulate a XBOX controller as
         * well and move the controller to emulatedPS5Port
         */
        @Getter @Setter boolean isXbox = true;

        @Getter @Setter int emulatedPS5Port;

        @Getter @Setter double leftStickDeadzone = 0.0; // TODO: reivew
        @Getter @Setter double leftStickExp = 1.0;
        @Getter @Setter double leftStickScalor = 1.0;

        @Getter @Setter double rightStickDeadzone = 0.0; // TODO: reivew
        @Getter @Setter double rightStickExp = 1.0;
        @Getter @Setter double rightStickScalor = 1.0;

        @Getter @Setter double triggersDeadzone = 0.0; // TODO: review
        @Getter @Setter double triggersExp = 1.0;
        @Getter @Setter double triggersScalor = 1.0;

        public Config(String name, int port) {
            this.name = name;
            this.port = port;
        }
    }

    private Config config;
    /**
     * Creates a new Gamepad.
     *
     * @param port The port the gamepad is plugged into
     * @param name The name of the gamepad
     * @param isXbox Xbox or PS5 controller
     * @param emulatedPS5Port emulated port for PS5 controller so we can rumble PS5 controllers.
     */
    public Gamepad(Config config) {
        super(config.getPort(), config.isXbox(), config.getEmulatedPS5Port(), config.getAttached());
        this.config = config;
        // Curve objects that we use to configure the controller axis ojbects
        LeftStickCurve =
                new ExpCurve(
                        config.getLeftStickExp(),
                        0,
                        config.getLeftStickScalor(),
                        config.getLeftStickDeadzone());
        RightStickCurve =
                new ExpCurve(
                        config.getRightStickExp(),
                        0,
                        config.getRightStickScalor(),
                        config.getRightStickDeadzone());
        TriggersCurve =
                new ExpCurve(
                        config.getTriggersExp(),
                        0,
                        config.getTriggersScalor(),
                        config.getTriggersDeadzone());
    }

    @Override
    public void periodic() {
        configure();
    }

    // Configure the pilot controller
    public void configure() {
        if (config.getAttached()) {
            // Detect whether the xbox controller has been plugged in after start-up
            if (!configured) {
                if (!isConnected()) {
                    if (!printed) {
                        Telemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                        printed = true;
                    }
                    return;
                }

                // Configure button bindings once the driver controller is connected
                if (DriverStation.isTest()) {
                    setupTestTriggers();
                } else if (DriverStation.isDisabled()) {
                    setupDisabledTriggers();
                } else {
                    setupTeleopTriggers();
                }
                configured = true;

                Telemetry.print("## " + getName() + ": gamepad is connected ##");
            }
        }
    }

    // Reset the controller configure, should be used with
    // CommandScheduler.getInstance.clearButtons()
    // to reset buttons
    public void resetConfig() {
        configured = false;
        configure();
    }

    /* Zero is stick up, 90 is stick to the left*/
    public Rotation2d getLeftStickDirection() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public double getRightStickMagnitude() {
        double x = getRightX();
        double y = getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get proper stick angles for each alliance
     *
     * @return
     */
    public double chooseCardinalDirections() {
        // hotfix
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return 0;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else {
            return Math.PI; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    // TODO: simplified but untested
    // public double getBlueAllianceStickCardinals() {
    //     double stickAngle = getRightStickDirection().getRadians();

    //     // Normalize angle to be between 0 and 2π
    //     stickAngle = (stickAngle + 2 * Math.PI) % (2 * Math.PI);

    //     // Round to nearest π/4 (45 degrees) and adjust for direction
    //     double aimAngle = Math.round(stickAngle / (Math.PI / 4)) * (Math.PI / 4);

    //     // Wrap angle to -π to π
    //     return MathUtil.angleModulus(aimAngle);
    // }

    /**
     * Flips the stick direction for the red alliance.
     *
     * @return
     */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();

        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return -Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return Math.PI / 4;
        } else {
            return 0; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    // TODO: simplified but untested
    // public double getRedAllianceStickCardinals() {
    //     double aimAngle = getBlueAllianceStickCardinals();
    //     return MathUtil.angleModulus(aimAngle + Math.PI); // Flip the angle
    // }

    // public double getStickSteer(int segments){
    //     double stickAngle = getRightStickDirection().getRadians();
    //     double[] outputs = new double[segments];
    //     for (int i = 0; i < segments; i++){
    //         outputs[i] = i * (2*Math.PI/segments);
    //     }

    // }
    // public double idk (double[] outputs, double stickAngle, int segments){ ///need three
    // parameters?, output array and current stickAngle in Radians, segments wanted
    //     double[] newoutputs = new double[segments]; //create copy of array with the rotated
    // values
    //     for (int i=0; i < segments; i++){ // loops for the length of outputs
    //         newoutputs[i] = outputs[i] + Math.PI/segments; /// shifts circle
    //     }
    //     for (int i=0; i< segments; i++){
    //         if (stickAngle > outputs[i] && stickAngle <= outputs[(i+1)%segments]){ ///if the
    // given stickAngle is between two of the other angles
    //             ///The %segments should account for the circle looping around?
    //             return outputs[(i+1)%segments];// shoudl be the desired angle output
    //         }
    //     }

    // }

    /** Setup modifier bumper and trigger buttons */
    public Trigger noModifers() {
        return noBumpers().and(noTriggers());
    }

    public Trigger noBumpers() {
        return rightBumper().negate().and(leftBumper().negate());
    }

    public Trigger leftBumperOnly() {
        return leftBumper().and(rightBumper().negate());
    }

    public Trigger rightBumperOnly() {
        return rightBumper().and(leftBumper().negate());
    }

    public Trigger bothBumpers() {
        return rightBumper().and(leftBumper());
    }

    public Trigger noTriggers() {
        return leftTrigger(0).negate().and(rightTrigger(0).negate());
    }

    public Trigger leftTriggerOnly() {
        return leftTrigger(0).and(rightTrigger(0).negate());
    }

    public Trigger rightTriggerOnly() {
        return rightTrigger(0).and(leftTrigger(0).negate());
    }

    public Trigger bothTriggers() {
        return leftTrigger(0).and(rightTrigger(0));
    }

    public Trigger leftYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> getLeftY());
    }

    public Trigger leftXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> getLeftX());
    }

    public Trigger rightYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> getRightY());
    }

    public Trigger rightXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> getRightX());
    }

    public Trigger rightStick(double threshold) {
        return new Trigger(
                () -> {
                    return Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold;
                });
    }

    public Trigger leftStick(double threshold) {
        return new Trigger(
                () -> {
                    return Math.abs(getLeftX()) >= threshold || Math.abs(getLeftY()) >= threshold;
                });
    }

    private Trigger axisTrigger(ThresholdType t, double threshold, DoubleSupplier v) {
        return new Trigger(
                () -> {
                    double value = v.getAsDouble();
                    switch (t) {
                        case GREATER_THAN:
                            return value > threshold;
                        case LESS_THAN:
                            return value < threshold;
                        case ABS_GREATER_THAN: // Also called Deadband
                            return Math.abs(value) > threshold;
                        default:
                            return false;
                    }
                });
    }

    public static enum ThresholdType {
        GREATER_THAN,
        LESS_THAN,
        ABS_GREATER_THAN;
    }

    private void rumble(double leftIntensity, double rightIntensity) {
        rumbleController(leftIntensity, rightIntensity);
    }

    /** Command that can be used to rumble the pilot controller */
    public Command rumbleCommand(
            double leftIntensity, double rightIntensity, double durationSeconds) {
        return new RunCommand(() -> rumble(leftIntensity, rightIntensity), this)
                .withTimeout(durationSeconds)
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Run a command while a button/trigger is held down. Also runs a command for a certain timeout
     * when the button/trigger is released.
     *
     * @param trigger
     * @param runCommand
     * @param endCommand
     */
    public void runWithEndSequence(
            Trigger trigger, Command runCommand, Command endCommand, double endTimeout) {
        trigger.whileTrue(runCommand);
        trigger.onFalse(endCommand.withTimeout(endTimeout).withName(endCommand.getName()));
    }

    /**
     * Returns a new Command object that combines the given command with a rumble command. The
     * rumble command has a rumble strength of 1 and a duration of 0.5 seconds. The name of the
     * returned command is set to the name of the given command.
     *
     * @param command the command to be combined with the rumble command
     * @return a new Command object with rumble command
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public abstract void setupTeleopTriggers();

    public abstract void setupDisabledTriggers();

    public abstract void setupTestTriggers();
}
