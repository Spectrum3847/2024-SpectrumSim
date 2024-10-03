package frc.robot;

import frc.robot.leds.LEDsCommands;

/**
 * This class is used for commands that use multiple subsystems and don't directly call a gamepad.
 * This is often command groups such as moving an arm and turning on an intake, etc. In 2023 we
 * called this MechanismCommands.java
 */
public class RobotCommands {
    public static void setupRobotTriggers() {
        // Example trigger, sets the LEDs to orange when the elevator is up
        Robot.elevator.isUp().whileTrue(LEDsCommands.solidOrangeLED());
    }
}
