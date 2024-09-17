package frc.robot.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LauncherCommands {
    public static LauncherFix launcher = Robot.launcher;

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static Command runVelocity(double velocity) {
        return launcher.runVelocityTCFOCrpm(velocity).withName("Launcher.runVelocity");
    }
}
