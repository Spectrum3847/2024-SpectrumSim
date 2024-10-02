package frc.robot.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    public static Launcher launcher = Robot.launcher;

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static Command runVelocity(DoubleSupplier velocity) {
        return launcher.runVelocityTCFOCrpm(velocity).withName("Launcher.runVelocity");
    }
}
