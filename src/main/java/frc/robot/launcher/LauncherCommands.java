package frc.robot.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    private static Launcher launcher = Robot.getLauncher();

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static Command runVelocity(DoubleSupplier velocityRPM) {
        return launcher.runVelocityTCFOCrpm(velocityRPM).withName("Launcher.runVelocity");
    }
}
