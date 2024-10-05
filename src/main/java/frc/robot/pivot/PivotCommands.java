package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.pivot.Pivot.PivotConfig;
import java.util.function.DoubleSupplier;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;
    private static PivotConfig config = Robot.config.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(pivot.runHoldPivot().ignoringDisable(true).withName("Pivot.default"));
    }

    // Tune value command?
}
