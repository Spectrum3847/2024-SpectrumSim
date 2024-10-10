package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoPaths {
    public static Command Madtown() {
        return AutonCommands.pathfindingCommandToPose(
                        1.410, 3.650, -180, 6,
                        6) // moves robot to auto starting pos (cords from pathplanner)
                .andThen(Commands.waitSeconds(1)) // waits before starting desired auto
                .andThen(AutoBuilder.buildAuto("Madtown")); // actually runs auto
    }
}
