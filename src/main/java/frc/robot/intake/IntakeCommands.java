package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

public class IntakeCommands {
    public static void setupDefaultCommand() {
        Robot.intake.setDefaultCommand(
                new InstantCommand(() -> Robot.intake.retract(), Robot.intake));
    }

    public static Command intake() {
        return new RunCommand(() -> Robot.intake.deploy(), Robot.intake);
    }
}
