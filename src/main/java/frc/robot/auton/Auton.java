package frc.robot.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotTelemetry;

public class Auton {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();
    public static boolean trackNote = false;
    public static boolean trackSpeaker = false;
    public static boolean noteIntaked = false;
    public static boolean intakeCheck = false;
    private static boolean autoMessagePrinted = true;
    private static double autonStart = 0;

    public static void setupSelectors() {
        // Config Autos (Uncomment to use)
        // autonChooser.addOption("1 Meter", new PathPlannerAuto("1 Meter Auto")); // Runs full Auto
        // autonChooser.addOption("3 Meter", new PathPlannerAuto("3 Meter Auto")); // Runs full Auto

        autonChooser.addOption(
                "Basic Front 4", new PathPlannerAuto("Basic Front 4")); // Runs full Auto
        autonChooser.addOption("Madtown", new PathPlannerAuto("Madtown")); // Runs full Auto
        autonChooser.addOption(
                "Do Nothing", Commands.print("Do Nothing Auto ran")); // Runs full Auto

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    public Auton() {
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        RobotTelemetry.print("Auton Subsystem Initialized: ");
    }

    public static Command getAutonomousCommand() {
        // return new CharacterizeLauncher(Robot.launcher);
        Command auton = autonChooser.getSelected(); // sees what auto is chosen on shuffleboard
        // Command auton = new PathPlannerAuto("Madtown");
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** Called in RobotPeriodic and displays the duration of the auton command Based on 6328 code */
    public static void printAutoDuration() {
        Command autoCommand = Auton.getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }
}
