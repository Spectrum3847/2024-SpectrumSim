package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberCommands {
    private static Climber climber = Robot.climber;
    private static ClimberConfig config = Robot.climber.config;
    
    public static void setupDefaultCommand() {
        climber.setDefaultCommand(
                climber.holdPosition().ignoringDisable(true).withName("Climber.default"));
    }

    public static void fullExtend() {
        climber.runPosition(config::getFullExtend).withName("Climber.fullExtend");
    }

    public static void home() {
        climber.runPosition(config::getHome).withName("Climber.home");
    }

    public static void topClimb() {
        climber.runPosition(config::getTopClimb).withName("Climber.topClimb");
    }

    public static void midClimb() {
        climber.runPosition(config::getMidClimb).withName("Climber.midClimb");
    }

    public static void botClimb() {
        climber.runPosition(config::getBotClimb).withName("Climber.botClimb");
    }

    public static void safeClimb() {
        climber.runPosition(config::getSafeClimb).withName("Climber.safeClimb");
    }

    public static void coastMode() {
        climber.coastMode().withName("Climber.CoastMode");
    }

    public static void ensureBrakeMode() {
        climber.ensureBrakeMode().withName("Climber.BrakeMode");
    }
}
