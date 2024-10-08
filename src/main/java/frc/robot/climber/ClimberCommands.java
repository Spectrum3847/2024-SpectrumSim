package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.climber.Climber.ClimberConfig;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;


public class ClimberCommands {
    private static Climber climber = Robot.getClimber();
    private static ClimberConfig config = Robot.getConfig().climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(holdPosition().ignoringDisable(true).withName("Climber.default"));
    }

    public static Command runClimber(DoubleSupplier speed) {
        return climber.runPercentage(speed).withName("Climber.runClimber");
    }

    public static Command holdPosition() {
        return climber.holdPosition().withName("Climber.holdPosition");
    }

    public static Command fullExtend() {
        return climber.runPosition(config::getFullExtend).withName("Climber.fullExtend");
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

    public static Command tuneClimber() {
        return climber.runPosition(new TuneValue("Tune Climber", 0).getSupplier()).withName("Climber.Tune");
    }
}
