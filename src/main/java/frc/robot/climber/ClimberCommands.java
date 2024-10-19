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
        return climber.moveToPosePercentage(config::getFullExtend).withName("Climber.fullExtend");
    }

    public static Command home() {
        return climber.moveToPosePercentage(config::getHome).withName("Climber.home");
    }

    public static Command topClimb() {
        return climber.moveToPosePercentage(config::getTopClimb).withName("Climber.topClimb");
    }

    public static Command midClimb() {
        return climber.moveToPosePercentage(config::getMidClimb).withName("Climber.midClimb");
    }

    public static Command botClimb() {
        return climber.moveToPosePercentage(config::getBotClimb).withName("Climber.botClimb");
    }

    public static Command safeClimb() {
        return climber.moveToPosePercentage(config::getSafeClimb).withName("Climber.safeClimb");
    }

    public static Command coastMode() {
        return climber.coastMode().withName("Climber.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return climber.ensureBrakeMode().withName("Climber.BrakeMode");
    }

    public static Command tuneClimber() {
        return climber.moveToPosePercentage(new TuneValue("Tune Climber", 0).getSupplier())
                .withName("Climber.Tune");
    }
}
