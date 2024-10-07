package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.pivot.Pivot.PivotConfig;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;
    private static PivotConfig config = Robot.config.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(
                pivot.runHoldPivot().ignoringDisable(true).withName("Pivot.default"));
    }

    // missing distance based pivot commands

    public static Command runPivot(DoubleSupplier speed) {
        return pivot.runPercentage(speed).withName("Pivot.runPivot");
    }

    public static Command home() {
        return pivot.moveToPosePercentage(config::getHome).withName("Pivot.home");
    }

    public static Command climbHome() {
        return pivot.moveToPosePercentage(config::getClimbHome).withName("Pivot.climbHome");
    }

    public static Command manualFeed() {
        return pivot.moveToPosePercentage(config::getManualFeed).withName("Pivot.manualFeed");
    }

    /* Scoring positions */

    public static Command subwoofer() {
        return pivot.moveToPosePercentage(config::getSubwoofer).withName("Pivot.subwoofer");
    }

    public static Command podium() {
        return pivot.moveToPosePercentage(config::getPodium).withName("Pivot.podium");
    }

    public static Command ampWing() {
        return pivot.moveToPosePercentage(config::getAmpWing).withName("Pivot.ampWing");
    }

    public static Command fromAmp() {
        return pivot.moveToPosePercentage(config::getFromAmp).withName("Pivot.fromAmp");
    }

    public static Command intoAmp() {
        return pivot.moveToPosePercentage(config::getIntoAmp).withName("Pivot.intoAmp");
    }

    // missing auton pivot commands, add when auton is added

    public static Command intake() {
        return pivot.moveToPosePercentage(config::getIntake).withName("Pivot.intake");
    }

    public static Command coastMode() {
        return pivot.coastMode().withName("Pivot.CoastMode");
    }

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stop");
    }

    public static Command ensureBrakeMode() {
        return pivot.ensureBrakeMode().withName("Pivot.BrakeMode");
    }

    /** increase vision shots by 0.5 percent */
    public static Command increaseOffset() {
        return pivot.runOnce(pivot::increaseOffset)
                .withName("Pivot.increaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** decrease vision shots by 0.5 percent */
    public static Command decreaseOffset() {
        return pivot.runOnce(pivot::decreaseOffset)
                .withName("Pivot.decreaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** reset fudge factor to 0 */
    public static Command resetOffset() {
        return pivot.runOnce(pivot::resetOffset)
                .withName("Pivot.resetFudgeFactor")
                .ignoringDisable(true);
    }

    public static Command switchFeedSpot() {
        return pivot.runOnce(pivot::switchFeedSpot)
                .withName("Pivot.switchFeedSpot")
                .ignoringDisable(true);
    }

    // Tune value command
    public static Command tunePivot() {
        return pivot.moveToPosePercentage(new TuneValue("Tune Pivot", 0).getSupplier())
                .withName("Pivot.Tune");
    }
}
