package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
    private static Elevator elevator = Robot.elevator;
    private static ElevatorConfig config = Robot.config.elevator;

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static Command runElevator(DoubleSupplier speed) {
        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    public static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    // public static TuneValue ElevatorExtension = new TuneValue("Elevator L", config.fullExtend);

    public static Command fullExtend() {
        return elevator.runPosition(() -> config.fullExtend).withName("Elevator.fullExtend");
    }

    public static Command amp() {
        return elevator.runPosition(config.amp).withName("Elevator.amp");
    }

    public static Command trap() {
        return elevator.runPosition(config.trap).withName("Elevator.trap");
    }

    public static Command home() {
        return elevator.runPosition(config.home).withName("Elevator.home");
    }

    public static Command zero() {
        return elevator.zeroElevatorRoutine().withName("Zero Elevator");
    }

    public static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    public static Command tuneElevator() {
        return elevator.runPosition(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }
}
