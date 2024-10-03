package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotConfig.RobotType;
import frc.robot.pilot.PilotCommands;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
    static Swerve swerve = Robot.swerve;
    static SwerveConfig config = Robot.config.swerve;

    public static void setupDefaultCommand(RobotType robotType) {
        if (robotType == RobotType.PM) {
            // Use this to set a different command based on robotType
            // Robot.swerve.setDefaultCommand(PhotonPilotCommands.pilotDrive());
            // return;
        }
        Robot.swerve.setDefaultCommand(PilotCommands.pilotDrive());
    }

    private static SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(config.kSpeedAt12VoltsMps * config.kDeadband)
                    .withRotationalDeadband(config.kMaxAngularRate * config.kDeadband)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Uses m/s and rad/s
    public static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return Robot.swerve
                .applyRequest(
                        () ->
                                fieldCentricDrive
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .ignoringDisable(true);
    }

    public static Command test() {
        swerve.getRotation();
        return new WaitCommand(2);
    }
}
