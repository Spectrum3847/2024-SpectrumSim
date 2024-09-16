package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig.DefaultConfig;
import frc.robot.pilot.PilotCommands;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
    static Swerve swerve = Robot.swerve;

    public static void setupDefaultCommand() {
        Robot.swerve.setDefaultCommand(PilotCommands.pilotDrive());
    }

    private static SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            DefaultConfig.Drivetrain.kMaxSpeed * DefaultConfig.Drivetrain.kDeadband)
                    .withRotationalDeadband(
                            DefaultConfig.Drivetrain.kMaxAngularRate
                                    * DefaultConfig.Drivetrain.kDeadband)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
}
