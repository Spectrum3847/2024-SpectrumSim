package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotConfig.RobotType;
import frc.robot.pilot.PilotCommands;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;

    public static void setupDefaultCommand(RobotType robotType) {
        if (robotType == RobotType.PM) {
            // Use this to set a different command based on robotType
            // Robot.swerve.setDefaultCommand(PhotonPilotCommands.pilotDrive());
            // return;
        }
        swerve.setDefaultCommand(
                PilotCommands.pilotDrive()
                        .withTimeout(0.5)
                        .andThen(PilotCommands.headingLockDrive())
                        .ignoringDisable(true)
                        .withName("SwerveCommands.default"));
    }

    private static SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(config.getSpeedAt12VoltsMps() * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Uses m/s and rad/s
    public static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                () ->
                        fieldCentricDrive
                                .withVelocityX(fwdPositive.getAsDouble())
                                .withVelocityY(leftPositive.getAsDouble())
                                .withRotationalRate(ccwPositive.getAsDouble()));
    }

    public static Command test() {
        swerve.getRotation();
        return new WaitCommand(2);
    }

    public static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    public static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.run(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engague if you are driving x or y.
     */
    public static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()).until(() -> true),
                        drive(
                                velocityX,
                                velocityY,
                                () -> {
                                    if (Field.isBlue()) {
                                        if (swerve.getRobotPose().getX()
                                                <= Field.getFieldLength() / 2) {
                                            return 0;
                                        }
                                    } else {
                                        if (swerve.getRobotPose().getX()
                                                >= Field.getFieldLength() / 2) {}
                                    }
                                    if (velocityX.getAsDouble() == 0
                                            && velocityY.getAsDouble() == 0) {
                                        return 0;
                                    } else {
                                        return swerve.calculateRotationController(
                                                () -> config.getTargetHeading());
                                    }
                                })).withName("Swerve.HeadingLock");
    }

    public static Command reorient(double angleDegrees) {
        return swerve.runOnce(() -> swerve.reorient(angleDegrees)).withName("Serve.reorient");
    }

    public static Command reorientForward() {
        return swerve.runOnce(() -> swerve.reorientForward()).withName("Swerve.reorientForward");
    }

    public static Command reorientLeft() {
        return swerve.runOnce(() -> swerve.reorientLeft()).withName("Swerve.reorientLeft");
    }

    public static Command reorientRight() {
        return swerve.runOnce(() -> swerve.reorientRight()).withName("Swerve.reorientRight");
    }

    public static Command reorientBack() {
        return swerve.runOnce(() -> swerve.reorientBack()).withName("Swerve.reorientBack");
    }

    public static Command cardinalReorient() {
        return swerve.runOnce(() -> swerve.cardinalReorient()).withName("Swerve.cardinalReorient");
    }
}
