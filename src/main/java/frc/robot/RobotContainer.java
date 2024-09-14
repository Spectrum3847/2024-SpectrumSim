// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TunerConstants;
import frc.robot.vision.VisionSystem;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);

    // Swerve setup based on
    // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java
    private final Swerve drivetrain = TunerConstants.DriveTrain;
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            RobotConfig.Drivetrain.kMaxSpeed * RobotConfig.Drivetrain.kDeadband)
                    .withRotationalDeadband(
                            RobotConfig.Drivetrain.kMaxAngularRate
                                    * RobotConfig.Drivetrain.kDeadband)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Vision
    private final VisionSystem visionSystem = new VisionSystem(() -> drivetrain.getState().Pose);

    // Subsystems
    private final Intake intake = new Intake();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Drive
        drivetrain.setDefaultCommand(
                drivetrain
                        .applyRequest(
                                () ->
                                        drive.withVelocityX(
                                                        -controller.getLeftY()
                                                                * RobotConfig.Drivetrain.kMaxSpeed)
                                                .withVelocityY(
                                                        -controller.getLeftX()
                                                                * RobotConfig.Drivetrain.kMaxSpeed)
                                                .withRotationalRate(
                                                        -controller.getRightX()
                                                                * RobotConfig.Drivetrain
                                                                        .kMaxAngularRate))
                        .ignoringDisable(true));

        // Move arm
        controller.rightBumper().onTrue(new InstantCommand(() -> intake.deploy(), intake));
        controller.rightBumper().onFalse(new InstantCommand(() -> intake.retract(), intake));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
