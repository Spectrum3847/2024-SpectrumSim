// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotTelemetry;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private SwerveConfig config;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private RotationController rotationController;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
            new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveConfig config) {
        super(config.DrivetrainConstants, config.getModules());
        this.config = config;
        configurePathPlanner();

        rotationController = new RotationController(config);

        if (Utils.isSimulation()) {
            startSimThread();
        }
        RobotTelemetry.print(
                getName() + " Subsystem Initialized: " + config.kFrontLeftEncoderOffset);
    }

    @Override
    public void periodic() {
        setDriverPerspective();
    }

    public double getRotationControl(double goalRadians) {
        return rotationController.calculate(goalRadians, getRotationRadians());
    }

    public void resetRotationController() {
        rotationController.reset(getRotationRadians());
    }

    public Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    public double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        seedFieldRelative(
                new Pose2d(
                        Units.feetToMeters(27),
                        Units.feetToMeters(27 / 2),
                        BlueAlliancePerspectiveRotation));
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) ->
                        this.setControl(
                                AutoRequest.withSpeeds(
                                        speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        config.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () ->
                        DriverStation.getAlliance().orElse(Alliance.Blue)
                                == Alliance.Red, // Assume the path needs to be flipped for Red vs
                // Blue, this is normally
                // the case
                this); // Subsystem for requirements
    }

    // This allows us to keep the robot pose on the sim field
    // May need to make this only change the pose if we are in Sim
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;

        if (pose.getX() < 0) {
            seedFieldRelative(new Pose2d(new Translation2d(0, pose.getY()), pose.getRotation()));
        }

        return getState().Pose;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setDriverPerspective() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            (allianceColor) -> {
                                this.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? RedAlliancePerspectiveRotation
                                                : BlueAlliancePerspectiveRotation);
                                hasAppliedOperatorPerspective = true;
                            });
        }
    }
}
