package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.swerve.config.DefaultConfig.SlotGains;
import lombok.Getter;
import lombok.Setter;

public class SwerveConfig {

    // Physical Config
    private static final double frontWheelBaseInches = 11.875;
    private static final double backWheelBaseInches = 8.375;
    private static final double trueWheelBaseInches = frontWheelBaseInches + backWheelBaseInches;
    private static final double trackWidthInches = 11.875;
    private static final double kDriveGearRatio = 5.35714285714;
    private static final double kSteerGearRatio = 21.4285714286;
    
    // Wheel Positions
    private static final double kFrontLeftXPos = Units.inchesToMeters(frontWheelBaseInches);
    private static final double kFrontLeftYPos = Units.inchesToMeters(trackWidthInches);
    private static final double kFrontRightXPos = Units.inchesToMeters(frontWheelBaseInches);
    private static final double kFrontRightYPos = Units.inchesToMeters(-trackWidthInches);
    private static final double kBackLeftXPos = Units.inchesToMeters(-backWheelBaseInches);
    private static final double kBackLeftYPos = Units.inchesToMeters(trackWidthInches);
    private static final double kBackRightXPos = Units.inchesToMeters(-backWheelBaseInches);
    private static final double kBackRightYPos = Units.inchesToMeters(-trackWidthInches);

    // Angle Offsets: from cancoder Absolute Position No Offset, opposite sign
    private static final double kFrontLeftCANcoderOffset = 0.045166; // 0.044922
    private static final double kFrontRightCANncoderOffset = 0.224609; // 0.224365
    private static final double kBackLeftCANcoderOffset = -0.305176; // -0.304199
    private static final double kBackRightCANcoderOffset = -0.296875; // -0.296631

    @Getter private final double simLoopPeriod = 0.005; // 5 ms
    @Getter @Setter private double robotWidth = Units.inchesToMeters(29.5);
    @Getter @Setter private double robotLength = Units.inchesToMeters(29.5);

    @Getter @Setter private double maxAngularRate = 1.5 * Math.PI; // rad/s
    @Getter @Setter private double deadband = 0.1;

    // -----------------------------------------------------------------------
    // Rotation Controller Constants
    // -----------------------------------------------------------------------
    @Getter private double maxAngularVelocity = 2 * Math.PI; // rad/s
    @Getter private double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2); // rad/s^2
    @Getter private double kPRotationController = 8.0;
    @Getter private double kIRotationController = 2.5;
    @Getter private double kDRotationController = 0.3;
    @Getter private double rotationTolerance = (Math.PI / 360); // rads

    @Getter private double kPHoldController = 10.0;
    @Getter private double kIHoldController = 0.0;
    @Getter private double kDHoldController = 0.0;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    @Getter private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    @Getter private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @Getter
    private Slot0Configs steerGains = new SlotGains(100, 0, 0, 0, 0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @Getter
    private Slot0Configs driveGains = new SlotGains(8, 0, 0.1, 0, 0.8);
    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    @Getter private ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    @Getter private ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    @Getter @Setter private double slipCurrentA = 80;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    @Getter private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();

    @Getter
    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a relatively
                                    // low
                                    // stator current limit to help avoid brownouts without
                                    // impacting performance.
                                    .withStatorCurrentLimit(60)
                                    .withStatorCurrentLimitEnable(true));

    @Getter private CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    @Getter private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    @Getter @Setter private double speedAt12VoltsMps = 6;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    @Getter private double coupleRatio = 3.5;

    @Getter @Setter private double driveGearRatio = kDriveGearRatio;
    @Getter @Setter private double steerGearRatio = kSteerGearRatio;

    @Getter @Setter
    private double wheelRadiusInches =
            2.167; // Estimated at first, then fudge-factored to make odom match record

    @Getter @Setter private boolean steerMotorReversed = true;
    @Getter @Setter private boolean invertLeftSide = false;
    @Getter @Setter private boolean invertRightSide = true;

    @Getter @Setter private String canBusName = "*";
    @Getter private int pigeonId = 1;

    // These are only used for simulation
    @Getter private double steerInertia = 0.00001;
    @Getter private double driveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    @Getter private double steerFrictionVoltage = 0.25;
    @Getter private double driveFrictionVoltage = 0.25;

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    @Getter private SwerveModuleConstantsFactory constantCreator;

    // Front Left
    @Getter private int frontLeftDriveMotorId = 5;
    @Getter private int frontLeftSteerMotorId = 4;
    @Getter private int frontLeftEncoderId = 2;
    @Getter private double frontLeftEncoderOffset = -0.83544921875;

    @Getter private double frontLeftXPosInches = 10.5;
    @Getter private double frontLeftYPosInches = 10.5;

    // Front Right
    @Getter private int frontRightDriveMotorId = 7;
    @Getter private int frontRightSteerMotorId = 6;
    @Getter private int frontRightEncoderId = 3;
    @Getter private double frontRightEncoderOffset = -0.15234375;

    @Getter private double frontRightXPosInches = 10.5;
    @Getter private double frontRightYPosInches = -10.5;

    // Back Left
    @Getter private int backLeftDriveMotorId = 1;
    @Getter private int backLeftSteerMotorId = 0;
    @Getter private int backLeftEncoderId = 0;
    @Getter private double backLeftEncoderOffset = -0.4794921875;

    @Getter private double backLeftXPosInches = -10.5;
    @Getter private double backLeftYPosInches = 10.5;

    // Back Right
    @Getter private int backRightDriveMotorId = 3;
    @Getter private int backRightSteerMotorId = 2;
    @Getter private int backRightEncoderId = 1;
    @Getter private double backRightEncoderOffset = -0.84130859375;

    @Getter private double kBackRightXPosInches = -10.5;
    @Getter private double kBackRightYPosInches = -10.5;

    @Getter private SwerveModuleConstants frontLeft;
    @Getter private SwerveModuleConstants frontRight;
    @Getter private SwerveModuleConstants backLeft;
    @Getter private SwerveModuleConstants backRight;

    public SwerveModuleConstants[] getModules() {
        return new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        drivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANbusName(canBusName)
                        .withPigeon2Id(pigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        constantCreator =
                new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(driveGearRatio)
                        .withSteerMotorGearRatio(steerGearRatio)
                        .withWheelRadius(wheelRadiusInches)
                        .withSlipCurrent(slipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                        .withSteerInertia(steerInertia)
                        .withDriveInertia(driveInertia)
                        .withSteerFrictionVoltage(steerFrictionVoltage)
                        .withDriveFrictionVoltage(driveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(coupleRatio)
                        .withSteerMotorInverted(steerMotorReversed)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withCANcoderInitialConfigs(cancoderInitialConfigs);

        frontLeft =
                constantCreator.createModuleConstants(
                        frontLeftSteerMotorId,
                        frontLeftDriveMotorId,
                        frontLeftEncoderId,
                        kFrontLeftCANcoderOffset,
                        Units.inchesToMeters(kFrontLeftXPos),
                        Units.inchesToMeters(kFrontLeftYPos),
                        invertLeftSide);

        frontRight =
                constantCreator.createModuleConstants(
                        frontRightSteerMotorId,
                        frontRightDriveMotorId,
                        frontRightEncoderId,
                        kFrontRightCANncoderOffset,
                        Units.inchesToMeters(kFrontRightXPos),
                        Units.inchesToMeters(kFrontRightYPos),
                        invertRightSide);

        backLeft =
                constantCreator.createModuleConstants(
                        backLeftSteerMotorId,
                        backLeftDriveMotorId,
                        backLeftEncoderId,
                        kBackLeftCANcoderOffset,
                        Units.inchesToMeters(kBackLeftXPos),
                        Units.inchesToMeters(kBackLeftYPos),
                        invertLeftSide);

        backRight =
                constantCreator.createModuleConstants(
                        backRightSteerMotorId,
                        backRightDriveMotorId,
                        backRightEncoderId,
                        kBackRightCANcoderOffset,
                        Units.inchesToMeters(kBackRightXPos),
                        Units.inchesToMeters(kBackRightYPos),
                        invertRightSide);

        return this;
    }

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = frontLeft;
        frontRightEncoderOffset = frontRight;
        backLeftEncoderOffset = backLeft;
        backRightEncoderOffset = backRight;
        return updateConfig();
    }
}
