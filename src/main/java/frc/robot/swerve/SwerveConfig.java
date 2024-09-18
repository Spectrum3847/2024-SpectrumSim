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
import edu.wpi.first.math.util.Units;

public class SwerveConfig {

    public double kMaxAngularRate = 1.5 * Math.PI; // rad/s
    public double kDeadband = 0.1;

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public Slot0Configs steerGains =
            new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public Slot0Configs driveGains =
            new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public double kSlipCurrentA = 150.0;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    public TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    public TalonFXConfiguration steerInitialConfigs =
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
    public CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public Pigeon2Configuration pigeonConfigs = null;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public double kSpeedAt12VoltsMps = 4.70;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public double kCoupleRatio = 3.5;

    public double kDriveGearRatio = 7.363636364;
    public double kSteerGearRatio = 15.42857143;
    public double kWheelRadiusInches =
            2.167; // Estimated at first, then fudge-factored to make odom match record

    public boolean kSteerMotorReversed = true;
    public boolean kInvertLeftSide = false;
    public boolean kInvertRightSide = true;

    public String kCANbusName = "rio";
    public int kPigeonId = 1;

    // These are only used for simulation
    public double kSteerInertia = 0.00001;
    public double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public double kSteerFrictionVoltage = 0.25;
    public double kDriveFrictionVoltage = 0.25;

    public SwerveDrivetrainConstants DrivetrainConstants;

    public SwerveModuleConstantsFactory ConstantCreator;

    // Front Left
    public int kFrontLeftDriveMotorId = 5;
    public int kFrontLeftSteerMotorId = 4;
    public int kFrontLeftEncoderId = 2;
    public double kFrontLeftEncoderOffset = -0.83544921875;

    public double kFrontLeftXPosInches = 10.5;
    public double kFrontLeftYPosInches = 10.5;

    // Front Right
    public int kFrontRightDriveMotorId = 7;
    public int kFrontRightSteerMotorId = 6;
    public int kFrontRightEncoderId = 3;
    public double kFrontRightEncoderOffset = -0.15234375;

    public double kFrontRightXPosInches = 10.5;
    public double kFrontRightYPosInches = -10.5;

    // Back Left
    public int kBackLeftDriveMotorId = 1;
    public int kBackLeftSteerMotorId = 0;
    public int kBackLeftEncoderId = 0;
    public double kBackLeftEncoderOffset = -0.4794921875;

    public double kBackLeftXPosInches = -10.5;
    public double kBackLeftYPosInches = 10.5;

    // Back Right
    public int kBackRightDriveMotorId = 3;
    public int kBackRightSteerMotorId = 2;
    public int kBackRightEncoderId = 1;
    public double kBackRightEncoderOffset = -0.84130859375;

    public double kBackRightXPosInches = -10.5;
    public double kBackRightYPosInches = -10.5;

    public SwerveModuleConstants FrontLeft;
    public SwerveModuleConstants FrontRight;
    public SwerveModuleConstants BackLeft;
    public SwerveModuleConstants BackRight;

    public SwerveModuleConstants[] getModules() {
        return new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        DrivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANbusName(kCANbusName)
                        .withPigeon2Id(kPigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        ConstantCreator =
                new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withCANcoderInitialConfigs(cancoderInitialConfigs);

        FrontLeft =
                ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId,
                        kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches),
                        Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);

        FrontRight =
                ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId,
                        kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches),
                        kInvertRightSide);

        BackLeft =
                ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId,
                        kBackLeftDriveMotorId,
                        kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches),
                        Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);

        BackRight =
                ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId,
                        kBackRightDriveMotorId,
                        kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches),
                        Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

        return this;
    }

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        kFrontLeftEncoderOffset = frontLeft;
        kFrontRightEncoderOffset = frontRight;
        kBackLeftEncoderOffset = backLeft;
        kBackRightEncoderOffset = backRight;
        return updateConfig();
    }
}
