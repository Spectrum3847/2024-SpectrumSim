package frc.robot.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
//import frc.spectrumLib.swerve.config.SwerveConfig;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Pivot extends Mechanism {

    public static class PivotConfig extends Config {
        /* Pivot constants in motor rotations */
        @Getter private final double maxRotation = 0.96; // 0.967
        @Getter private final double minRotation = 0;

        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        @Getter private final double score = 65;
        @Getter private final double climbHome = 3;
        @Getter private final double home = 1;
        @Getter private final double subwoofer = 81;
        @Getter private final double intoAmp = 78;
        @Getter private final double podium = 53.5;
        @Getter private final double fromAmp = 52;
        @Getter private final double ampWing = 41;
        @Getter private final double intake = 50;
        @Getter private final double manualFeed = 70;

        /* Pivot config settings */
        @Getter private final double zeroSpeed = -0.1;
        /**
        * Percentage of pivot rotation added/removed from vision launching pivot angles (percentage
        * of the CHANGE in angle you set to, not +- the angle you set to) (the actual offset to
        * angles gets bigger as you get farther away)
        */
        @Getter private final double STARTING_OFFSET = 3;
        @Getter private double OFFSET = STARTING_OFFSET; // do not change this line
        @Getter private boolean shortFeed = false;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double threshold = 40;
        @Getter private final double velocityKp = 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        /* Cancoder config settings */
        private final int CANcoderID = 44;
        private final double CANcoderGearRatio = 35.1;
        private double CANcoderOffset = 0;
        private enum CANCoderFeedbackType {
            RemoteCANcoder,
            FusedCANcoder,
            SyncCANcoder,
        }
        private CANCoderFeedbackType pivotFeedbackSource = CANCoderFeedbackType.FusedCANcoder;
            
        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        public PivotConfig() {
            super("Pivot", 41, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            configReverseSoftLimit(minRotation, true);
            configForwardSoftLimit(maxRotation, true);
            configMotionMagic(147000, 161000, 0);
        }

        public void configCANcoderOffset(double CANcoderOffset) {
            this.CANcoderOffset = CANcoderOffset;
        }

        public void modifyMotorConfig(PivotConfig config) {
            config.getTalonConfig().Feedback.FeedbackRemoteSensorID = config.CANcoderID;
            switch (config.pivotFeedbackSource) {
                case RemoteCANcoder:
                    config.getTalonConfig().Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.RemoteCANcoder;
                    break;
                case FusedCANcoder:
                    config.getTalonConfig().Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.FusedCANcoder;
                    break;
                case SyncCANcoder:
                    config.getTalonConfig().Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.SyncCANcoder;
                    break;
            }
            config.getTalonConfig().Feedback.RotorToSensorRatio = config.CANcoderGearRatio;
        }
    }

    private PivotConfig config;
    private CANcoder m_CANcoder;

    public Pivot(PivotConfig config) {
        super(config);
        this.config = config; // unsure if we need this, may delete and test

        if (isAttached()) {
            config.modifyMotorConfig(config); // Modify configuration to use remote CANcoder fused
            motor = TalonFXFactory.createConfigTalon(config.getId(), config.getTalonConfig());
            m_CANcoder = new CANcoder(config.CANcoderID, RobotConfig.CANIVORE);
            CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
            cancoderConfigs.MagnetSensor.MagnetOffset = config.CANcoderOffset;
            cancoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            cancoderConfigs.MagnetSensor.AbsoluteSensorRange =
                    AbsoluteSensorRangeValue.Unsigned_0To1;
            checkMotorResponse(m_CANcoder.getConfigurator().apply(cancoderConfigs));

            simulationInit();
            telemetryInit();
        }
    }

    @Override
    public void periodic() {}

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Percent", null, null);
        }
    }

    // Removed tree map getters

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroPivotRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(() -> config.zeroSpeed), // execute
                        (b) -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Pivot.zeroPivotRoutine");
    }
    
    /** Holds the position of the pivot. */
    public Command runHoldPivot() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Pivot.holdPosition");
                addRequirements(Pivot.this);
            }

            @Override
            public void initialize() {
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                runPosition(() -> holdPosition);
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public boolean pivotHasError() {
        if (isAttached()) {
            return getMotorPosition() > config.maxRotation;
        }
        return false;
    }

    public void checkMotorResponse(StatusCode response) {
        if (!response.isOK()) {
            System.out.println(
                    "Pivot CANcoder ID "
                            + config.CANcoderID
                            + " failed config with error "
                            + response.toString());
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
