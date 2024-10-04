package frc.robot.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
        /* Cancoder config */
        private final int CANcoderID = 44;
        private final double CANcoderGearRatio = 35.1;
        private double CANcoderOffset = 0;

        /* Pivot constants in motor rotations */
        @Getter public final double maxRotation = 0.96; // 0.967
        @Getter public final double minRotation = 0;

        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        @Getter public final double score = 65;
        @Getter public final double climbHome = 3;
        @Getter public final double home = 1;
        @Getter public final double subwoofer = 81;
        @Getter public final double intoAmp = 78;
        @Getter public final double podium = 53.5;
        @Getter public final double fromAmp = 52;
        @Getter public final double ampWing = 41;
        @Getter public final double intake = 50;
        @Getter public final double manualFeed = 70;
            
        // Need to add auto launching positions when auton is added

        /**
        * Percentage of pivot rotation added/removed from vision launching pivot angles (percentage
        * of the CHANGE in angle you set to, not +- the angle you set to) (the actual offset to
        * angles gets bigger as you get farther away)
        */

        public final double STARTING_OFFSET = 3;

        public double OFFSET = STARTING_OFFSET; // do not change this line

        public boolean shortFeed = false;

        /* Pivot config values */
        public double currentLimit = 30;
        public double torqueCurrentLimit = 100;
        public double threshold = 40;
        public double velocityKp = 186; // 200 w/ 0.013 good
        public double velocityKv = 0.018;
        public double velocityKs = 0;

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
    }

    private PivotConfig config;
    private CANcoder m_CANcoder;

    public Pivot(PivotConfig config) {
        super(config);
        this.config = config; // unsure if we need this, may delete and test

        if (isAttached()) {
            modifyMotorConfig(swerveConfig); // Modify configuration to use remote CANcoder fused
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
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
                        () -> setPercentOutput(config.zeroSpeed), // execute
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

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
