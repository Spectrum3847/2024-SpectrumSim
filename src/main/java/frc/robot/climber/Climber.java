package frc.robot.climber;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.*;

public class Climber extends Mechanism {
    public static class ClimberConfig extends Config {
        /* Climber constants in rotations */
        @Getter private final double maxRotation = 104;
        @Getter private final double minRotation = -1;

        /* Climber positions in percent (0 - 100) of full rotation */
        @Getter private double fullExtend = 100;
        @Getter private double home = 0;

        @Getter private double topClimb = 100;
        @Getter private double midClimb = 74;
        @Getter private double safeClimb = 60;
        @Getter private double botClimb = 0;

        /* Climber Percentage Output */
        @Getter private double raisePercentage = 0.2;
        @Getter private double lowerPercentage = -0.2;

        /* Climber config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 1.3; // 20 FOC // 10 Regular
        @Getter private final double positionKv = 0.013; // .12 FOC // .15 regular
        @Getter private final double currentLimit = 80;
        @Getter private final double statorCurrentLimit = 200;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double threshold = 80;

        public ClimberConfig() {
            super("Climber", 53, RobotConfig.CANIVORE);
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(14700, 16100, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxRotation, true);
            configReverseSoftLimit(minRotation, true);
            configNeutralBrakeMode(true);
            // configMotionMagicPosition(0.12);
            configClockwise_Positive();
        }
    }

    private ClimberConfig config;
    //private ClimberSim sim;

    public Climber(ClimberConfig config) {
        super(config); // unsure if we need this, may delete and test
        this.config = config;

        // simulationInit();
        // simulationPeriodic();

        RobotTelemetry.print(getName() + " Subsystem Initialized");
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
            builder.addDoubleProperty("Position", this::getMotorPosition, null);
            builder.addDoubleProperty(
                    "Percent Angle", () -> getMotorPosition() / config.maxRotation * 100, null);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the climber. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Climber.holdPosition");
                addRequirements(Climber.this);
            }

            @Override
            public void initialize() {
                stop();
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                double currentPosition = getMotorPosition();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(() -> holdPosition);
                } else {
                    DriverStation.reportError(
                            "ClimberHoldPosition tried to go too far away from current position. Current Position: "
                                    + currentPosition
                                    + " || Hold Position "
                                    + holdPosition,
                            false);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command zeroClimberRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        (b) -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this)
                .withName("Climber.zeroClimberRoutine"); // requirement
    }

    // didn't keep percentToRotation methods, could maybe add to Mechanism.java

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

    // public void simulationInit() {
    //     if (isAttached()) {
    //         sim = new ClimberSim(motor.getSimState(), RobotSim.leftView);
    //     }
    // }

    // @Override
    // public void simulationPeriodic() {
    //     if (isAttached()) {
    //         sim.simulationPeriodic();
    //     }

    // }
}
