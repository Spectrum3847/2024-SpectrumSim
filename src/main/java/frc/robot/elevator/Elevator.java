package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;

public class Elevator extends Mechanism {
    public static class ElevatorConfig extends Config {
        /* Elevator constants in rotations */
        public double maxHeight = 29.8;
        public double minHeight = 0;

        /* Elevator positions in rotations */
        public double fullExtend = maxHeight;
        public double home = minHeight;
        public double amp = 15;
        public double trap = 5;

        /* Elevator config settings */
        public final double zeroSpeed = -0.2;
        public final double positionKp = 0.86; // 20 FOC // 10 Regular
        public final double positionKv = 0.13; // .12 FOC // .15 regular
        public double currentLimit = 20;
        public final double torqueCurrentLimit = 100;
        public final double threshold = 20;

        /* Sim properties */
        public double kElevatorGearing = 5;
        public double kCarriageMass = 1;
        public double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        public double initialX = 0.5;
        public double initialY = 0.0;
        public double angle = 180 - 72;

        public ElevatorConfig() {
            super("Elevator", 52, RobotConfig.CANIVORE);
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(700, 900, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxHeight, true);
            configReverseSoftLimit(minHeight, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, threshold, true);
        }
    }

    private ElevatorConfig config;
    private ElevatorSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config; // unsure if we need this, may delete and test
        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
        simulationInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized: ");
    }

    @Override
    public void periodic() {}

    /* Check Elevator States */
    // Is Amp Height
    public Boolean isAtAmpHeight() {
        return getMotorPosition() > config.amp * 0.8;
    }

    public boolean isElevatorUp() {
        return getMotorPosition() >= 5;
    }

    //--------------------------------------------------------------------------------
    // Custom Commands
    //--------------------------------------------------------------------------------

    /** Holds the position of the elevator. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elevator.holdPosition");
                addRequirements(Elevator.this);
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
                    setMMPosition(holdPosition);
                } else {
                    stop();
                    DriverStation.reportError(
                            "ElevatorHoldPosition tried to go too far away from current position. Current Position: "
                                    + currentPosition
                                    + " || Hold Position: "
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

    public Command zeroElevatorRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config.zeroSpeed), // execute
                        (b) -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elevator.zeroElevatorRoutine");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) { // Only run simulation if it's attached
            sim = new ElevatorSim(motor.getSimState(), RobotSim.leftView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) { // Only run if it's attached
            sim.simulationPeriodic();
        }
    }

    class ElevatorSim extends LinearSim {
        public ElevatorSim(TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
            super(
                    new LinearConfig(
                                    config.initialX,
                                    config.initialY,
                                    config.kElevatorGearing,
                                    config.kElevatorDrumRadiusMeters)
                            .setAngle(config.angle),
                    mech,
                    elevatorMotorSim,
                    config.name);
        }
    }
}
