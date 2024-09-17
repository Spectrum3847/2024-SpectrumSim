package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConfig.DEFAULT;

public class LinearSim {
    private ElevatorSim elevatorSim;

    private final MechanismRoot2d m_mech2dRoot;
    private final MechanismLigament2d m_elevatorMech2d;

    public class LinearConfig {
        public int numMotors = 1;
        public double kElevatorGearing = 1.0 / 10.0;
        public double kCarriageMass = 5.0;
        public double kElevatorDrumRadius = 0.0254;
        public double kMinElevatorHeight = 0.0;
        public double kMaxElevatorHeight = 2.0;

        // Display Config
        public double angle = 90;
        public Color8Bit color = new Color8Bit(Color.kPurple);
        public double lineWidth = 2;
        public double maxHeight = 30;
    }

    public LinearSim(Mechanism2d mech, TalonFXSimState linearMotorSim, String name) {
        LinearConfig config = new LinearConfig();

        this.elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.numMotors),
                        config.kElevatorGearing,
                        config.kCarriageMass,
                        config.kElevatorDrumRadius,
                        config.kMinElevatorHeight,
                        config.kMaxElevatorHeight,
                        true,
                        0,
                        VecBuilder.fill(0.01));

        m_mech2dRoot = mech.getRoot("Elevator Root", 10, 0);

        m_elevatorMech2d =
                m_mech2dRoot.append(
                        new MechanismLigament2d(
                                "Elevator",
                                Units.metersToInches(elevatorSim.getPositionMeters()),
                                config.angle));
    }

    public void simulationPeriodic(TalonFXSimState linearMotorSim) {
        elevatorSim.setInput(linearMotorSim.getMotorVoltage());
        elevatorSim.update(TimedRobot.kDefaultPeriod);

        linearMotorSim.setRotorVelocity(
                elevatorSim.getVelocityMetersPerSecond()
                        * DEFAULT.Intake.Arm.ratio
                        / (2.0 * Math.PI));
        linearMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() * 2.0 * Math.PI);

        m_elevatorMech2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
    }
}
