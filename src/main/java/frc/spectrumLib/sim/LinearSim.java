package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LinearSim {
    private ElevatorSim elevatorSim;

    private final MechanismRoot2d m_mech2dRoot;
    private final MechanismLigament2d m_elevatorMech2d;
    LinearConfig config;

    public LinearSim(
            Mechanism2d mech, TalonFXSimState linearMotorSim, LinearConfig config, String name) {
        this.config = config;

        this.elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.numMotors),
                        config.kElevatorGearing,
                        config.kCarriageMass,
                        config.kElevatorDrumRadius,
                        config.kMinElevatorHeight,
                        config.kMaxElevatorHeight,
                        true,
                        0);

        m_mech2dRoot = mech.getRoot("Elevator Root", 0.1, 0);

        m_elevatorMech2d =
                m_mech2dRoot.append(
                        new MechanismLigament2d(
                                "Elevator",
                                Units.inchesToMeters(20),
                                config.angle,
                                config.lineWidth,
                                new Color8Bit(Color.kOrange)));
    }

    public double getRotationPerSec() {
        return (elevatorSim.getVelocityMetersPerSecond()
                        / (2 * Math.PI * config.kElevatorDrumRadius))
                * config.kElevatorGearing;
    }

    public double getRotations() {
        return (elevatorSim.getPositionMeters() / (2 * Math.PI * config.kElevatorDrumRadius))
                * config.kElevatorGearing;
    }

    public void simulationPeriodic(TalonFXSimState linearMotorSim) {
        elevatorSim.setInput(linearMotorSim.getMotorVoltage());
        elevatorSim.update(TimedRobot.kDefaultPeriod);

        linearMotorSim.setRotorVelocity(getRotationPerSec());
        linearMotorSim.setRawRotorPosition(getRotations());

        m_mech2dRoot.setPosition(0.1, elevatorSim.getPositionMeters());
    }
}
