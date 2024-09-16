package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

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
    }

    public LinearSim(Mechanism2d mech, TalonFXSimState elevatorMotorSim, String name) {
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
                        0);

        m_mech2dRoot = mech.getRoot("Elevator Root", 10, 0);

        m_elevatorMech2d =
                m_mech2dRoot.append(
                        new MechanismLigament2d(
                                "Elevator",
                                Units.metersToInches(elevatorSim.getPositionMeters()),
                                90));
    }
}
