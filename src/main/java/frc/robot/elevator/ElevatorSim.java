package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;

public class ElevatorSim extends LinearSim {

    public ElevatorSim(ElevatorConfig config, TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
        super(
                new LinearConfig(
                                0.5, 0.0, config.kElevatorGearing, config.kElevatorDrumRadiusMeters)
                        .setAngle(180 - 72),
                mech,
                elevatorMotorSim,
                config.name);
    }
}
