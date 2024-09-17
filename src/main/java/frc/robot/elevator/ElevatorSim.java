package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.mechanism.LinearSim;

public class ElevatorSim extends LinearSim {
    public ElevatorSim(ElevatorConfig config, TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
        super(mech, elevatorMotorSim, config.name);
    }
}
