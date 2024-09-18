package frc.robot.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.RobotSim;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;

public class IntakeSim {

    public ArmConfig armConfig = new ArmConfig(0.7, 0.3, 5, 0.5, -60, 90).setStartingAngle(90);

    private ArmSim armSim;
    private RollerSim rollerSim;

    public IntakeSim(TalonFXSimState rollerMotorSim, TalonFXSimState armMotorSim) {
        rollerSim =
                new RollerSim(
                        new RollerConfig().setDiameter(3),
                        RobotSim.leftView,
                        rollerMotorSim,
                        "Intake Roller");

        armSim = new ArmSim(armConfig, RobotSim.leftView, armMotorSim, "Intake Arm");
    }

    public void simulationPeriodic(TalonFXSimState armMotorSim) {
        armSim.simulationPeriodic();
        rollerSim.simulationPeriodic(
                armConfig.pivotX + armConfig.length * Math.cos(armSim.getAngleRads()),
                armConfig.pivotY + armConfig.length * Math.sin(armSim.getAngleRads()));
    }
    // --- END STUFF FOR SIMULATION ---
}
