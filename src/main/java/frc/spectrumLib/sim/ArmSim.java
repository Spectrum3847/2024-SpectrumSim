package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotSim;

public class ArmSim {
    private SingleJointedArmSim armSim;
    private ArmConfig config;

    private MechanismRoot2d armPivot;
    private MechanismLigament2d armMech2d;
    private TalonFXSimState armMotorSim;

    public ArmSim(ArmConfig config, Mechanism2d mech, TalonFXSimState armMotorSim, String name) {
        this.config = config;
        this.armMotorSim = armMotorSim;
        armSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(config.numMotors),
                        config.ratio,
                        config.simMOI,
                        config.simCGLength,
                        config.minAngle,
                        config.maxAngle,
                        true, // Simulate gravity
                        config.startingAngle);

        armPivot = RobotSim.leftView.getRoot("Intake Arm Pivot", config.pivotX, config.pivotY);
        armMech2d =
                armPivot.append(
                        new MechanismLigament2d(
                                "Intake Arm", config.length, 0.0, 5.0, new Color8Bit(Color.kBlue)));
    }

    public void simulationPeriodic() {
        armSim.setInput(armMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);

        armMotorSim.setRawRotorPosition(
                (armSim.getAngleRads() - config.startingAngle) * config.ratio * 2.0 * Math.PI);
        armMotorSim.setRotorVelocity(
                armSim.getVelocityRadPerSec() * config.ratio / (2.0 * Math.PI));

        // ------ Update viz based on sim
        armMech2d.setAngle(Math.toDegrees(armSim.getAngleRads()));
    }

    public double getAngleRads() {
        return armSim.getAngleRads();
    }
}
