package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConfig.DefaultConfig;

public class RollerSim {
    private FlywheelSim rollerSim;

    private MechanismRoot2d rollerAxle;

    private MechanismLigament2d rollerViz;
    private TalonFXSimState rollerMotorSim;

    public RollerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim, String name) {
        this.rollerMotorSim = rollerMotorSim;
        rollerSim =
                new FlywheelSim(
                        DCMotor.getKrakenX60Foc(1),
                        DefaultConfig.Intake.Roller.ratio,
                        DefaultConfig.Intake.Roller.simMOI);

        rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);

        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(2),
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));

        SmartDashboard.putData(name + " Viz", mech);
    }

    public void simulationPeriodic(double x, double y) {
        // ------ Update sim based on motor output
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(TimedRobot.kDefaultPeriod);

        // ------ Update motor based on sim
        // Make sure to convert radians at the mechanism to rotations at the motor
        // Subtracting out the starting angle is necessary so the simulation can't "cheat" and use
        // the
        // sim as an absolute encoder.
        double rotationsPerSecond = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rollerMotorSim.setRotorVelocity(rotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotationsPerSecond * TimedRobot.kDefaultPeriod);

        // Update the axle as the robot moves
        rollerAxle.setPosition(x, y);

        // Scale down the angular velocity so we can actually see what is happening
        rollerViz.setAngle(
                rollerViz.getAngle()
                        + Math.toDegrees(rollerSim.getAngularVelocityRPM())
                                * TimedRobot.kDefaultPeriod
                                * 0.1);
    }
}
