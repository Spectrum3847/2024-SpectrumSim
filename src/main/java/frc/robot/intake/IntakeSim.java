package frc.robot.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConfig.DefaultConfig;

public class IntakeSim {

    // --- BEGIN STUFF FOR SIMULATION ---
    private static final SingleJointedArmSim armSim =
            new SingleJointedArmSim(
                    DCMotor.getKrakenX60Foc(1),
                    DefaultConfig.Intake.Arm.ratio,
                    DefaultConfig.Intake.Arm.simMOI,
                    DefaultConfig.Intake.Arm.simCGLength,
                    DefaultConfig.Intake.Arm.minAngle,
                    DefaultConfig.Intake.Arm.maxAngle,
                    true, // Simulate gravity
                    DefaultConfig.Intake.Arm.startingAngle);

    private static final FlywheelSim rollerSim =
            new FlywheelSim(
                    DCMotor.getKrakenX60Foc(1),
                    DefaultConfig.Intake.Roller.ratio,
                    DefaultConfig.Intake.Roller.simMOI);

    // Mechanism2d Visualization
    // See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    private static final Mechanism2d mech = new Mechanism2d(1, 1);
    private static final double kArmPivotX = 0.7;
    private static final double kArmPivotY = 0.3;
    private static final MechanismRoot2d armPivot =
            mech.getRoot("Intake Arm Pivot", kArmPivotX, kArmPivotY);
    private static final double kIntakeLength = 0.5;
    private static final MechanismLigament2d armViz =
            armPivot.append(
                    new MechanismLigament2d(
                            "Intake Arm", kIntakeLength, 0.0, 5.0, new Color8Bit(Color.kBlue)));

    private static final MechanismRoot2d rollerAxle = mech.getRoot("Intake Roller Axle", 0.0, 0.0);
    private static final MechanismLigament2d rollerViz =
            rollerAxle.append(
                    new MechanismLigament2d(
                            "Intake Roller", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));

    public IntakeSim() {
        SmartDashboard.putData("Arm Viz", mech);
    }

    public void simulationPeriodic(TalonFXSimState armMotorSim, TalonFXSimState rollerMotorSim) {
        // ------ Update sim based on motor output
        armSim.setInput(armMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);

        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(TimedRobot.kDefaultPeriod);

        // ------ Update motor based on sim
        // Make sure to convert radians at the mechanism to rotations at the motor
        // Subtracting out the starting angle is necessary so the simulation can't "cheat" and use
        // the
        // sim as an absolute encoder.
        armMotorSim.setRawRotorPosition(
                (armSim.getAngleRads() - DefaultConfig.Intake.Arm.startingAngle)
                        * DefaultConfig.Intake.Arm.ratio
                        * 2.0
                        * Math.PI);
        armMotorSim.setRotorVelocity(
                armSim.getVelocityRadPerSec() * DefaultConfig.Intake.Arm.ratio / (2.0 * Math.PI));

        double rotationsPerSecond = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rollerMotorSim.setRotorVelocity(rotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotationsPerSecond * TimedRobot.kDefaultPeriod);

        // ------ Update viz based on sim
        armViz.setAngle(Math.toDegrees(armSim.getAngleRads()));

        // Update the axle as the arm moves
        rollerAxle.setPosition(
                kArmPivotX + kIntakeLength * Math.cos(armSim.getAngleRads()),
                kArmPivotY + kIntakeLength * Math.sin(armSim.getAngleRads()));
        // Scale down the angular velocity so we can actually see what is happening
        rollerViz.setAngle(
                rollerViz.getAngle()
                        + Math.toDegrees(rollerSim.getAngularVelocityRPM())
                                * TimedRobot.kDefaultPeriod
                                * DefaultConfig.Intake.Roller.angularVelocityScalar);
    }
    // --- END STUFF FOR SIMULATION ---
}
