package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConfig.DEFAULT;

public class RollerSim {

    public static class RollerConfig {
        public double rollerDiameterInches = 2;
        public int backgroundLines = 36;
        public Color8Bit offColor = new Color8Bit(Color.kBlack);
        public Color8Bit fwdColor = new Color8Bit(Color.kGreen);
        public Color8Bit revColor = new Color8Bit(Color.kRed);

        public RollerConfig() {}

        public RollerConfig setDiameter(double diameter) {
            rollerDiameterInches = diameter;
            return this;
        }
    }

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d[] rollerBackground;
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;

    public RollerSim(
            Mechanism2d mech, TalonFXSimState rollerMotorSim, RollerConfig config, String name) {
        this.config = config;
        this.rollerMotorSim = rollerMotorSim;
        rollerSim =
                new FlywheelSim(
                        DCMotor.getKrakenX60Foc(1),
                        DEFAULT.Intake.Roller.ratio,
                        DEFAULT.Intake.Roller.simMOI);

        rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);

        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(config.rollerDiameterInches) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));

        rollerBackground = new MechanismLigament2d[config.backgroundLines];
        for (int i = 0; i < config.backgroundLines; i++) {
            rollerBackground[i] =
                    rollerAxle.append(
                            new MechanismLigament2d(
                                    name + " Background " + i,
                                    Units.inchesToMeters(config.rollerDiameterInches) / 2.0,
                                    (360 / config.backgroundLines) * i,
                                    config.rollerDiameterInches,
                                    new Color8Bit(Color.kBlack)));
        }
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
        double rpm = rollerSim.getAngularVelocityRPM() / 2;
        rollerViz.setAngle(
                rollerViz.getAngle() + Math.toDegrees(rpm) * TimedRobot.kDefaultPeriod * 0.1);

        if (rollerSim.getAngularVelocityRadPerSec() < -1) {
            setHalfBackground(config.revColor);
        } else if (rollerSim.getAngularVelocityRadPerSec() > 1) {
            setBackgroundColor(config.fwdColor);
        } else {
            setBackgroundColor(config.offColor);
        }
    }

    public void setBackgroundColor(Color8Bit color8Bit) {
        for (int i = 0; i < config.backgroundLines; i++) {
            rollerBackground[i].setColor(color8Bit);
        }
    }

    public void setHalfBackground(Color8Bit color8Bit) {
        for (int i = 0; i < config.backgroundLines; i++) {
            if (i % 2 == 0) {
                rollerBackground[i].setColor(color8Bit);
            } else {
                rollerBackground[i].setColor(config.offColor);
            }
        }
    }
}
