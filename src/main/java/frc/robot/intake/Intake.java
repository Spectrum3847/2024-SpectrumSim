package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.DEFAULT;

public class Intake extends SubsystemBase {
    private static final TalonFX armMotor = new TalonFX(DEFAULT.Intake.Arm.deviceID);
    private static final TalonFXSimState armMotorSim = armMotor.getSimState();
    private static final MotionMagicVoltage armMotionMagicControl = new MotionMagicVoltage(0);

    private static final TalonFX rollerMotor = new TalonFX(DEFAULT.Intake.Roller.deviceID);
    private static final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();
    private static IntakeSim intakeSim;

    public Intake() {
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        // CTRE uses rotations, we are using radians
        armConfig.Feedback.SensorToMechanismRatio = DEFAULT.Intake.Arm.ratio * 2.0 * Math.PI;
        armConfig.Slot0.kV = 1.1;
        armConfig.Slot0.kA = 0.03;
        armConfig.Slot0.kP = 10.0;
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0;
        armConfig.MotionMagic.MotionMagicAcceleration = 20.0;
        armMotor.getConfigurator().apply(armConfig);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(rollerConfig);

        // This assumes the arm is always stowed when the code boots.
        armMotor.setPosition(DEFAULT.Intake.Arm.startingAngle);
        retract();
        intakeSim = new IntakeSim(rollerMotorSim);
    }

    public void deploy() {
        setPosition(DEFAULT.Intake.Arm.deployAngle);
        rollerMotor.set(1.0);
    }

    public void retract() {
        setPosition(DEFAULT.Intake.Arm.stowAngle);
        rollerMotor.set(0.0);
    }

    private void setPosition(double position) {
        armMotionMagicControl.Slot = 0;
        armMotionMagicControl.Position = position;
        armMotor.setControl(armMotionMagicControl);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "Arm Target Position", armMotor.getClosedLoopReference().getValue());
        SmartDashboard.putNumber("Arm Actual Position", armMotor.getPosition().getValue());
    }

    public void simulationPeriodic() {
        intakeSim.simulationPeriodic(armMotorSim, rollerMotorSim);
    }
}
