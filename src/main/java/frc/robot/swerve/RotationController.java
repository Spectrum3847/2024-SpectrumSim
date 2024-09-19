package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class RotationController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;
    PIDController holdController;
    Constraints constraints;

    double calculatedValue = 0;

    public RotationController(SwerveConfig config) {
        this.config = config;
        constraints = new Constraints(config.maxAngularVelocity, config.maxAngularAcceleration);
        controller =
                new ProfiledPIDController(
                        config.kPRotationController,
                        config.kIRotationController,
                        config.kDRotationController,
                        constraints);

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(config.rotationTolerance);

        // Hold controller is standard PID
        holdController =
                new PIDController(
                        config.kPHoldController, config.kIHoldController, config.kDHoldController);

        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(
                config.rotationTolerance / 2); // Half the tolerance of turn controller
    }

    public double calculate(double goalRadians, double currentRadians, boolean isHoldController) {
        double measurement = currentRadians;
        calculatedValue = controller.calculate(measurement, goalRadians);

        if (atSetpoint()) {
            if (isHoldController) {
                return calculatedValue = calculateHold(goalRadians, currentRadians);
            }
            return calculatedValue = 0;
        } else {
            return calculatedValue;
        }
    }

    public double calculate(double goalRadians, double currentRadians) {
        return calculate(goalRadians, currentRadians, false);
    }

    public double calculateHold(double goalRadians, double currentRadians) {
        double calculatedValue = holdController.calculate(currentRadians, goalRadians);
        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void reset(double currentRadians) {
        controller.reset(currentRadians);
        holdController.reset();
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
