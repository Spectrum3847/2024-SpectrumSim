package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Xbrake implements SwerveRequest {
    public static Command run() {
        return Robot.getSwerve().applyRequest(() -> new Xbrake()).withName("Xbrake");
    }

    /** True to use open-loop control while stopped. */
    public boolean IsOpenLoop = false;

    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        SwerveModuleState[] states = new SwerveModuleState[modulesToApply.length];
        for (int i = 0; i < modulesToApply.length; ++i) {
            states[i] = new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
            if (IsOpenLoop) {
                modulesToApply[i].apply(states[i], DriveRequestType.OpenLoopVoltage);
            } else {
                modulesToApply[i].apply(states[i], DriveRequestType.Velocity);
            }
        }
        Robot.getSwerve().writeSetpoints(states);
        return StatusCode.OK;
    }

    public Xbrake withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }
}
