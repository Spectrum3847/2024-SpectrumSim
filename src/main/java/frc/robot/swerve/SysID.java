package frc.robot.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysID {
    private Swerve swerve;

    public SysID(Swerve swerve) {
        this.swerve = swerve;
    }

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) ->
                                    swerve.setControl(TranslationCharacterization.withVolts(volts)),
                            null,
                            swerve));

    private final SysIdRoutine SysIdRoutineRotation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) -> swerve.setControl(RotationCharacterization.withVolts(volts)),
                            null,
                            swerve));

    private final SysIdRoutine SysIdRoutineSteer =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(7),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) -> swerve.setControl(SteerCharacterization.withVolts(volts)),
                            null,
                            swerve));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}
