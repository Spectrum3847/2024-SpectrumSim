package frc.spectrumLib.talonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.ArrayList;

/** From 254 lib imported from 1678-2024 */
public class TalonFXChecker extends MotorChecker<TalonFX> {
    private static class ControlRequest {
        public StatusSignal<ControlModeValue> mMode;
        public double mSetValue;
    }

    protected ArrayList<ControlRequest> mControlRequests = new ArrayList<>();

    public static boolean checkMotors(
            Mechanism mechanism,
            ArrayList<MotorConfig<TalonFX>> motorsToCheck,
            CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorsImpl(mechanism, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<TalonFX> config : mMotorsToCheck) {
            TalonFX talon = (TalonFX) config.mMotor;

            ControlRequest configuration = new ControlRequest();
            configuration.mMode = talon.getControlMode();

            mControlRequests.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.getAppliedControl();
            mMotorsToCheck.get(i).mMotor.set(mControlRequests.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(TalonFX motor, double output) {
        motor.set(output);
    }

    @Override
    public double getMotorCurrent(TalonFX motor) {
        return motor.getStatorCurrent().getValue();
    }
}
