package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.wpilibj.Timer;
import frc.spectrumLib.util.Conversions;
import frc.spectrumLib.util.Util;
import java.util.ArrayList;
import lombok.Getter;
import lombok.Setter;

public class MotorChecker {
    public static class CheckerConfig {
        @Getter @Setter private double currentFloor = 5;
        @Getter @Setter private double rpmFloor = 2000;

        @Getter @Setter private double currentEpsilon = 5.0;
        @Getter @Setter private double rpmEpsilon = 500;

        @Getter @Setter private double runTimeSec = 1.0;
        @Getter @Setter private double waitTimeSec = 2.0;
        @Getter @Setter private double runOutput = 0.5;
        @Getter @Setter private double stopOutput = 0.0;

        @Getter @Setter private ControlModeValue controlMode = ControlModeValue.VoltageOut;
    }

    private static class StoredControlRequest {
        @Getter @Setter private ControlRequest storedSetValue;
    }

    protected TalonFX[] mMotorsToCheck;

    protected ArrayList<StoredControlRequest> controlRequests = new ArrayList<>();

    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (TalonFX talon : mMotorsToCheck) {

            StoredControlRequest configuration = new StoredControlRequest();
            configuration.storedSetValue = talon.getAppliedControl();

            controlRequests.add(configuration);
        }
    }

    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.length; ++i) {
            mMotorsToCheck[i].setControl(controlRequests.get(i).getStoredSetValue());
        }
    }

    protected void setMotorOutput(TalonFX motor, double output) {
        motor.set(output);
    }

    public double getMotorCurrent(TalonFX motor) {
        return motor.getStatorCurrent().getValue();
    }

    public TalonFX[] getMotorsToCheck(Mechanism mechanism) {
        TalonFX[] motors = new TalonFX[mechanism.config.getFollowerConfigs().length + 1];
        motors[0] = mechanism.getMotor();
        for (int i = 0; i < mechanism.config.getFollowerConfigs().length; i++) {
            motors[i + 1] = mechanism.getFollowerMotors()[i];
        }
        return motors;
    }

    public boolean checkMotorsImpl(Mechanism mechanism) {
        boolean failure = false;
        mMotorsToCheck = getMotorsToCheck(mechanism);
        CheckerConfig checkerConfig = mechanism.config.getCheckerConfig();
        System.out.println("////////////////////////////////////////////////");
        System.out.println(
                "Checking subsystem "
                        + mechanism.getName()
                        + " for "
                        + mMotorsToCheck.length
                        + " motor(s).");

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        storeConfiguration();

        for (TalonFX motor : mMotorsToCheck) {
            setMotorOutput(motor, 0.0);
        }

        for (TalonFX motor : mMotorsToCheck) {
            System.out.println("Checking: " + motor.getDeviceID());

            setMotorOutput(motor, checkerConfig.runOutput);
            Timer.delay(checkerConfig.runTimeSec);

            // poll the interesting information
            double current = getMotorCurrent(motor);
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            rpm =
                    Conversions.RPStoRPM(
                            motor.getVelocity().getValueAsDouble()); // Convert the RPS to RPM
            rpms.add(rpm);
            System.out.print(" RPM: " + rpm);
            System.out.print('\n');

            setMotorOutput(motor, 0.0);

            // perform checks
            if (current < checkerConfig.currentFloor) {
                System.out.println(
                        motor.getDeviceID()
                                + " has failed current floor check vs "
                                + checkerConfig.currentFloor
                                + "!!");
                failure = true;
            }
            if (rpm < checkerConfig.rpmFloor) {
                System.out.println(
                        motor.getDeviceID()
                                + " has failed rpm floor check vs "
                                + checkerConfig.rpmFloor
                                + "!!");
                failure = true;
            }

            Timer.delay(checkerConfig.waitTimeSec);
        }

        // run aggregate checks

        if (currents.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.currentEpsilon)) {
                System.out.println("Currents varied!!!!!!!!!!!");
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.rpmEpsilon)) {
                System.out.println("RPMs varied!!!!!!!!");
                failure = true;
            }
        }

        // restore talon configurations
        restoreConfiguration();

        return !failure;
    }
}
