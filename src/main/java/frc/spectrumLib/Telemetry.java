package frc.spectrumLib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {

    private static boolean disablePrints = false;

    public Telemetry() {
        super();
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /** Disable Print Statement */
    public static void disablePrints() {
        disablePrints = true;
    }

    /** Enable Print Statements */
    public static void enablePrints() {
        disablePrints = false;
    }

    /** Print a statment if they are enabled */
    public static void print(String output) {
        if (!disablePrints) {
            System.out.println(
                    "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output);
        }
    }
}
