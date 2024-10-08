package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.configs.ULTRAVIOLET2024;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.launcher.Launcher.LauncherConfig;
import frc.robot.leds.LEDsConfig;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.pivot.Pivot.PivotConfig;
import frc.robot.swerve.SwerveConfig;

public class RobotConfig {

    public static class ConfigHolder {
        public SwerveConfig swerve;
        public ElevatorConfig elevator;
        public LauncherConfig launcher;
        public LEDsConfig leds;
        public PilotConfig pilot;
        public PivotConfig pivot;

        public ConfigHolder() {
            swerve = new SwerveConfig();
            elevator = new ElevatorConfig();
            launcher = new LauncherConfig();
            leds = new LEDsConfig();
            pilot = new PilotConfig();
            pivot = new PivotConfig();
        }
    }

    public String rioSerial = "empty";
    public static final Double robotInitDelay = 2.0; // Seconds to wait before starting robot code

    public final String ALPHA2024SERIAL = "032B1F69";
    public final String PM2024SERIAL = "03223839";
    public final String ULTRAVIOLET2024SERIAL = "032B1F69"; // "0329AD07";

    public static final String CANIVORE = "*"; // CANbus name is 3847
    public static final String RIO_CANBUS = "rio";

    public final int ledPWMport = 0;

    private RobotType robotType = null;

    public ConfigHolder config;

    public RobotConfig() {

        if (Robot.isReal()) {
            Timer.delay(RobotConfig.robotInitDelay); // Wait for the robot to fully boot up
        }
        // Set the RoboRio Serial number for this robot, useful for adjusting comp/practice bot
        // settings
        if (RobotController.getSerialNumber() != null) {
            rioSerial = RobotController.getSerialNumber();
            System.out.println("RIO SERIAL: " + rioSerial);
        }

        checkRobotType();
        // Set Config based on which robot we are on
        switch (getRobotType()) {
            case ALPHA:
                config = new ConfigHolder();
                break;
            case PM:
                config = new ConfigHolder();
                break;
            case SIM:
            case ULTRAVIOLET:
            default:
                /* Set all the default configs */
                config = new ULTRAVIOLET2024();
                break;
        }

        RobotTelemetry.print("ROBOT: " + getRobotType());
    }

    /** Set the RobotType based on if simulation or the serial number of the RIO */
    public RobotType checkRobotType() {
        if (Robot.isSimulation()) {
            robotType = RobotType.SIM;
            RobotTelemetry.print("Robot Type: Simulation");
        } else if (rioSerial.equals(ULTRAVIOLET2024SERIAL)) {
            robotType = RobotType.ULTRAVIOLET;
            RobotTelemetry.print("Robot Type: ULTRAVIOLET 2024");
        } else if (rioSerial.equals(ALPHA2024SERIAL)) {
            robotType = RobotType.ALPHA;
            RobotTelemetry.print("Robot Type: ALPHA 2024");
        } else if (rioSerial.equals(PM2024SERIAL)) {
            robotType = RobotType.PM;
            RobotTelemetry.print("Robot Type: PM 2024");
        } else {
            robotType = RobotType.ULTRAVIOLET;
            DriverStation.reportError(
                    "Could not match rio to robot config; defaulting to ULTRAVIOLET robot config",
                    false);
            RobotTelemetry.print("Robot Type: ULTRAVIOLET 2024");
        }
        return robotType;
    }

    public RobotType getRobotType() {
        return robotType;
    }

    // Add aditional robot types here, need to add them to the checkRobotType method and
    // the config switch statement
    public enum RobotType {
        ALPHA,
        PM,
        ULTRAVIOLET,
        SIM
    }
}
