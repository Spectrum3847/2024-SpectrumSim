package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static String rioSerial = "empty";
    public static final Double robotInitDelay = 2.0; // Seconds to wait before starting robot code

    public final String ALPHA2024SERIAL = "032B1F69";
    public final String PM2024SERIAL = "03223839";
    public final String ULTRAVIOLET2024SERIAL = "032B1F69"; // "0329AD07";

    public static final String CANIVORE = "*"; // CANbus name is 3847
    public static final String RIO_CANBUS = "rio";

    public static final int ledPWMport = 0;

    private RobotType robotType = null;
    public boolean intakeAttached = true;

    public DEFAULT config;

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
                config = new DEFAULT();
                break;
            case PM:
                config = new DEFAULT();
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

    public static class DEFAULT {
        public SwerveConfig swerve;
        public ElevatorConfig elevator;
        public LauncherConfig launcher;
        public PivotConfig pivot;
        public LEDsConfig leds;
        public PilotConfig pilot;

        public DEFAULT() {
            swerve = new SwerveConfig();
            elevator = new ElevatorConfig();
            launcher = new LauncherConfig();
            pivot = new PivotConfig();
            leds = new LEDsConfig();
            pilot = new PilotConfig();
        }

        // None of the below should exist in the final version of our code, should all be inside
        // their mechanism classes as config classes
        public static class Intake {
            public class Arm {
                public static int deviceID = 20;
                public static double ratio = 50.0; // 50:1 reduction

                // Hard-stop limits
                // public static double minAngle = Math.toRadians(-60);
                // public static double maxAngle = Math.toRadians(90);
                public static double startingAngle = Math.toRadians(90);

                // Setpoints
                public static double stowAngle = Math.toRadians(80);
                public static double deployAngle = Math.toRadians(0);

                // For simulation
                // public static double simMOI = 0.2; // kgMetersSquared
                // public static double simCGLength = 0.3; // m
            }

            public class Roller {
                public static int deviceID = 21;
                // Spublic static double ratio = 5.0; // 5:1 reduction

                // For simulation
                // public static final double simMOI = 0.01; // kgMetersSquared
                // public static final double angularVelocityScalar = 0.03;
            }
        }

        public class Transforms {
            public static Transform3d robotToCamera =
                    new Transform3d(
                            new Translation3d(0, 0, 0.5), // Centered on the robot, 0.5m up
                            new Rotation3d(0, Math.toRadians(-15), 0)); // Pitched 15 deg up
        }
    }
}
