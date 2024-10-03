package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConfig.DEFAULT;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeCommands;
import frc.robot.launcher.Launcher;
import frc.robot.launcher.LauncherCommands;
import frc.robot.leds.LEDs;
import frc.robot.leds.LEDsCommands;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.PilotCommands;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveCommands;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.util.CrashTracker;

public class Robot extends TimedRobot {
    public static RobotConfig robotConfig;
    public static DEFAULT config;

    /** Create a single static instance of all of your subsystems */
    public static RobotTelemetry telemetry;

    public static RobotSim robotSim;

    public static Swerve swerve;
    public static Elevator elevator;
    public static Intake intake;
    public static Launcher launcher;
    public static LEDs leds;
    public static Pilot pilot;
    public static VisionSystem visionSystem;

    private Command m_autonomousCommand;

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is intialized
     */
    public static void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();

        LEDsCommands.setupLEDTriggers();
        RobotCommands.setupRobotTriggers();
    }

    public static void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    @Override
    public void robotInit() {
        try {
            RobotTelemetry.print("--- Robot Init Starting ---");
            robotSim = new RobotSim();

            /** Set up the config */
            robotConfig = new RobotConfig(); // Setup the robot config and choose which robot
            config = robotConfig.config; // This just makes it easier to access the config

            /**
             * Intialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are alos subsystems.
             */
            swerve = new Swerve(config.swerve);
            Timer.delay(0.1);
            intake = new Intake(); // new Intake(config.intakeAttached);
            Timer.delay(0.1);
            elevator = new Elevator(config.elevator);
            Timer.delay(0.1);
            launcher = new Launcher(config.launcher);
            pilot = new Pilot(config.pilot);
            leds = new LEDs(config.leds);
            visionSystem = new VisionSystem(() -> swerve.getRobotPose());

            /** Intialize Telemetry */
            telemetry = new RobotTelemetry();

            /**
             * Set Default Commands this method should exist for each subsystem that has default
             * command these must be done after all the subsystems are intialized
             */
            SwerveCommands.setupDefaultCommand();
            IntakeCommands.setupDefaultCommand();
            LauncherCommands.setupDefaultCommand();
            LEDsCommands.setupDefaultCommand();
            PilotCommands.setupDefaultCommand();

            pilot.setupTeleopButtons();
            RobotTelemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        RobotTelemetry.print("### Disabled Init Starting ### ");

        resetCommandsAndButtons();

        /*if (!AutonConfig.commandInit) {
            Command AutonStartCommand =
                    FollowPathCommand.warmupCommand().andThen(PathfindingCommand.warmupCommand());
            AutonStartCommand.schedule();
            AutonConfig.commandInit = true;
        }*/

        RobotTelemetry.print("### Disabled Init Complete ### ");
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        // RobotCommands.ensureBrakeMode().schedule(); // sets all motors to brake mode if not
        // already
        RobotTelemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        /*try {
            RobotTelemetry.print("@@@ Auton Init Starting @@@ ");
            clearCommandsAndButtons();
            Command autonCommand = Commands.waitSeconds(0.01).andThen(Auton.getAutonomousCommand());

            if (autonCommand != null) {
                autonCommand.schedule();
                Auton.startAutonTimer();
            } else {
                RobotTelemetry.print("No Auton Command Found");
            }

            LEDsCommands.countdown(15, 10).schedule();

            RobotTelemetry.print("@@@ Auton Init Complete @@@ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }*/
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        RobotTelemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            RobotTelemetry.print("!!! Teleop Init Starting !!! ");
            resetCommandsAndButtons();

            // flip pilot's forward based on what alliance robot is
            /*             swerve.setDriverPerspective(
            Rotation2d.fromDegrees(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? 0
                            : 180)); */

            // if(DriverStation.isFMSAttached()) {
            //     ClimberCommands.safeClimb().withTimeout(2).schedule();
            // }
            RobotTelemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        RobotTelemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            RobotTelemetry.print("~~~ Test Init Starting ~~~ ");
            resetCommandsAndButtons();

            RobotTelemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        RobotTelemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    public void simulationInit() {
        RobotTelemetry.print("$$$ Simulation Init Starting $$$ ");

        RobotTelemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    public void simulationPeriodic() {}
}
