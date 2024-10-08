package frc.spectrumLib.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.function.IntSupplier;
import lombok.Getter;
import lombok.Setter;

public class LaserCanSubsystem implements Subsystem, NTSendable {
    private LaserCan lasercan;
    private LaserCanConfig config;
    @Setter private double cachedValue = -1000; // -1000 is an error value or no data

    public static class LaserCanConfig {
        @Getter @Setter private String name;;
        @Getter @Setter private int id;
        @Getter @Setter private boolean attached = true;
        @Getter @Setter private boolean shortRange = true;
        @Getter @Setter private int x = 8;
        @Getter @Setter private int y = 8;
        @Getter @Setter private int w = 4;
        @Getter @Setter private int h = 4;

        @Getter @Setter
        private LaserCan.TimingBudget timingBudget = LaserCan.TimingBudget.TIMING_BUDGET_100MS;

        public LaserCanConfig(String name, int id) {
            this.name = name;
            this.id = id;
        }
    }

    // default constructor
    public LaserCanSubsystem(LaserCanConfig config) {
        this.config = config;
        if (config.isAttached()) {
            lasercan = new LaserCan(config.id);
        }
        if (config.isShortRange() == true) {
            setShortRange();
        } else {
            setLongRange();
        }
        setRegionOfInterest(
                config.getX(), config.getY(), config.getW(), config.getH()); // max region
        setTimingBudget(config.getTimingBudget()); // Can only set ms to 20, 33, 50, and 100
        telemetryInit();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public LaserCanSubsystem(String name, int id, boolean attached) {
        this(new LaserCanConfig(name, id).setAttached(attached));
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    @Override
    public void periodic() {
        if (isAttached() && Robot.isReal()) {
            cachedValue = updateDistance();
        }
    }

    @Override
    public String getName() {
        return config.getName();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        builder.addBooleanProperty("Attached", this::isAttached, null);
        if (Robot.isSimulation()) {
            builder.addDoubleProperty("Distance", this::getDistance, this::setCachedValue);
        } else {
            builder.addDoubleProperty("Distance", this::getDistance, null);
        }
    }

    // Setup the telemetry values, has to be called at the end of the implemetned mechanism
    // constructor
    public void telemetryInit() {
        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);
    }

    public Trigger isGreaterThan(IntSupplier distance) {
        return new Trigger(() -> getDistance() > distance.getAsInt())
                .and(validDistance())
                .debounce(0.25);
    }

    public Trigger isLessThan(IntSupplier distance) {
        return new Trigger(() -> getDistance() < distance.getAsInt())
                .and(validDistance())
                .debounce(0.25);
    }

    public Trigger validDistance() {
        return new Trigger(() -> getDistance() >= 0);
    }

    public double getDistance() {
        return cachedValue;
    }

    /* Helper methods for constructors */
    public void setShortRange() {
        config.setShortRange(true);
        if (isAttached() && Robot.isReal()) {
            try {
                lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            } catch (ConfigurationFailedException e) {
                logError();
            }
        }
    }

    public void setLongRange() {
        config.setShortRange(false);
        if (isAttached() && Robot.isReal()) {
            try {
                lasercan.setRangingMode(LaserCan.RangingMode.LONG);
            } catch (ConfigurationFailedException e) {
                logError();
            }
        }
    }

    public void setRegionOfInterest(int x, int y, int w, int h) {
        config.setX(x);
        config.setY(y);
        config.setW(w);
        config.setH(h);
        if (isAttached() && Robot.isReal()) {
            try {
                lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(x, y, w, h));
            } catch (ConfigurationFailedException e) {
                logError();
            }
        }
    }

    public void setTimingBudget(LaserCan.TimingBudget timingBudget) {
        config.setTimingBudget(timingBudget);
        if (isAttached() && Robot.isReal()) {
            try {
                lasercan.setTimingBudget(timingBudget);
            } catch (ConfigurationFailedException e) {
                logError();
            }
        }
    }

    /* Other methods */
    private static void logError() {
        DriverStation.reportWarning("LaserCan: failed to complete operation", false);
    }

    private int updateDistance() {
        if (isAttached() && Robot.isReal()) {
            LaserCan.Measurement measurement = lasercan.getMeasurement();
            if (measurement != null) {
                if (measurement.status == 0) {
                    return measurement.distance_mm;
                } else {
                    if (measurement.status != 2) {
                        DriverStation.reportWarning(
                                "LaserCan #"
                                        + config.getId()
                                        + " status went bad: "
                                        + measurement.status,
                                false);
                    }
                    return measurement.distance_mm;
                }
            }
        }
        return -1000;
    }
}
