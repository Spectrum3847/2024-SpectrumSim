package frc.spectrumLib.lasercan;

import java.util.function.IntSupplier;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;

public class LaserCanSubsystem implements Subsystem{
    private LaserCan lasercan;
    private LaserCanConfig config;
    private double cachedValue = -1000;

    public static class LaserCanConfig{
        @Getter @Setter private String name;;
        @Getter @Setter private int id;
        @Getter @Setter private boolean shortRange = true;
        @Getter @Setter private int x = 8;
        @Getter @Setter private int y = 8;
        @Getter @Setter private int w = 4;
        @Getter @Setter private int h = 4;
        @Getter @Setter private LaserCan.TimingBudget timingBudget = LaserCan.TimingBudget.TIMING_BUDGET_100MS;

        public LaserCanConfig(String name, int id){
            this.name = name;
            this.id = id;
        }
    }

    // default constructor
    public LaserCanSubsystem(LaserCanConfig config) {
        this.config = config;
        lasercan = new LaserCan(config.id);
        if (config.isShortRange() == true) {
            setShortRange();
        } else {
            setLongRange();
        }
        setRegionOfInterest(config.getX(), config.getY(), config.getW(), config.getH()); // max region
        setTimingBudget(config.getTimingBudget()); // Can only set ms to 20, 33, 50, and 100
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        cachedValue = updateDistance();
    }

    @Override
    public String getName() {
        return config.getName();
    }

    public Trigger isGreaterThan(IntSupplier distance) {
        return new Trigger(() -> getDistance() > distance.getAsInt()).and(validDistance()).debounce(0.25);
    }

    public Trigger isLessThan(IntSupplier distance) {
        return new Trigger(() -> getDistance() < distance.getAsInt()).and(validDistance()).debounce(0.25);
    }

    public Trigger validDistance() {
        return new Trigger(() -> getDistance() >= 0);
    }

    /* Helper methods for constructors */

    public void setShortRange() {
        config.setShortRange(true);
        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setLongRange() {
        config.setShortRange(false);
        try {
            lasercan.setRangingMode(LaserCan.RangingMode.LONG);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setRegionOfInterest(int x, int y, int w, int h) {
        config.setX(x);
        config.setY(y);
        config.setW(w);
        config.setH(h);
        try {
            lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(x, y, w, h));
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setTimingBudget(LaserCan.TimingBudget timingBudget) {
        config.setTimingBudget(timingBudget);
        try {
            lasercan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    /* Other methods */
    private static void logError() {
        DriverStation.reportWarning("LaserCan: failed to complete operation", false);
    }

    public double getDistance() {
        return cachedValue;
    }

    private int updateDistance() {
        LaserCan.Measurement measurement = lasercan.getMeasurement();
        if (measurement != null) {
            if (measurement.status == 0) {
                return measurement.distance_mm;
            } else {
                if (measurement.status != 2) {
                    DriverStation.reportWarning(
                            "LaserCan #" + config.getId() + " status went bad: " + measurement.status, false);
                }
                return measurement.distance_mm;
            }
        } else {
            return -1000;
        }
    }
}
