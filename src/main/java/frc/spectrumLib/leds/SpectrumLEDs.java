package frc.spectrumLib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SpectrumLEDs implements Subsystem {
    // LED IO
    private AddressableLED leds;
    private SpectrumLEDBuffer buffer;
    private boolean update = true;
    private int counter = 0;
    private double time = 0;
    private int length = 0;

    public SpectrumLEDs(int port, int length) {
        leds = new AddressableLED(port);
        this.length = length;
        leds.setLength(length);
        buffer = new SpectrumLEDBuffer(length);
        leds.setData(buffer.getLEDBuffer());
        leds.start();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Allows to reconfigure the LEDs after boot. This can be helpful if different robot
     * configurations have different LEDs
     */
    public void setLEDsPortLength(int port, int length) {
        leds.stop();
        leds.close();
        leds = new AddressableLED(port);
        leds.setLength(length);
        buffer = new SpectrumLEDBuffer(length);
        leds.start();
    }

    public void periodic() {
        // Have our LEDs calculate only every other cycle
        leds.setData(buffer.getLEDBuffer());

        if (counter % 5 == 0) {
            time = counter * 0.02;
        }
        counter++;
        if (counter > 500) {
            counter = 0;
        }
    }

    public void resetPriority() {
        buffer.resetPriorityBuffer();
    }

    public boolean getUpdate() {
        return update;
    }

    public double getLEDTime() {
        return time;
    }

    public void setLED(int i, Color c, int priority) {
        buffer.setLED(i, c, priority);
        buffer.setLED(i + length / 2, c, priority);
    }

    public void setLED(int i, int r, int g, int b, int priority) {
        buffer.setLED(i, new Color(r, g, b), priority);
        buffer.setLED(i + length / 2, new Color(r, g, b), priority);
    }

    public void setHSV(int i, int h, int s, int v, int priority) {
        buffer.setHSV(i, h, s, v, priority);
        buffer.setHSV(i + length / 2, h, s, v, priority);
    }
}
