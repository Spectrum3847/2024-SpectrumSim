package frc.spectrumLib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpectrumLEDs extends SubsystemBase {
    protected int priority = 0;
    // LED IO
    byte[] priorityBuffer;
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;
    protected Runnable defaultPattern = () -> {}; // Start with blank pattern
    private boolean update = true;
    private int counter = 0;
    private double time = 0;
    private int length = 0;

    public SpectrumLEDs(int port, int length) {
        leds = new AddressableLED(port);
        this.length = length;
        leds.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        priorityBuffer = new byte[length];
        leds.setData(buffer);
        leds.start();
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
        buffer = new AddressableLEDBuffer(length);
        leds.start();
    }

    public void periodic() {
        // Have our LEDs calculate only every other cycle
        leds.setData(buffer);

        if (counter % 5 == 0) {
            time = counter * 0.02;
            // update = true;
        } else if ((counter - 1) % 5 == 0) {
            // leds.setData(buffer.getLEDBuffer());
        } else {
            // update = false;
        }
        counter++;
        if (counter > 500) {
            counter = 0;
        }
    }

    public void resetPriority() {
        for (int i = 0; i < priorityBuffer.length; i++) {
            priorityBuffer[i] = 0;
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    public boolean getUpdate() {
        return update;
    }

    public double getLEDTime() {
        return time;
    }

    public void setLED(int i, Color c, int priority) {
        buffer.setLED(i, c);
        priorityBuffer[i] = (byte) priority;
        buffer.setLED(i + length / 2, c);
        priorityBuffer[i + length / 2] = (byte) priority;
    }

    public void setLED(int i, int r, int g, int b, int priority) {
        buffer.setLED(i, new Color(r, g, b));
        priorityBuffer[i] = (byte) priority;
        buffer.setLED(i + length / 2, new Color(r, g, b));
        priorityBuffer[i + length / 2] = (byte) priority;
    }

    public void setHSV(int i, int h, int s, int v, int priority) {
        buffer.setHSV(i, h, s, v);
        priorityBuffer[i] = (byte) priority;
        buffer.setHSV(i + length / 2, h, s, v);
        priorityBuffer[i + length / 2] = (byte) priority;
    }
}
