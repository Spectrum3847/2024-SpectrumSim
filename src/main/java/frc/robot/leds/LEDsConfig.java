package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDsConfig {
    // Length has to be static to make the enums work, it would be nice if there was another way
    public static int length = 29;

    public final int port = 0;
    public final double strobeFastDuration = 0.1;
    public final double strobeSlowDuration = 0.2;
    public final double breathDuration = 1.0;
    public final double rainbowCycleLength = 25.0;
    public final double rainbowDuration = 0.25;
    public final double waveExponent = 0.4;
    public final double waveFastCycleLength = 25.0;
    public final double waveFastDuration = 0.25;
    public final double waveSlowCycleLength = 25.0;
    public final double waveSlowDuration = 3.0;
    public final double waveAllianceCycleLength = 15.0;
    public final double waveAllianceDuration = 2.0;
    public final double autoFadeTime = 2.5;
    public final double autoFadeMaxTime = 5.0;

    public final Color SPECTRUM_COLOR = new Color(130, 103, 185);

    public LEDsConfig() {}

    public enum Section {
        FULL,
        HALF_LOW,
        HALF_HIGH,
        QUARTER_LOW,
        QUARTER_HIGH;

        public int start() {
            switch (this) {
                case FULL:
                    return 0;
                case HALF_LOW:
                    return 0;
                case HALF_HIGH:
                    return length - (length / 2);
                case QUARTER_LOW:
                    return 0;
                case QUARTER_HIGH:
                    return length - (length / 4);
                default:
                    return 0;
            }
        }

        public int end() {
            switch (this) {
                case FULL:
                    return length;
                case HALF_LOW:
                    return length / 2;
                case HALF_HIGH:
                    return length;
                case QUARTER_LOW:
                    return length / 4;
                case QUARTER_HIGH:
                    return length;
                default:
                    return 0;
            }
        }
    }
}
