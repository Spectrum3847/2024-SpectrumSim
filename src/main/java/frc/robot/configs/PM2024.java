package frc.robot.configs;

public class PM2024 extends ULTRAVIOLET2024 {

    // We should be able to configure default LED colors per robot, so we know the right config is
    // loading

    public PM2024() {
        super();
        swerve.configEncoderOffsets(0.3, 0.3, 0.3, 0.3);
        launcher.setAttached(false);
    }
}
