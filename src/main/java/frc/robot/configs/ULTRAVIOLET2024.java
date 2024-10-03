package frc.robot.configs;

import frc.robot.RobotConfig.DEFAULT;

public class ULTRAVIOLET2024 extends DEFAULT {

    public ULTRAVIOLET2024() {
        super();
        swerve.configEncoderOffsets(0, 0, 0, 0);
        // Attached Mechanisms
        elevator.setAttached(true);
        launcher.setAttached(true);
    }
}
