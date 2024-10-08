package frc.robot.configs;

import frc.robot.RobotConfig.ConfigHolder;

public class ULTRAVIOLET2024 extends ConfigHolder {

    public ULTRAVIOLET2024() {
        super();
        swerve.configEncoderOffsets(0, 0, 0, 0);
        // Attached Mechanisms
        climber.setAttached(true);
        elevator.setAttached(true);
        launcher.setAttached(true);
        pilot.setAttached(true);
    }
}
