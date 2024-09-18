package frc.robot.configs;

import frc.robot.RobotConfig.DEFAULT;

public class ULTRAVIOLET2024 extends DEFAULT {

    public ULTRAVIOLET2024() {
        super();
        // swerve.withEncoderOffsets(0, 0, 0, 0);
        // Eleavtor
        elevator.attached(true);
        elevator.configSupplyCurrentLimit(20);
        launcher.attached(true);
    }
}
