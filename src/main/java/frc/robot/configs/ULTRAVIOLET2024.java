package frc.robot.configs;

import frc.robot.RobotConfig.DEFAULT;

public class ULTRAVIOLET2024 extends DEFAULT {

    public ULTRAVIOLET2024() {
        super();
        // Eleavtor
        elevator.attached(true);
        elevator.configSupplyCurrentLimit(15);
    }
}
