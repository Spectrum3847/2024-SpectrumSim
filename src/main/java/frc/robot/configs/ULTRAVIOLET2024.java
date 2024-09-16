package frc.robot.configs;

import frc.robot.RobotConfig.DefaultConfig;

public class ULTRAVIOLET2024 extends DefaultConfig {

    public ULTRAVIOLET2024() {
        super();
        elevator = new Elevator();
    }

    public class Elevator extends DefaultConfig.Elevator {
        public final double maxHeight = 15;
    }
}
