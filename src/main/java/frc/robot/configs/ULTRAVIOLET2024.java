package frc.robot.configs;

import frc.robot.RobotConfig.DefaultConfig;
import frc.robot.elevator.Elevator.ElevatorConfig;

public class ULTRAVIOLET2024 extends DefaultConfig {
    public static class elevator extends ElevatorConfig {
        public final double maxHeight = 15;
    }
}
