package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// General Sim principles
// Always move the root/orgin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    public static final double height = 60;
    public static final double width = 30;

    public static final Translation2d origin =
            new Translation2d(Units.inchesToMeters(width / 2), 0.0);

    public static final Mechanism2d leftView =
            new Mechanism2d(Units.inchesToMeters(width) * 2, Units.inchesToMeters(height));

    public RobotSim() {
        SmartDashboard.putData("Viz", RobotSim.leftView);
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
    }
}
