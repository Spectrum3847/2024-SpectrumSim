package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

public class Circle {

    @Getter private MechanismLigament2d[] circleBackground;
    private Mechanism2d mech;
    @Getter private int backgroundLines;
    private double diameterInches;
    private MechanismRoot2d root;
    @Setter private Color8Bit color = new Color8Bit(Color.kBlack);
    @Setter private String name;

    public Circle(
            Mechanism2d mech,
            int backgroundLines,
            double diameterInches,
            Color8Bit color,
            String name,
            MechanismRoot2d root) {
        this.mech = mech;
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.color = color;
        this.name = name;
        this.root = root;
    }

    public Circle(
            Mechanism2d mech,
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root) {
        this.mech = mech;
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.name = name;
        this.root = root;
    }

    public void drawCircle() {
        circleBackground = new MechanismLigament2d[this.backgroundLines];
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i] =
                    root.append(
                            new MechanismLigament2d(
                                    name + " Background " + i,
                                    Units.inchesToMeters(diameterInches) / 2.0,
                                    (360 / backgroundLines) * i,
                                    diameterInches,
                                    color));
        }
    }

    public void setBackgroundColor(Color8Bit color) {
        for (int i = 0; i < getBackgroundLines(); i++) {
            circleBackground[i].setColor(color);
        }
    }

    public void setHalfBackground(Color8Bit color8Bit, Color8Bit color8Bit2) {
        for (int i = 0; i < backgroundLines; i++) {
            if (i % 2 == 0) {
                circleBackground[i].setColor(color8Bit);
            } else {
                circleBackground[i].setColor(color8Bit2);
            }
        }
    }
}
