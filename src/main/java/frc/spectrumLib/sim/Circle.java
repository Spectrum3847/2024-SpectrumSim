package frc.spectrumLib.sim;


import com.fasterxml.jackson.core.sym.Name;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Setter;


public class Circle {
    
    private Mechanism2d mech;
    private int backgroundLines;
    private double diameterInches;
    private MechanismRoot2d root;
    @Setter private Color8Bit color = new Color8Bit(Color.kBlack);
    private String name;

    public Circle(Mechanism2d mech, int backgroundLines, double diameterInches, Color8Bit color, String name, MechanismRoot2d root){
        this.mech = mech;
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.color = color;
        this.name = name;
        this.root = root;
    }

    public Circle drawCircle(){
        MechanismLigament2d[] circleBackground = new MechanismLigament2d[this.backgroundLines];
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i] =
                    root.append(new MechanismLigament2d(                           
                                    name + " Background " + i,
                                    Units.inchesToMeters(diameterInches) / 2.0,
                                    (360 / backgroundLines) * i,
                                    diameterInches,
                                    color));       
    }
    return this;
}



}
    
        


