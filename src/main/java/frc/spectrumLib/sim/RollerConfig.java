package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RollerConfig {
    public double rollerDiameterInches = 2;
    public int backgroundLines = 36;
    public double gearRatio = 5;
    public double simMOI = 0.01;
    public Color8Bit offColor = new Color8Bit(Color.kBlack);
    public Color8Bit fwdColor = new Color8Bit(Color.kGreen);
    public Color8Bit revColor = new Color8Bit(Color.kRed);

    public RollerConfig() {}

    public RollerConfig setDiameter(double diameter) {
        rollerDiameterInches = diameter;
        return this;
    }

    public RollerConfig setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    public RollerConfig setSimMOI(double moi) {
        simMOI = moi;
        return this;
    }
}
