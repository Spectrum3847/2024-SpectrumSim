package frc.robot.launcher;

import frc.spectrumLib.sim.RollerSim.RollerConfig;

public class LauncherSim {
    public static class LauncherSimConfig extends RollerConfig {
        public LauncherSimConfig() {
            super();
            setDiameter(6);
        }
    }
}
