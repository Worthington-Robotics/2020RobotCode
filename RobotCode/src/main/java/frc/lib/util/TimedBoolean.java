package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class TimedBoolean {
    private double startingTime;

    public TimedBoolean() {
        this.startingTime = Timer.getFPGATimestamp();
    }

    public boolean getBoolean() {
        return Timer.getFPGATimestamp() > startingTime + Constants.TIME_BEFORE_STATIONARY;
    }
}
