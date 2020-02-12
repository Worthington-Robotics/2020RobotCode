package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Designed to process a boolean that must stay true over a set period of time
 * e.g. a object passing a sensor
 * 
 * @version 1.0.2
 * @author Spazzinq (George Fang)
 */
public class TimedBoolean {
    private double startingTime, checkTime;
    private boolean isStarted = false;

    /**
     * @param checkTime wait time until boolean is sufficently true
     */
    public TimedBoolean(double checkTime) {
        this.startingTime = Timer.getFPGATimestamp();
        this.checkTime = checkTime;
    }

    /**
     * 
     * @return if the given boolean has been true for a sufficent amount of time
     */
    public boolean getBoolean() {
        return  isStarted && (Timer.getFPGATimestamp() > startingTime + checkTime);
    }

    /**
     * sets the starting time of the count-off to the current time if not already
     * set
     */
    public void start() {
        if (!isStarted) {

            this.startingTime = Timer.getFPGATimestamp();
            isStarted = true;
        }
    }

    public void stop() {
        isStarted = false;
    }

    /**
     * resets the starting time of the count-off to the current time
     */
    public void reset() {
        this.startingTime = Timer.getFPGATimestamp();
    }
}
