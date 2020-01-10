package frc.robot.subsystems;

import frc.lib.loops.ILooper;

public abstract class Subsystem {

    /**
     * Updates all periodic variables and sensors
     */
    public void readPeriodicInputs() {
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public abstract void writePeriodicOutputs();

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public abstract void outputTelemetry();


    /**
     * Called to reset and configure the subsystem
     */
    public abstract void reset();

    /**
     * Required for the subsystem's looper to be registered to the state machine
     * not required for subsystems that do not use looper
     *
     * @param enabledLooper the subsystem's Looper
     */
    public void registerEnabledLoops(ILooper enabledLooper) {

    }

}