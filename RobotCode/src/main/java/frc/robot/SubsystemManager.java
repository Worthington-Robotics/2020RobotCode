package frc.robot;

import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;
import frc.lib.util.Logable;
import frc.lib.util.ReflectingLogger;
import frc.robot.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class SubsystemManager implements ILooper, Logable {

    private final List<Subsystem> mAllSubsystems;
    private final List<Loop> mLoops = new ArrayList<>();
    final List<Logable> logables = new ArrayList<>();
    private ReflectingLogger<LogData> logger;
    private final boolean mIgnoreLogger;
    private Timing timing = new Timing();

    /**
     * a manager class to handle all of the individual subsystems
     *
     * @param allSubsystems -- a list of subsystems in order of importance.
     *                      Dependent subsystems should be reset before their parents
     * @param ignoreLogger  -- ignores the initialization of the logger
     *                      and any subsequent errors it may generate
     */
    public SubsystemManager(final List<Subsystem> allSubsystems, final boolean ignoreLogger) {
        mAllSubsystems = allSubsystems;
        mIgnoreLogger = ignoreLogger;

        //get all subsystems to log from
        logables.addAll(allSubsystems);
    }

    /**
     * Registers any additonal logables to be logged when the robot is enabled.
     * @param toLog the list of logable classes to pull data from
     */
    public void addLoggingSource(final List<Logable> toLog){
        logables.addAll(new ArrayList<Logable>(toLog));
        logables.add(this);
    }

    private void createLogging(){
        try {
            String logableNames = "Logging on: ";
            //create reflecting logger
            List<LogData> logData = new ArrayList<>();
            for (Logable logable: logables) {
                logableNames += logable.getClass().getSimpleName() + ", ";
                logData.add(logable.getLogger());
            }
            System.out.println(logableNames);

            logger = new ReflectingLogger<>(logData);
        } catch (final Exception e) {
            // show logger failed to init
            DriverStation.reportError("Logger unable to start", e.getStackTrace());

            //throw the runtime error only if turned on via argument
            if (!mIgnoreLogger) {
                throw new RuntimeException("Error instantiating the logger");
            }
        }

    }

    /**
     * Runs a pass of the reflection based logger over all substems
     */
    public void logTelemetry(double fpgaTimestamp) {
        //make sure logger is properly initialized
        if (logger != null) {
            // create current list of subsystem IO
            final List<LogData> logData = new ArrayList<>();
            logables.forEach((logable) -> logData.add(logable.getLogger()));

            //update the logger from the current form of the list
            logger.update(logData, fpgaTimestamp);
        }
    }

    /**
     * Runs the output telemetry method on all subsystems to communicate data
     */
    public void outputTelemetry() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    /**
     * Calls the onStop method for all subsystems to put the robot into
     * an open loop control mode
     */
    public void onStop() {
        mAllSubsystems.forEach(Subsystem::onStop);
    }

    /**
     * Resets all subsystems in the order that they were registered
     * with the subsystem manager. If the order is incorrect determinism
     * during the reset may be lost
     */
    public void resetAllSubsystems() {
        mAllSubsystems.forEach(Subsystem::reset);
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(final double timestamp) {
            createLogging();
            for (final Loop l : mLoops) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(final double timestamp) {
            timing.timing.clear();
            for (final Subsystem s : mAllSubsystems) {
                final double start = Timer.getFPGATimestamp();
                s.readPeriodicInputs();
                timing.timing.add(Timer.getFPGATimestamp() - start);
            }
            for (final Loop l : mLoops) {
                final double start = Timer.getFPGATimestamp();
                l.onLoop(timestamp);
                timing.timing.add(Timer.getFPGATimestamp() - start);
            }
            
            for (final Subsystem s : mAllSubsystems) {
                final double start = Timer.getFPGATimestamp();
                s.writePeriodicOutputs();
                timing.timing.add(Timer.getFPGATimestamp() - start);
            }
            //run logging pass
            logTelemetry(timestamp);
        }

        @Override
        public void onStop(final double timestamp) {
            for (final Loop l : mLoops) {
                l.onStop(timestamp);
            }
            if(logger != null)
            logger.close();
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(final double timestamp) {

        }

        @Override
        public void onLoop(final double timestamp) {
            for (final Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (final Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(final double timestamp) {
        }
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    public LogData getLogger(){
        return timing;
    }

    public class Timing extends LogData{
        public List<Double> timing = new ArrayList<>();
    }

}
