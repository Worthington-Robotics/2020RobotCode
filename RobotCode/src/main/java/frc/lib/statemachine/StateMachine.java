package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Logable;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class StateMachine implements Logable{

    private static final StateMachine instance =  new StateMachine();

    /**
     * @return singleton instance of the state machine
     */
    public static StateMachine getInstance(){
        return instance;
    }

    private final LoggingData data = new LoggingData();
    private final double delay = 0.020;
    private volatile StateMachineDescriptor descriptor;

    /**
     * prevent the creation of a new state machine object
     */
    private StateMachine(){

    }

    /**
     * The runnable thread object that handles the execution of a state machine descriptor.
     * The machine handles the starting code, running each state for a specified amount of time
     * (unless it ends early) and then advancing to the next state untill complete. 
     */
    private final Runnable Man = () -> {
        try {
            System.out.println("State machine starting execution");

            //run the onStart code from the descriptor
            descriptor.onStart();

            //state goes to 0 to start
            ConcurrentLinkedQueue<ActionGroup> queuedStates = descriptor.getStates();
            data.state.set(0);
            SmartDashboard.putNumber("StateMachine/state", data.state.get());
            if (queuedStates == null) {
                //a null state queue has been passed
                //need to set conditions and exit major loop
                data.state.set(-2);
                SmartDashboard.putNumber("StateMachine/state", data.state.get());
            } else {
                while (!queuedStates.isEmpty() && !data.wantStop.get()) {
                    //pull the next element from the queue and run the state
                    SmartDashboard.putNumber("StateMachine/state", data.state.get());
                    data.currentState = queuedStates.poll();
                    data.currentState.onStart();

                    //wait for the state to complete exectuing
                    while (!data.currentState.isFinished() && !data.wantStop.get()) {
                        data.t_start = Timer.getFPGATimestamp();
                        data.currentState.onLoop();
                        Timer.delay(delay - (Timer.getFPGATimestamp() - data.t_start));
                    }

                    //when complete stop the state and increment the state counter
                    data.currentState.onStop();
                    data.state.getAndAdd(1);
                }
            }
            data.state.set(-1);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            data.state.set(-3);
        } finally {
            //run all cleanup procedures
            SmartDashboard.putNumber("StateMachine/state", data.state.get());
            data.stateLock.set(false);
            data.wantStop.set(true);

            //run the onStop code from the descriptor
            descriptor.onStop();
            System.out.println("State machine execution complete");
        }
    };

    /**
     * Starts the state machine (if not already runnning a descriptor)
     *
     * @param descrip the descriptor to run in the machine
     * @return true if the machine was started successfully
     */
    public boolean runMachine(StateMachineDescriptor descrip) {
        //if the machine is currenly running it must be shut down independently in order to start a new descriptor
        if (data.stateLock.get()) {
            return false;
        }

        //entered a resettable state
        //reset the state booleans
        data.stateLock.set(true);
        data.wantStop.set(false);

        //move the new state machine in
        descriptor = descrip;

        //start the new state machine thread
        Thread thread = new Thread(Man);
        thread.start();

        return true;
    }

    /**
     * Gets the current status of the state machine
     *
     * @return true if the machine is currently running
     */
    public boolean isRunning() {
        return data.stateLock.get();
    }

    /**
     * Forces the state machine to stop and exit within the next iteration.
     */
    public void assertStop() {
        if (!data.wantStop.get()) {
            data.wantStop.set(true);
            System.out.println("State Machine Halting");
        }
    }

    /**
     * Allows logging access to internal data structure of the state machine.
     * @return the state machines internal data class
     */
    public LoggingData getLogger(){
        return data;
    }

    /**
     * Internal data class for logging the behaviour of the state machine during runtime
     * for use by the reflecting logger.
     */
    public class LoggingData extends Logable.LogData{
        public final AtomicInteger state = new AtomicInteger(-1);
        public final AtomicBoolean wantStop = new AtomicBoolean(true);
        public final AtomicBoolean stateLock = new AtomicBoolean(false);
        public volatile ActionGroup currentState;
        public volatile double t_start;
    }


}
