package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class StateMachine {

    private static final AtomicInteger state = new AtomicInteger(-1);
    private static final AtomicBoolean wantStop = new AtomicBoolean(true);
    private static final AtomicBoolean stateLock = new AtomicBoolean(false);
    private volatile static StateMachineDescriptor descriptor;
    private volatile static ActionGroup currentState;
    private volatile static double t_start;
    private static final double delay = 0.020;

    private static final Runnable Man = () -> {
        try {
            //run the onStart code from the descriptor
            descriptor.onStart();

            //state goes to 0 to start
            ConcurrentLinkedQueue<ActionGroup> queuedStates = descriptor.getStates();
            state.set(0);
            SmartDashboard.putNumber("StateMachine/state", state.get());
            if (queuedStates == null) {
                //a null state queue has been passed
                //need to set conditions and exit major loop
                state.set(-2);
                SmartDashboard.putNumber("StateMachine/state", state.get());
            } else {
                while (!queuedStates.isEmpty() && !wantStop.get()) {
                    //pull the next element from the queue and run the state
                    SmartDashboard.putNumber("StateMachine/state", state.get());
                    currentState = queuedStates.poll();
                    currentState.onStart();

                    //wait for the state to complete exectuing
                    while (!currentState.isFinished() && !wantStop.get()) {
                        t_start = Timer.getFPGATimestamp();
                        currentState.onLoop();
                        Timer.delay(delay - (Timer.getFPGATimestamp() - t_start));
                    }

                    //when complete stop the state and increment the state counter
                    currentState.onStop();
                    state.getAndAdd(1);
                }
            }
            state.set(-1);
        }catch (Exception e){
            System.out.println(e.getMessage());
            state.set(-3);
        } finally{
            //run all cleanup procedures
            SmartDashboard.putNumber("StateMachine/state", state.get());
            stateLock.set(false);
            wantStop.set(true);

            //run the onStop code from the descriptor
            descriptor.onStop();
        }
    };

    /**
     * starts the state machine (if not already runnning a descriptor)
     * @param descrip the descriptor to run in the machine
     * @return true if the machine was started successfully
     */
    public static boolean runMachine(StateMachineDescriptor descrip) {
        //if the machine is currenly running it must be shut down independently in order to start a new descriptor
        if(stateLock.get()) return false;

        //entered a resettable state
        //reset the state booleans
        stateLock.set(true);
        wantStop.set(false);

        //move the new state machine in
        descriptor = descrip;
        
        //start the new state machine thread
        Thread thread = new Thread(Man);
        thread.start();

        return true;
    }

    /**
     * gets the current status of the state machine
     * @return true if the machine is currently running
     */
    public static boolean isRunning(){
        return stateLock.get();
    }

    /**
     * forces the state machine to stop and exit within the next iteration.
     */
    public static void assertStop(){
        if(!wantStop.get()){
            wantStop.set(true);
            System.out.println("State Machine Halting");
        }
    }


}
