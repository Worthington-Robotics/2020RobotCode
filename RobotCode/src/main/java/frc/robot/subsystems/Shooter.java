package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.subsystems.Subsystem;

public class Shooter extends Subsystem {

    private static Shooter m_Shooter = new Shooter();
    public static Shooter getInstance(){return m_Shooter;}
    private TalonSRX Talon1, Talon2;
    private Shooter(){
        Talon1 = new TalonSRX(1);
        Talon2 = new TalonSRX(2);
    }


    /**
     * Updates all periodic variables and sensors
     */
    @Override
    public void readPeriodicInputs() {

    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            /**
             * what the loop runs when started by the subsystem manager
             *
             * @param timestamp handled by subsystem manager
             */
            @Override
            public void onStart(double timestamp) {

            }

            /**
             * what the loop runs while run by the subsystem manager
             *
             * @param timestamp handled by subsystem manager
             */
            @Override
            public void onLoop(double timestamp) {

            }

            /**
             * what the loop runs when ended by the subsystem manager
             *
             * @param timestamp handled by subsystem manager
             */
            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    @Override
    public void writePeriodicOutputs() {

    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    @Override
    public void outputTelemetry() {

    }

    /**
     * Called to reset and configure the subsystem
     */
    @Override
    public void reset() {

    }

    public class PeriodicIO {

    }
}
