package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class Shooter extends Subsystem {

    private static Shooter m_Shooter = new Shooter();
    private MotorControlMode flywheelMode = MotorControlMode.DISABLED;
    private MotorControlMode turretMode = MotorControlMode.DISABLED;
    private ShooterIO periodic;
    private TalonFX rightFlywheelFalcon, leftFlywheelFalcon;
    private TalonSRX turretControl;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry camtran = table.getEntry("camtran");

    private Shooter() {
        rightFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_LEFT);
        leftFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_RIGHT);
        turretControl = new TalonSRX(Constants.TURRET_CONTROL);
        reset();
    }

    public static Shooter getInstance() {
        return m_Shooter;
    }

    /**
     * Updates all periodic variables and sensors
     */
    @Override
    public void readPeriodicInputs() {
        periodic.targetArea = ta.getDouble(0.0);
        periodic.targetX = tx.getDouble(0.0);
        periodic.targetY = ty.getDouble(0.0);
        periodic.RPMClosedLoopError = rightFlywheelFalcon.getClosedLoopError();
        periodic.rotationsClosedLoopError = turretControl.getClosedLoopError();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                
            }

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
        switch (flywheelMode) {
            case OPEN_LOOP:
                leftFlywheelFalcon.set(ControlMode.PercentOutput, periodic.flywheelDemand);
                rightFlywheelFalcon.set(ControlMode.Follower, Constants.SHOOTER_FLYWHEEL_LEFT);
                break;
            case PID_MODE:
                leftFlywheelFalcon.set(ControlMode.Velocity, periodic.flywheelDemand);
                rightFlywheelFalcon.set(ControlMode.Follower, Constants.SHOOTER_FLYWHEEL_LEFT);
                break;
            default:
                leftFlywheelFalcon.set(ControlMode.Disabled, 0);
                rightFlywheelFalcon.set(ControlMode.Disabled, 0);
                break;
        }
        switch (turretMode) {
            case OPEN_LOOP:
                turretControl.set(ControlMode.PercentOutput, periodic.turretDemand);
                break;
            case PID_MODE:
                turretControl.set(ControlMode.Position, periodic.turretDemand);
                break;
            default:
                turretControl.set(ControlMode.Disabled, 0);
                break;
        }
        System.out.println("Flywheel is in " + flywheelMode);
        System.out.println("Turret is in " + turretControl);
    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    @Override
    public void outputTelemetry() {

    }

    public void configLimelight() {
        //Forces led on
        table.getEntry("ledMode").setNumber(3);
        //Sets limelight's current pipeline to 0
        table.getEntry("pipeline").setNumber(0);
        //Sets the mode of the camera to vision processor mode
        table.getEntry("camMode").setNumber(0);
        //Defaults Limelight's snapshotting feature to off
        table.getEntry("snapshot").setNumber(0);
    }

    public void configTalons() {
        turretControl.config_kP(1, Constants.TURRET_CONTROL_PID_P);
        turretControl.config_kD(1, Constants.TURRET_CONTROL_PID_D);
        rightFlywheelFalcon.config_kP(1, Constants.RIGHTFLYWHEELFALCON_KP);
        rightFlywheelFalcon.config_kD(1, Constants.RIGHTFLYWHEELFALCON_KD);
        leftFlywheelFalcon.config_kP(1, Constants.LEFTFLYWHEELFALCON_KP);
        leftFlywheelFalcon.config_kD(1, Constants.LEFTFLYWHEELFALCON_KD);
    }

    /**
     * Called to reset and configure the subsystem
     */
    @Override
    public void reset() {
        periodic = new ShooterIO();
        configLimelight();
        configTalons();
    }
    /**
     * Takes in ta (See Limelight Docs) and outputs lateral distance from robot to target in inches
     * Equation came from a degree 2 polynomial regression on data points recorded manually
     * @return lateral distance from limelight lens to target in inches
     */

    public double limelightRanging()
    {
        //Equation that takes in ta (See Limelight Docs) and outputs distance from target in inches
        return 505 - 409 * periodic.targetArea + 119 * periodic.targetArea * periodic.targetArea; 
    }

    public double calculateRPM(double distance)
    {
        return 0; //TODO implement the equation to calculate the required inittal velocity and then convert to revolutions per Miniut
    }

    public double RPMToTicksPer100ms(double RPM)
    {
        return 0; //TODO Implement a convertion to convert RPM to tickes per 100ms
    }

    public void setFlywheelRPM(double demand){
        if(flywheelMode != MotorControlMode.PID_MODE)
            flywheelMode = MotorControlMode.PID_MODE;
        leftFlywheelFalcon.set(ControlMode.Velocity, demand); //TODO add safety that moves to hold current speed
        rightFlywheelFalcon.set(ControlMode.Follower, Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    public void setTurretRPM(double demand) {
        if (turretMode != MotorControlMode.PID_MODE)
            turretMode = MotorControlMode.PID_MODE;
        turretControl.set(ControlMode.Velocity, demand);
    }

    public void setFlywheelDemand(double newDemand) {
        if (flywheelMode != MotorControlMode.OPEN_LOOP)
            flywheelMode = MotorControlMode.OPEN_LOOP;
        periodic.flywheelDemand = newDemand;
    }

    public void setTurretDemand(double newDemand) {
        if (turretMode != MotorControlMode.OPEN_LOOP)
            turretMode = MotorControlMode.OPEN_LOOP;
        periodic.turretDemand = newDemand;
    }

    public boolean getRPMOnTarget()
    {
        return periodic.RPMOnTarget;
    }
    public void setRPMOnTarget(boolean isTarget)
    {
        periodic.RPMOnTarget = isTarget;
    }
    public double getRPMClosedLoopError(){
        return periodic.RPMClosedLoopError;
    }


    public Subsystem.PeriodicIO getLogger() {
        return periodic;
    }

    public enum MotorControlMode {
        DISABLED,
        OPEN_LOOP,
        PID_MODE;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class ShooterIO extends Subsystem.PeriodicIO {
        private double targetX = 0.0;
        private double targetY = 0.0;
        private double targetArea = 0.0;
        private double flywheelDemand = 0.0;
        private double flywheelRPM = 0.0;
        private double turretDemand = 0.0;
        private double turretRPM = 0.0;
        private boolean RPMOnTarget = false;
        private double RPMClosedLoopError = 0;
        private double rotationsClosedLoopError = 0;
    }
}
