package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Shooter extends Subsystem {
    private double[] tangent;
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
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry camtran = table.getEntry("camtran");

    private Shooter() {
        SmartDashboard.putNumber("Shooter/Turret/P", 0);
        SmartDashboard.putNumber("Shooter/Turret/D", 0);
        SmartDashboard.putBoolean("Shooter/Turret/SaveChanges", false);
        rightFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_LEFT);
        leftFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_RIGHT);
        turretControl = new TalonSRX(Constants.TURRET_CONTROL);
        rightFlywheelFalcon.setInverted(true);
        leftFlywheelFalcon.setInverted(false);
        turretControl.configContinuousCurrentLimit(10);
        tangent = new double[181];
        for(int i = 0; i <= 180; i++) {
            tangent[i] = Math.tan(Math.toRadians((double) i / 2));
        }
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
        if (SmartDashboard.getBoolean("Shooter/Turret/SaveChanges", false)) {
            updateTurretPID(SmartDashboard.getNumber("Shooter/Turret/P", 0),
                    SmartDashboard.getNumber("Shooter/Turret/D", 0));
        }
        periodic.turretEncoder = turretControl.getSelectedSensorPosition();
        periodic.flywheelClosedLoopError = leftFlywheelFalcon.getClosedLoopError();
        periodic.flywheelVelocity = leftFlywheelFalcon.getSelectedSensorVelocity();
        periodic.operatorInput = Constants.SECOND.getPOV();
        periodic.turretAmps = turretControl.getStatorCurrent();
        periodic.operatorFlywheelInput = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0); // Makes all
                                                                                                        // values
                                                                                                        // positive with
                                                                                                        // -1 being 0
                                                                                                        // and 1 being 1
        periodic.targetArea = ta.getDouble(0.0);
        periodic.targetX = tx.getDouble(0.0);
        periodic.targetV = tv.getDouble(0.0);
        periodic.targetY = ty.getDouble(0.0);
        periodic.RPMClosedLoopError = rightFlywheelFalcon.getClosedLoopError();
        periodic.rotationsClosedLoopError = turretControl.getClosedLoopError();
        if(turretControl.getSelectedSensorPosition() >= 6000) {
            periodic.canUnfold = true;
        } else {
            periodic.canUnfold = false;
        }
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                periodic.turretAngle = Rotation2d.fromDegrees(ticksToDegrees(periodic.turretEncoder));
                
                switch (flywheelMode) {
                case OPEN_LOOP:
                    periodic.flywheelDemand = periodic.operatorFlywheelInput;
                    break;
                case PID_MODE:
                    periodic.flywheelDemand = RPMToTicksPer100ms(periodic.operatorFlywheelInput);
                    break;
                default:
                    leftFlywheelFalcon.set(ControlMode.Disabled, 0);
                    rightFlywheelFalcon.set(ControlMode.Disabled, 0);
                    break;
                }
                switch (turretMode) {
                case OPEN_LOOP:
                    if (periodic.operatorInput == 90 && !(periodic.turretEncoder > Constants.rightTurretLimit)) {
                        periodic.turretDemand = .2;
                    } else if (periodic.operatorInput == 270 && !(periodic.turretEncoder < Constants.leftTurretLimit)) {
                        periodic.turretDemand = -.2;
                    } else {
                        periodic.turretDemand = 0;
                    }
                    break;
                case PID_MODE:
                if(periodic.turretEncoder < Constants.leftTurretLimit || periodic.turretEncoder > Constants.rightTurretLimit)
                turretMode = MotorControlMode.DISABLED;
                    periodic.turretDemand = limelightGoalAngle();
                    break;
                default:
                    turretControl.set(ControlMode.Disabled, 0);
                    break;
                }
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
        case LIMELIGHT_MODE:
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
        case RECENTER_MODE:
            turretControl.set(ControlMode.Position, periodic.turretDemand);
            break;
        default:
            turretControl.set(ControlMode.Disabled, 0);
            break;
        }
    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter/Turret/Amps", periodic.turretAmps);
        SmartDashboard.putBoolean("Shooter/Turret/OnTarget", isTurretOnTarget());
        SmartDashboard.putNumber("Shooter/Turret/Encoder", periodic.turretEncoder);
        SmartDashboard.putNumber("Shooter/Turret/Range (in)", limelightRanging());
        SmartDashboard.putNumber("Shooter/Turret/EncoderGoal", limelightGoalAngle());
        SmartDashboard.putNumber("Shooter/Turret/OperatorInput", periodic.operatorInput);
        SmartDashboard.putNumber("Shooter/Turret/Demand", periodic.turretDemand);
        SmartDashboard.putString("Shooter/Turret/Mode", "" + turretMode);
        SmartDashboard.putNumber("Shooter/Turret/Angle", (ticksToDegrees(periodic.turretEncoder) + 360) % 360);
        SmartDashboard.putNumber("Shooter/Flywheel/OperatorInput", periodic.operatorFlywheelInput);
        SmartDashboard.putNumber("Shooter/Flywheel/Demand", periodic.flywheelDemand);
        SmartDashboard.putString("Shooter/Flywheel/Mode", "" + flywheelMode);
        SmartDashboard.putNumber("Shooter/Flywheel/Velocity", periodic.flywheelVelocity);
        SmartDashboard.putNumber("Shooter/Flywheel/RPM", TicksPer100msToRPM(periodic.flywheelVelocity));
        SmartDashboard.putBoolean("Shooter/Turret/Can Unfold", periodic.canUnfold);
    }

    public void configLimelight() {
        // Forces led on
        table.getEntry("ledMode").setNumber(3);
        // Sets limelight's current pipeline to 0
        table.getEntry("pipeline").setNumber(0);
        // Sets the mode of the camera to vision processor mode
        table.getEntry("camMode").setNumber(0);
        // Defaults Limelight's snapshotting feature to off
        table.getEntry("snapshot").setNumber(0);
    }

    public void configTalons() {
        turretControl.config_kP(0, Constants.TURRET_ANGLE_KP);
        turretControl.config_kI(0, 0.0023);
        turretControl.config_kD(0, Constants.TURRET_ANGLE_KD);
        turretControl.configMaxIntegralAccumulator(0, 10);
        turretControl.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretControl.setSelectedSensorPosition(0);
        turretControl.configContinuousCurrentLimit(15);
        turretControl.setInverted(true);
        turretControl.setSensorPhase(true);
        turretControl.configMotionAcceleration((int)degreesToTicks(90));
        turretControl.configMotionCruiseVelocity((int)degreesToTicks(90));

        rightFlywheelFalcon.config_kP(0, Constants.TURRET_RIGHT_FLY_KP);
        rightFlywheelFalcon.config_kD(0, Constants.TURRET_RIGHT_FLY_KD);
        rightFlywheelFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        rightFlywheelFalcon.configVoltageCompSaturation(Constants.VOLTAGE_COMP_TURRET);
        
        leftFlywheelFalcon.config_kP(0, Constants.TURRET_LEFT_FLY_KP);
        leftFlywheelFalcon.config_kD(0, Constants.TURRET_LEFT_FLY_KD);
        leftFlywheelFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        leftFlywheelFalcon.configVoltageCompSaturation(Constants.VOLTAGE_COMP_TURRET);
        disable();
    }

    public void updateTurretPID(double P, double D) {
        turretControl.config_kP(0, P);
        turretControl.config_kD(0, D);
    }

    public void disable() {
        turretMode = MotorControlMode.DISABLED;
        flywheelMode = MotorControlMode.DISABLED;
    }

    /**
     * Called to reset and configure the subsystem
     */
    @Override
    public void reset() {
        periodic = new ShooterIO();
        flywheelMode = MotorControlMode.DISABLED;
        turretMode = MotorControlMode.DISABLED;
        configLimelight();
        configTalons();
    }

    /**
     * Method that maps the raw input from the slider on the EXTREME 3D and convert
     * the value to a 0 - 1 bottom to top map
     * 
     */

    public double degreesToTicks(double degree) {
        // implement a ticks to degrees method
        return degree * Constants.TURRET_DEGREES_TO_TICKS; // empiricly mesured
    }

    public double ticksToDegrees(double degree) {
        return degree / Constants.TURRET_DEGREES_TO_TICKS; // 360 / (4096 * 9.5)
    }

    /**
     * Takes in ta (See Limelight Docs) and outputs lateral distance from robot to
     * target in inches Equation came from a degree 2 polynomial regression on data
     * points recorded manually
     * 
     * @return lateral distance from limelight lens to target in inches
     */

    public double calculateRPM(double distance) {
        return 0; // TODO implement the equation to calculate the required inittal velocity and
                  // then convert to revolutions per Miniut
    }

    public double RPMToTicksPer100ms(double RPM) {
        return RPM * 3.413; // .1 * 2048 * 4/60
    }

    public double TicksPer100msToRPM(double Ticks) {
        return Ticks / 3.413; // .1 * 2048 * 4 / 60
    }

    public void setFlywheelRPM(double demand) {
        if (flywheelMode != MotorControlMode.PID_MODE)
            flywheelMode = MotorControlMode.PID_MODE;
        leftFlywheelFalcon.set(ControlMode.Velocity, demand); // TODO add safety that moves to hold current speed
        rightFlywheelFalcon.set(ControlMode.Follower, Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    public void setTurretRPM(double demand) {
        if (turretMode != MotorControlMode.PID_MODE)
            turretMode = MotorControlMode.PID_MODE;
        turretControl.set(ControlMode.Velocity, demand);
    }

    public void setTurretCenter(double angle) {
        if (turretMode != MotorControlMode.RECENTER_MODE)
            turretMode = MotorControlMode.RECENTER_MODE;
        periodic.turretDemand = degreesToTicks(angle);
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

    public boolean getRPMOnTarget() {
        return periodic.RPMOnTarget;
    }

    public void setRPMOnTarget(boolean isTarget) {
        periodic.RPMOnTarget = isTarget;
    }

    public double getRPMClosedLoopError() {
        return periodic.RPMClosedLoopError;
    }

    public boolean isTurretOnTarget() {
        return Math.abs(periodic.targetX) < 2 && periodic.targetV == 1;
    }

    public Subsystem.PeriodicIO getLogger() {
        return periodic;
    }

    public enum MotorControlMode {
        DISABLED, OPEN_LOOP, PID_MODE, LIMELIGHT_MODE, RECENTER_MODE, LEFT45_MODE, RIGHT45_MODE;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    /**
     * 
     * @param angle angle offset given by the limelight (tx)
     * @return goal ticks to turret ticks
     */
    public double limelightRanging() {
        // Equation that takes in ta (See Limelight Docs) and outputs distance from
        // target in inches
        // TODO need to test data points based on actual bot
        if(periodic.targetY == 0.0) {
            return 0.0;
        }
        return (98.5-Constants.LIMELIGHT_HIGHT) / tangent[(int) ((Constants.LIMELIGHT_PITCH + periodic.targetY) * 2)];
    }

    public double limelightGoalAngle() {
        double goal = degreesToTicks(periodic.targetX) + periodic.turretEncoder;
        if (periodic.turretEncoder + degreesToTicks(periodic.targetX) <= Constants.leftTurretLimit) {
            goal = Constants.leftTurretLimit;
        }
        if (periodic.turretEncoder + degreesToTicks(periodic.targetX) > Constants.rightTurretLimit) {
            goal = Constants.rightTurretLimit;
        }
        return goal;
    }

    public boolean canUnfold() {
        return periodic.canUnfold;
    }

    public class ShooterIO extends Subsystem.PeriodicIO {
        public double targetX = 0.0;
        public double targetY = 0.0;
        public double targetV = 0.0;
        public double targetArea = 0.0;
        public double flywheelDemand = 0.0;
        public double flywheelRPM = 0.0;
        public double turretDemand = 0.0;
        public double turretEncoder = 0.0;
        public Rotation2d turretAngle = Rotation2d.identity();
        public boolean RPMOnTarget = false;
        public double RPMClosedLoopError = 0;
        public double rotationsClosedLoopError = 0;
        public int operatorInput = 0;
        public double operatorFlywheelInput = 0;
        public double flywheelVelocity = 0;
        public double flywheelClosedLoopError = 0;
        public double turretAmps = 0.0;
        public boolean canUnfold = false;
    }
}
