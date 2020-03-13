package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.lib.util.Util;
import frc.robot.Constants;

public class Shooter extends Subsystem {
    private double[] tangent;
    private static Shooter m_Shooter = new Shooter();
    private NetworkTable limelight;
    private MotorControlMode flywheelMode = MotorControlMode.RAMP_UP;
    private MotorControlMode turretMode = MotorControlMode.OPEN_LOOP;
    private ShooterIO periodic;
    private TalonFX rightFlywheelFalcon, leftFlywheelFalcon;
    private TalonSRX turretControl;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");

    private Shooter() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Shooter/Turret/P", 0);
            SmartDashboard.putNumber("Shooter/Turret/I", 0);
            SmartDashboard.putNumber("Shooter/Turret/D", 0);
            SmartDashboard.putNumber("Shooter/Turret/F", 0);
            SmartDashboard.putNumber("Shooter/Flywheel/OffsetStep", Constants.FLYWHEEL_OFFSET_RPM_INCREMENT);
            SmartDashboard.putNumber("Shooter/Flywheel/OffsetMax", Constants.FLYWHEEL_MAX_RPM_OFFSET);
            SmartDashboard.putNumber("Shooter/Flywheel/OffsetMin", Constants.FLYWHEEL_MIN_RPM_OFFSET);
            SmartDashboard.putBoolean("Shooter/Turret/SaveChanges", false);
        }
        rightFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_RIGHT);
        leftFlywheelFalcon = new TalonFX(Constants.SHOOTER_FLYWHEEL_LEFT);
        turretControl = new TalonSRX(Constants.TURRET_CONTROL);
        rightFlywheelFalcon.setInverted(true);
        leftFlywheelFalcon.setInverted(false);
        turretControl.configContinuousCurrentLimit(10);
        tangent = new double[181];
        for (int i = 0; i <= 180; i++) {
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
        if (SmartDashboard.getBoolean("Shooter/Turret/SaveChanges", false) && Constants.DEBUG) {
            updateTurretPID(SmartDashboard.getNumber("Shooter/Turret/P", 0),
                    SmartDashboard.getNumber("Shooter/Turret/I", 0), SmartDashboard.getNumber("Shooter/Turret/D", 0),
                    SmartDashboard.getNumber("Shooter/Turret/F", 0));
        }
        periodic.limelightPrev = periodic.limelight_distance;
        periodic.limelight_distance = limelightRanging();
        periodic.limelightDelta = (periodic.limelightDelta + periodic.limelight_distance - periodic.limelightPrev)/2;
        periodic.turretEncoder = turretControl.getSelectedSensorPosition();
        periodic.flywheelVelocity = leftFlywheelFalcon.getSelectedSensorVelocity();
        periodic.operatorInput = Constants.SECOND.getPOV();
        periodic.turretAmps = turretControl.getSupplyCurrent();
        periodic.AmpsL = leftFlywheelFalcon.getSupplyCurrent();
        periodic.AmpsR = rightFlywheelFalcon.getSupplyCurrent();
        // Makes all values positive with -1 being 0 and 1 being 1
        periodic.operatorFlywheelInput = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0);
        periodic.targetX = tx.getDouble(0.0) + Constants.TURRET_OFFSET;
        periodic.targetV = tv.getDouble(0.0);
        periodic.targetY = ty.getDouble(0.0);
        periodic.onTarget = Math.abs(periodic.targetX) < Constants.TURRET_LOCKON_DELTA && periodic.targetV == 1;
        periodic.flywheelRPM = TicksPer100msToRPM(periodic.flywheelVelocity);
        periodic.RPMClosedLoopError = periodic.flywheelRPMDemand - periodic.flywheelRPM;
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                periodic.flywheelDemand = leftFlywheelFalcon.getSelectedSensorVelocity();
                periodic.flywheelRPMDemand = TicksPer100msToRPM(periodic.flywheelDemand);
            }

            @Override
            public void onLoop(double timestamp) {
                periodic.turretAngle = Rotation2d.fromDegrees(ticksToDegrees(periodic.turretEncoder));
                /*if (periodic.targetV == 1) {
                    turretMode = MotorControlMode.LIMELIGHT_MODE;
                }
*/
                switch (flywheelMode) {
                case OPEN_LOOP:
                    periodic.flywheelDemand = periodic.operatorFlywheelInput;
                    periodic.flywheelRPMDemand = periodic.operatorFlywheelInput * Constants.FLYWHEEL_MAX_RPM;
                    break;
                case PID_MODE:
                    periodic.flywheelRPMDemand = periodic.operatorFlywheelInput * Constants.FLYWHEEL_MAX_RPM;
                    periodic.flywheelDemand = RPMToTicksPer100ms(periodic.flywheelRPMDemand);
                    break;
                case RAMP_UP:
                    if (Util.epsilonEquals(periodic.flywheelRPM, Constants.FLYWHEEL_IDLE_RPM, 100)) {
                        setLimelightRPM();
                    } else {
                        periodic.flywheelRPMDemand = Math.min(
                                periodic.flywheelRPMDemand
                                        + (Constants.FLYWHEEL_IDLE_RPM / (Constants.FLYWHEEL_SPINUP_TIME)),
                                Constants.FLYWHEEL_IDLE_RPM + 200);
                        periodic.flywheelRPMDemand = Math.min(periodic.flywheelRPMDemand, Constants.FLYWHEEL_MAX_RPM);
                        periodic.flywheelDemand = RPMToTicksPer100ms(periodic.flywheelRPMDemand);
                    }
                    break;
                case LIMELIGHT_MODE:
                    if (periodic.limelight_distance > 60 && periodic.limelight_distance < 700) {
                        periodic.RPMGoal = (limelightRanging() * Constants.FLYWHEEL_RPM_PER_IN)
                        + Constants.FLYWHEEL_BASE_RPM + periodic.FlywheelBaseRPMOffset + periodic.limelightDelta * 0;
                        periodic.flywheelRPMDemand = Math.min(periodic.RPMGoal,
                                Constants.FLYWHEEL_MAX_RPM);
                        periodic.flywheelRPMDemand = Math.max(
                                periodic.RPMGoal,
                                Constants.FLYWHEEL_IDLE_RPM + periodic.FlywheelBaseRPMOffset);
                    } else {
                        periodic.flywheelRPMDemand = Constants.FLYWHEEL_IDLE_RPM + periodic.FlywheelBaseRPMOffset;
                    }
                    periodic.flywheelDemand = RPMToTicksPer100ms(periodic.flywheelRPMDemand);
                    break;
                default:
                    leftFlywheelFalcon.set(ControlMode.Disabled, 0);
                    break;
                }
                switch (turretMode) {
                case OPEN_LOOP:
                    if (periodic.operatorInput == 90 && !(periodic.turretEncoder > Constants.rightTurretLimit)) {
                        periodic.turretDemand = Constants.TURRET_MAX_SPEED;
                    } else if (periodic.operatorInput == 270 && !(periodic.turretEncoder < Constants.leftTurretLimit)) {
                        periodic.turretDemand = -Constants.TURRET_MAX_SPEED;
                    } else {
                        periodic.turretDemand = 0;
                    }
                    break;
                case PID_MODE:
                    if (periodic.turretEncoder < Constants.leftTurretLimit
                            || periodic.turretEncoder > Constants.rightTurretLimit)
                        turretMode = MotorControlMode.DISABLED;
                    periodic.turretDemand = -1 * degreesToTicks(Drive.getInstance().getHeading().getDegrees());
                    if (periodic.turretDemand <= Constants.leftTurretLimit) {
                        periodic.turretDemand = Constants.leftTurretLimit;
                    }
                    if (periodic.turretDemand > Constants.rightTurretLimit) {
                        periodic.turretDemand = Constants.rightTurretLimit;
                    }
                    break;
                case LIMELIGHT_MODE:
                    if (periodic.turretEncoder < Constants.leftTurretLimit
                            || periodic.turretEncoder > Constants.rightTurretLimit)
                        turretMode = MotorControlMode.DISABLED;
                    periodic.turretDemand = limelightGoalAngle();
                    break;
                default:
                    turretControl.set(ControlMode.Disabled, 0);
                    break;
                }
                if (periodic.targetV == 1) {
                    limelight.getEntry("snapshot").setNumber(1);
                } else {
                    limelight.getEntry("snapshot").setNumber(0);
                }
            }

            @Override
            public void onStop(double timestamp) {
                turretControl.set(ControlMode.Disabled, 0);
                periodic.targetX = 0;
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
            break;
        case PID_MODE:
        case LIMELIGHT_MODE:
        case RAMP_UP:
            leftFlywheelFalcon.set(ControlMode.Velocity, periodic.flywheelDemand);
            break;
        default:
            leftFlywheelFalcon.set(ControlMode.Disabled, 0);
            break;
        }
        switch (turretMode) {
        case OPEN_LOOP:
            turretControl.set(ControlMode.PercentOutput, periodic.turretDemand);
            break;
        case PID_MODE:case LIMELIGHT_MODE:
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
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Shooter/Flywheel/RPMError", periodic.RPMClosedLoopError);
            SmartDashboard.putNumber("Shooter/Flywheel/AmpsL", periodic.AmpsL);
            SmartDashboard.putNumber("Shooter/Flywheel/AmpsR", periodic.AmpsR);
            SmartDashboard.putNumber("Shooter/Turret/OperatorInput", periodic.operatorInput);
            SmartDashboard.putNumber("Shooter/Turret/Demand", periodic.turretDemand);
            SmartDashboard.putNumber("Shooter/Turret/Range (in)", limelightRanging());
            SmartDashboard.putNumber("Shooter/Turret/Encoder", periodic.turretEncoder);
            SmartDashboard.putNumber("Shooter/Turret/EncoderGoal", limelightGoalAngle());
            SmartDashboard.putNumber("Shooter/Turret/AngleError",
                    ticksToDegrees(limelightGoalAngle() - periodic.turretEncoder) + Constants.TURRET_OFFSET);
            SmartDashboard.putString("Shooter/Turret/Mode", "" + turretMode);
            SmartDashboard.putNumber("Shooter/Flywheel/OperatorInput", periodic.operatorFlywheelInput);
            SmartDashboard.putNumber("Shooter/Flywheel/Demand", periodic.flywheelDemand);
            SmartDashboard.putNumber("Shooter/Flywheel/RPMDemand", periodic.flywheelRPMDemand);
            SmartDashboard.putString("Shooter/Flywheel/Mode", "" + flywheelMode);
            SmartDashboard.putNumber("Shooter/Flywheel/Velocity", periodic.flywheelVelocity);
        }
        SmartDashboard.putNumber("Shooter/Flywheel/CurrOffset", periodic.FlywheelBaseRPMOffset);
        SmartDashboard.putBoolean("Shooter/Flywheel/AmpDeltaError",
                Math.abs(periodic.AmpsL - periodic.AmpsR) > Constants.FLYWHEEL_DELTA_AMPS);
        SmartDashboard.putNumber("Shooter/Turret/Amps", periodic.turretAmps);
        SmartDashboard.putBoolean("Shooter/Turret/OnTarget", periodic.onTarget);
        SmartDashboard.putNumber("Shooter/Turret/Angle", (ticksToDegrees(periodic.turretEncoder) + 360) % 360);
        SmartDashboard.putNumber("Shooter/Flywheel/RPM", TicksPer100msToRPM(periodic.flywheelVelocity));
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
        turretControl.config_kI(0, Constants.TURRET_ANGLE_KI);
        turretControl.config_kD(0, Constants.TURRET_ANGLE_KD);
        turretControl.configMaxIntegralAccumulator(0, 0);
        turretControl.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretControl.setSelectedSensorPosition(0);
        turretControl.configContinuousCurrentLimit(15);
        turretControl.setInverted(true);
        turretControl.setSensorPhase(true);
        turretControl.configMotionAcceleration((int) degreesToTicks(90));
        turretControl.configMotionCruiseVelocity((int) degreesToTicks(90));

        rightFlywheelFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        rightFlywheelFalcon.configVoltageCompSaturation(Constants.VOLTAGE_COMP_FLYWHEEL);
        rightFlywheelFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0);
        rightFlywheelFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0);
        rightFlywheelFalcon.follow(leftFlywheelFalcon);

        leftFlywheelFalcon.config_kP(0, Constants.TURRET_LEFT_FLY_KP);
        leftFlywheelFalcon.config_kD(0, Constants.TURRET_LEFT_FLY_KD);
        leftFlywheelFalcon.config_kF(0, Constants.TURRET_LEFT_FLY_KF);
        leftFlywheelFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFlywheelFalcon.setNeutralMode(NeutralMode.Coast);
        leftFlywheelFalcon.configVoltageCompSaturation(Constants.VOLTAGE_COMP_FLYWHEEL);
        disable();
    }

    public void updateTurretPID(double P, double I, double D, double F) {
        turretControl.config_kP(0, P);
        turretControl.config_kI(0, I);
        turretControl.config_kD(0, D);
        turretControl.config_kF(0, F);
    }

    public void disable() {
        turretMode = MotorControlMode.OPEN_LOOP;
        flywheelMode = MotorControlMode.RAMP_UP;
    }

    /**
     * Called to reset and configure the subsystem
     */
    @Override
    public void reset() {
        periodic = new ShooterIO();
        flywheelMode = MotorControlMode.DISABLED;
        turretMode = MotorControlMode.OPEN_LOOP;
        configLimelight();
        configTalons();
    }

    public void softStart() {
        flywheelMode = MotorControlMode.RAMP_UP;
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

    public boolean onTarget() {
        return periodic.onTarget;
    }

    public void setRampUp() {
        if (flywheelMode != MotorControlMode.RAMP_UP) {
            flywheelMode = MotorControlMode.RAMP_UP;
            periodic.rampUpTime = Timer.getFPGATimestamp();
        }
    }

    public double RPMToTicksPer100ms(double RPM) {
        return RPM * Constants.FLYWHEEL_TP100MS; // .1 * 2048 * 4/60
    }

    public double TicksPer100msToRPM(double Ticks) {
        return Ticks / Constants.FLYWHEEL_TP100MS; // .1 * 2048 * 4 / 60
    }

    public void setFlywheelRPM(double demand) {
        if (flywheelMode != MotorControlMode.PID_MODE) {
            flywheelMode = MotorControlMode.PID_MODE;
            leftFlywheelFalcon.set(ControlMode.Velocity, demand); // TODO add safety that moves to hold current speed
        }
    }

    public double getShooterAngle() {
        return (ticksToDegrees(periodic.turretEncoder) + 360) % 360;
    }

    public void setLimelightRPM() {
        if (flywheelMode != MotorControlMode.LIMELIGHT_MODE)
            flywheelMode = MotorControlMode.LIMELIGHT_MODE;
    }

    public void setTurretLimelightControl(double demand) {
        if (turretMode != MotorControlMode.LIMELIGHT_MODE)
            turretMode = MotorControlMode.LIMELIGHT_MODE;
    }

    public void setTurretFieldCentric() {
        if (turretMode != MotorControlMode.PID_MODE)
            turretMode = MotorControlMode.PID_MODE;
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

    public void increaseRPMOffset() {
        periodic.FlywheelBaseRPMOffset = Math.min(
                periodic.FlywheelBaseRPMOffset + Constants.FLYWHEEL_OFFSET_RPM_INCREMENT,
                Constants.FLYWHEEL_MAX_RPM_OFFSET);
    }

    public void decreaseRPMOffset() {
        periodic.FlywheelBaseRPMOffset = Math.max(
                periodic.FlywheelBaseRPMOffset - Constants.FLYWHEEL_OFFSET_RPM_INCREMENT,
                Constants.FLYWHEEL_MIN_RPM_OFFSET);
    }

    public void setRPMOnTarget(boolean isTarget) {
        periodic.RPMOnTarget = isTarget;
    }

    public double getRPMClosedLoopError() {
        return periodic.RPMClosedLoopError;
    }

    public Subsystem.PeriodicIO getLogger() {
        return periodic;
    }

    public enum MotorControlMode {
        DISABLED, OPEN_LOOP, PID_MODE, LIMELIGHT_MODE, RECENTER_MODE, RAMP_UP;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public double getSupplyCurrent() {
        return periodic.turretAmps;
    }

    /**
     * 
     * @param angle angle offset given by the limelight (tx)
     * @return goal ticks to turret ticks
     */
    public double limelightRanging() {
        if (periodic.targetV == 0.0) {
            return 0.0;
        }
        return (98.5 - Constants.LIMELIGHT_HIGHT) / limeTan((Math.toRadians(Constants.LIMELIGHT_PITCH + periodic.targetY)));
    }

    public double limelightGoalAngle() {
        double goal = 0;
        if (periodic.targetV == 1) {
            goal = degreesToTicks(periodic.targetX) + periodic.turretEncoder;
            if (goal <= Constants.leftTurretLimit) {
                goal = Constants.leftTurretLimit;
            }
            if (goal > Constants.rightTurretLimit) {
                goal = Constants.rightTurretLimit;
            }
        } else {
            goal = periodic.turretEncoder;
        }
        return goal;
    }

    public double limeTan(double angle)
    {
        double angle2 = angle * angle;
        return (angle * (1 - angle2/6)) / (1 - (angle2 * (1 - angle2/12) / 2));
    }

    public class ShooterIO extends Subsystem.PeriodicIO {
        public double RPMGoal = 0.0;
        public double limelight_distance = 0.0;
        public double limelightPrev = 0.0;
        public double limelightDelta = 0.0;
        public int FlywheelBaseRPMOffset = 0;
        public double rampUpTime = 0.0;
        public double AmpsL = 0.0;
        public double AmpsR = 0.0;
        public double targetX = 0.0;
        public double targetY = 0.0;
        public double targetV = 0.0;
        public double flywheelDemand = 0.0;
        public double flywheelRPMDemand = 0.0;
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
        public boolean onTarget = false;
    }
}
