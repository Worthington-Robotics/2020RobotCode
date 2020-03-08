package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.AdaptivePurePursuitController;
import frc.lib.control.Path;
import frc.lib.drivers.PIDF;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.Kinematics;

public class Drive extends Subsystem {

    // construct one and only 1 instance of this class
    private static Drive m_DriveInstance = new Drive();

    public static Drive getInstance() {
        return m_DriveInstance;
    }

    public PeriodicIO getLogger() {
        return periodic;
    }

    // used internally for data
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private boolean mOverrideTrajectory = false;
    private DriveIO periodic;
    private PigeonIMU pigeonIMU;
    private DoubleSolenoid trans;
    private TalonFX driveFrontLeft, driveBackRight, driveFrontRight, driveBackLeft;
    private PIDF anglePID;
    private AdaptivePurePursuitController pathFollowingController;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                }

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    if (Constants.ENABLE_MP_TEST_MODE && DriverStation.getInstance().isTest()) {
                        mDriveControlState = DriveControlState.PROFILING_TEST;
                    }
                    if (periodic.inverse) {
                        periodic.operatorInput[1] *= -1;
                    }
                    switch (mDriveControlState) {
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    case PROFILING_TEST:
                        if (Constants.RAMPUP) {
                            periodic.left_demand = periodic.ramp_Up_Counter * .0025 + .01;
                            periodic.right_demand = periodic.ramp_Up_Counter * .0025 + .01;
                            periodic.ramp_Up_Counter++;
                        } else if (DriverStation.getInstance().isTest()) {
                            periodic.left_demand = radiansPerSecondToTicksPer100ms(
                                    inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                            periodic.right_demand = radiansPerSecondToTicksPer100ms(
                                    inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                        }
                        break;
                    case OPEN_LOOP:
                        if (!DriverStation.getInstance().isAutonomous()) {
                            setOpenLoop(arcadeDrive(periodic.operatorInput[1], periodic.operatorInput[0]));
                        }
                        break;
                    case ANGLE_PID:
                        periodic.PIDOutput = anglePID.update(periodic.gyro_heading.getDegrees());
                        DriveSignal drivesignal = arcadeDrive(periodic.operatorInput[1], periodic.PIDOutput);
                        periodic.right_demand = drivesignal.getRight();
                        periodic.left_demand = drivesignal.getLeft();
                        break;
                    default:
                        System.out.println("You fool, unexpected control state");

                    }
                    if (Constants.DEBUG) {
                        if (periodic.savePIDSettings) {
                            System.out.print("Configed PID");
                            configPID();
                        }
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (SmartDashboard.getBoolean("Drive/PID/SaveChanges", false) && Constants.DEBUG) {
            updateDrivePID(SmartDashboard.getNumber("Drive/PID/P", 0), SmartDashboard.getNumber("Drive/PID/I", 0),
                    SmartDashboard.getNumber("Drive/PID/D", 0), SmartDashboard.getNumber("Drive/PID//F", 0));
        }
        if (periodic.TransState) {
            periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK_SHIFTED);
        } else {
            periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
        }

        periodic.AnglePIDError = anglePID.getError();
        periodic.gyro_heading = Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).rotateBy(periodic.gyro_offset);

        periodic.left_velocity_ticks_per_100ms = driveFrontLeft.getSelectedSensorVelocity();
        periodic.right_velocity_ticks_per_100ms = driveFrontRight.getSelectedSensorVelocity();

        double prevLeftTicks = periodic.left_pos_ticks;
        double prevRightTicks = periodic.right_pos_ticks;
        periodic.left_pos_ticks = driveFrontLeft.getSelectedSensorPosition();
        periodic.right_pos_ticks = driveFrontRight.getSelectedSensorPosition();

        double deltaLeftTicks = ((periodic.left_pos_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        periodic.left_distance += deltaLeftTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        double deltaRightTicks = ((periodic.right_pos_ticks - prevRightTicks) / 4096.0) * Math.PI;
        periodic.right_distance += deltaRightTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;

        periodic.rightCurrent = driveFrontRight.getSupplyCurrent();
        periodic.leftCurrent = driveFrontLeft.getSupplyCurrent();

    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // System.out.println(mDriveControlState);
        if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveControlState == DriveControlState.ANGLE_PID
                || (mDriveControlState == DriveControlState.PROFILING_TEST && Constants.RAMPUP)) {
            driveFrontLeft.set(ControlMode.PercentOutput, periodic.left_demand);
            driveFrontRight.set(ControlMode.PercentOutput, periodic.right_demand);
        } else {
            driveFrontLeft.set(ControlMode.Velocity, periodic.left_demand);
            driveFrontRight.set(ControlMode.Velocity, periodic.right_demand);
        }
        trans.set(periodic.TransState ? Value.kForward : Value.kReverse);
    }

    private Drive() {
        SmartDashboard.putBoolean("Drive/PID/SaveChanges", false);
        SmartDashboard.putNumber("Drive/PID/P", 0);
        SmartDashboard.putNumber("Drive/PID/I", 0);
        SmartDashboard.putNumber("Drive/PID/D", 0);
        SmartDashboard.putNumber("Drive/PID/F", 0);
        anglePID = new PIDF(Constants.ANGLE_KP, Constants.ANGLE_KD);
        anglePID.setContinuous(true);
        driveFrontLeft = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        driveBackLeft = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveBackRight = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        pigeonIMU = new PigeonIMU(Constants.PIGION_ID);
        trans = new DoubleSolenoid(Constants.TRANS_LOW_ID, Constants.TRANS_HIGH_ID);
        configTalons();
        reset();

    }

    public PIDF getAnglePID() {
        return anglePID;
    }

    public void setTrans(boolean state) {
        periodic.TransState = state;
    }

    public synchronized Rotation2d getHeading() {
        return periodic.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());
        periodic.gyro_offset = heading.rotateBy(Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + periodic.gyro_offset.getDegrees());
        periodic.gyro_heading = heading;
    }

    public void updateDrivePID(double P, double I, double D, double F) {
        driveFrontLeft.config_kP(0, P);
        driveFrontLeft.config_kI(0, I);
        driveFrontLeft.config_kD(0, D);
        driveFrontLeft.config_kF(0, F);

        driveFrontRight.config_kP(0, P);
        driveFrontRight.config_kI(0, I);
        driveFrontRight.config_kD(0, D);
        driveFrontRight.config_kF(0, F);
    }

    public double getLeftEncoderRotations() {
        return periodic.left_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodic.right_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodic.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodic.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public void reset() {
        mOverrideTrajectory = false;
        periodic = new DriveIO();
        setHeading(Rotation2d.fromDegrees(0));
        resetEncoders();
    }

    private void resetEncoders() {
        driveFrontRight.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSelectedSensorPosition(0, 0, 0);
    }

    private void configPID() {
        anglePID.setPID(periodic.PIDPUpdate, 0, periodic.PIDDUpdate);
    }

    private void configTalons() {
        // primary closed-loop, 100 ms timeout
        ErrorCode sensorPresent = driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        // DO NOT FORGET THIS! use 5ms packet time on feedback
        driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        driveFrontLeft.setSensorPhase(true);
        driveFrontLeft.selectProfileSlot(0, 0);
        driveFrontLeft.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveFrontLeft.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveFrontLeft.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveFrontLeft.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveFrontLeft.config_IntegralZone(0, 300);
        driveFrontLeft.setInverted(true);
        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveFrontLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontLeft.enableVoltageCompensation(true);
        driveFrontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveBackLeft.setInverted(true);
        driveBackLeft.setNeutralMode(NeutralMode.Brake);
        driveBackLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackLeft.enableVoltageCompensation(true);
        driveBackLeft.follow(driveFrontLeft);
        driveBackLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0);
        driveBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0);

        // primary closed-loop, 100ms timeout
        sensorPresent = driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect right encoder: " + sensorPresent, false);
        }
        // DO NOT FORGET THIS! use 5ms packet time on feedback
        driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        driveFrontRight.setSensorPhase(true);
        driveFrontRight.selectProfileSlot(0, 0);
        driveFrontRight.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveFrontRight.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveFrontRight.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveFrontRight.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveFrontRight.config_IntegralZone(0, 300);
        driveFrontRight.setInverted(false);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveFrontRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontRight.enableVoltageCompensation(true);
        driveFrontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveBackRight.setInverted(false);
        driveBackRight.setNeutralMode(NeutralMode.Brake);
        driveBackRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackRight.enableVoltageCompensation(true);
        driveBackRight.follow(driveFrontRight);
        driveBackRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0);
        driveBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0);

    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            if (!mOverrideTrajectory) {
                Pose2d robot_pose = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue();
                Twist2d command = pathFollowingController.update(robot_pose, Timer.getFPGATimestamp());
                DriveSignal setpoint = Kinematics.inverseKinematics(command);

                // Scaling the controler to a set max velocity
                double max_vel = 0;
                max_vel = Math.max(max_vel, Math.abs(setpoint.getRight()));
                max_vel = Math.max(max_vel, Math.abs(setpoint.getLeft()));
                if (max_vel > Constants.ROBOT_MAX_VELOCITY) {
                    double scaling = Constants.ROBOT_MAX_VELOCITY / max_vel;
                    setpoint = new DriveSignal(setpoint.getLeft() * scaling, setpoint.getRight() * scaling);
                }
                setpoint = new DriveSignal(
                        radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(setpoint.getRight())),
                        radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(setpoint.getLeft())));
                setVelocity(setpoint, DriveSignal.NEUTRAL);
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mDriveControlState = DriveControlState.OPEN_LOOP;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    /**
     * Configure talons for open loop control
     *
     * @param signal input to drive train
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("Switching to open loop");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
    }

    /**
     * Configure for Angle PID control
     */
    public synchronized void setAnglePidLoop(DriveSignal signal, double angle) {
        if (mDriveControlState != DriveControlState.ANGLE_PID) {
            System.out.println("Switching to angle control");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.ANGLE_PID;
        }
        anglePID.setPoint(angle);
        periodic.gyro_pid_angle = angle;
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
    }

    public boolean getPIDOnTarget() {
        return anglePID.onTarget(Constants.ANGLE_PID_EPISLON);
    }

    public void setInverse(boolean isInverse) {
        periodic.inverse = isInverse;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            System.out.println("Switching to velocity control");
            driveFrontLeft.set(ControlMode.Velocity, 0);
            driveFrontRight.set(ControlMode.Velocity, 0);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
    }

    public boolean isDoneWithTrajectory() {
        return (mDriveControlState == DriveControlState.PATH_FOLLOWING && pathFollowingController.isDone())
                || mDriveControlState != DriveControlState.PATH_FOLLOWING;
    }

    public void outputTelemetry() {
        double[] PIDData = anglePID.getPID();

        // SmartDashboard.putNumber("Drive/Gyro/CurAngle",
        // periodic.gyro_heading.getDegrees());
        SmartDashboard.putBoolean("isInverse", periodic.inverse);

        SmartDashboard.putNumber("Drive/AnglePID/Set Point", periodic.gyro_pid_angle);
        SmartDashboard.putNumber("Drive/AnglePID/Error", periodic.AnglePIDError);

        SmartDashboard.putString("Drive/Drive State", mDriveControlState.toString());
        SmartDashboard.putNumberArray("Drive/Stick", periodic.operatorInput);
        SmartDashboard.putBoolean("Drive/Shift", periodic.TransState);
        SmartDashboard.putNumber("Drive/Error/X", periodic.error.x());
        SmartDashboard.putNumber("Drive/Error/Y", periodic.error.y());

        SmartDashboard.putNumber("Drive/Left/Current", periodic.leftCurrent);
        SmartDashboard.putNumber("Drive/Left/Demand", periodic.left_demand);
        SmartDashboard.putNumber("Drive/Left/Talon Velocity", periodic.left_velocity_ticks_per_100ms);
        // SmartDashboard.putNumber("Drive/Left/Talon Error", periodic.left_error);
        // SmartDashboard.putNumber("Drive/Left/Talon Voltage Out",
        // driveFrontLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Left/Encoder Counts", periodic.left_pos_ticks);

        SmartDashboard.putNumber("Drive/Right/Current", periodic.rightCurrent);
        SmartDashboard.putNumber("Drive/Right/Demand", periodic.right_demand);
        SmartDashboard.putNumber("Drive/Right/Talon Velocity", periodic.right_velocity_ticks_per_100ms);
        // SmartDashboard.putNumber("Drive/Right/Talon Error", periodic.right_error);
        // SmartDashboard.putNumber("Drive/Right/Talon Voltage Out",
        // driveFrontRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Right/Encoder Counts", periodic.right_pos_ticks);

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Drive/AnglePID/P", PIDData[0]);
            SmartDashboard.putNumber("Drive/AnglePID/D", PIDData[2]);
        }
    }

    enum DriveControlState {
        OPEN_LOOP, PATH_FOLLOWING, PROFILING_TEST, GYRO_LOCK, ANGLE_PID;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class DriveIO extends PeriodicIO {
        // INPUTS
        public double left_pos_ticks = 0;
        // public double left_error = 0;
        public double left_velocity_ticks_per_100ms = 0;
        public double leftCurrent = 0;

        public double right_pos_ticks = 0;
        // public double right_error = 0;
        public double right_velocity_ticks_per_100ms = 0;
        public double rightCurrent = 0;

        public Rotation2d gyro_heading = Rotation2d.identity();
        public Rotation2d gyro_offset = Rotation2d.identity();
        public double gyro_pid_angle = 0;
        public double AnglePIDError = 0;

        public Translation2d error = new Translation2d(0, 0);
        public double[] operatorInput = { 0, 0, 0 };
        public boolean inverse = false;
        public double PIDOutput = 0;

        // Smartdashboard Settings
        private double PIDDUpdate = 0;
        private double PIDPUpdate = 0;
        private boolean savePIDSettings = false;

        // OUTPUTS
        public double ramp_Up_Counter = 0;
        public boolean TransState = false;

        // public double left_accl = 0.0;
        public double left_demand = 0.0;
        public double left_distance = 0.0;

        // public double right_accl = 0.0;
        public double right_demand = 0.0;
        public double right_distance = 0.0;

    }

    public LogData logData() {
        return periodic;
    }

    /**
     * internal methods beyond this point
     **/

    public synchronized void followPath(Path path, boolean reversed) {
        pathFollowingController = new AdaptivePurePursuitController(Constants.PATH_FOLLOWING_LOOKAHEAD,
                Constants.PATH_FOLLOWING_MAX_ACCELERATION, Constants.DRIVETRAIN_UPDATE_RATE, path, reversed, 1);
        mDriveControlState = DriveControlState.PATH_FOLLOWING;
        System.out.println("Now entering into path following mode");
        updatePathFollower();
    }

    private static double rotationsToInches(double rotations) {
        return rotations * Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 * 3.68 / 10.0;
    }

    private static double inchesPerSecondToRadiansPerSecond(double in_sec) {
        return in_sec / (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI) * 2 * Math.PI;
    }

    private static double rpmToTicksPer100ms(double rpm) {
        return ((rpm * 512.0) / 75.0);
    }

    /**
     * Arcade drive method for calculating drivetrain output.
     * <p>
     * defined as positive forward on both outputs and turning right yields positive
     * left output and negative right output
     * 
     * @param xSpeed    desired travel velocity
     * @param zRotation desired rotational velocity
     * @return a drivesignal for open loop use
     */
    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        final double maxInput = Math.max(Math.max(Math.abs(xSpeed - zRotation), Math.abs(xSpeed + zRotation)), 1);

        final double rightMotorOutput = (xSpeed + zRotation) / maxInput;
        final double leftMotorOutput = (xSpeed - zRotation) / maxInput;

        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }

}
