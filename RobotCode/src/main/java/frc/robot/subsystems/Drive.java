package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Drive extends Subsystem {

    //construct one and only 1 instance of this class
    private static Drive m_DriveInstance = new Drive();

    public static Drive getInstance() {
        return m_DriveInstance;
    }

    //used internally for data
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private boolean mOverrideTrajectory = false;
    private PeriodicIO periodic;
    private double[] operatorInput = {0, 0, 0}; //last input set from joystick update
    private PigeonIMU pigeonIMU;
    private DoubleSolenoid trans;
    private TalonSRX driveFrontLeft, driveBackRight, driveFrontRight;
    private VictorSPX driveMiddleLeft, driveMiddleRight, driveBackLeft;

    private final Loop mLoop = new Loop() {

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

                switch (mDriveControlState) {
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    case PROFILING_TEST:
                        if (Constants.RAMPUP) {
                            periodic.left_demand = -(periodic.ramp_Up_Counter * .0025 + .01);
                            periodic.right_demand = -(periodic.ramp_Up_Counter * .0025 + .01);
                            periodic.ramp_Up_Counter++;
                        } else if (DriverStation.getInstance().isTest()) {
                            periodic.left_demand = radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                            periodic.right_demand = radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                        }

                        break;
//TODO Change z input to PID
                    case GYRO_LOCK:
                        operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        operatorInput[2] = (periodic.gyro_heading.getDegrees() - periodic.gyro_pid_angle) / 720;
                        setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;

                    case OPEN_LOOP:
                        operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
                        if (Constants.MASTER.getRawButton(9)) {
                            operatorInput[1] = operatorInput[1] / 7 * 10;
                        }
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;
                    case ANGLE_PID:
                        // Let the AnglePIDOutput set the values
                        break;
                    default:
                        System.out.println("You fool, unexpected control state");
                }


            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public synchronized void readPeriodicInputs() {
        if (periodic.flaggyflag) {
            periodic.B1 = false;
        } else {
            periodic.B1 = Constants.MASTER.getRawButton(1);
        }
        double prevLeftTicks = periodic.left_pos_ticks;
        double prevRightTicks = periodic.right_pos_ticks;
        periodic.left_error = driveFrontLeft.getClosedLoopError();
        periodic.right_error = driveFrontRight.getClosedLoopError();

        //periodic.B2 = Constants.MASTER.getRawButton(2);
        periodic.left_pos_ticks = -driveFrontLeft.getSelectedSensorPosition(0);
        periodic.right_pos_ticks = -driveFrontRight.getSelectedSensorPosition(0);
        periodic.left_velocity_ticks_per_100ms = -driveFrontLeft.getSelectedSensorVelocity(0);
        periodic.right_velocity_ticks_per_100ms = -driveFrontRight.getSelectedSensorVelocity(0);
        periodic.gyro_heading = Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).rotateBy(periodic.gyro_offset);

        double deltaLeftTicks = ((periodic.left_pos_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        periodic.left_distance += deltaLeftTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        double deltaRightTicks = ((periodic.right_pos_ticks - prevRightTicks) / 4096.0) * Math.PI;
        periodic.right_distance += deltaRightTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;

    }

    @Override
    public synchronized void writePeriodicOutputs() {
        /*if (periodic.B2) {
            periodic.left_demand = -periodic.left_demand;
            periodic.right_demand = -periodic.right_demand;
        } else {
        }*/
        if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveControlState == DriveControlState.ANGLE_PID || (mDriveControlState == DriveControlState.PROFILING_TEST && Constants.RAMPUP)) {
            driveFrontLeft.set(ControlMode.PercentOutput, periodic.left_demand);
            driveFrontRight.set(ControlMode.PercentOutput, periodic.right_demand);
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        } else {
            driveFrontLeft.set(ControlMode.Velocity, -periodic.left_demand, DemandType.ArbitraryFeedForward, -(periodic.left_feedforward + Constants.DRIVE_LEFT_KD * periodic.left_accl / 1023.0));
            driveFrontRight.set(ControlMode.Velocity, -periodic.right_demand, DemandType.ArbitraryFeedForward, -(periodic.right_feedforward + Constants.DRIVE_RIGHT_KD * periodic.right_accl / 1023.0));
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        }
        //gearShift();
        if (periodic.B1) {
            trans.set(DoubleSolenoid.Value.kReverse);
        } else {
            trans.set(DoubleSolenoid.Value.kForward);
        }
    }

    private Drive() {
        driveFrontLeft = new TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
        driveMiddleLeft = new VictorSPX(Constants.DRIVE_MIDDLE_LEFT_ID);
        driveBackLeft = new VictorSPX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveMiddleRight = new VictorSPX(Constants.DRIVE_MIDDLE_RIGHT_ID);
        driveBackRight = new TalonSRX(Constants.DRIVE_BACK_RIGHT_ID);
        pigeonIMU = new PigeonIMU(driveBackRight);
        trans = new DoubleSolenoid(Constants.TRANS_LOW_ID, Constants.TRANS_HIGH_ID);
        configTalons();
        reset();

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
        //pigeonIMU.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
        periodic = new PeriodicIO();
        setHeading(Rotation2d.fromDegrees(0));
        resetEncoders();

        // Set the camera selection to the front drive camera
        SmartDashboard.putString("CameraSelection", "Front");
    }

    private void resetEncoders() {
        driveFrontRight.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSelectedSensorPosition(0, 0, 0);
    }

    private void configTalons() {
        driveFrontLeft.setSensorPhase(true);
        driveFrontLeft.selectProfileSlot(0, 0);
        driveFrontLeft.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveFrontLeft.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveFrontLeft.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveFrontLeft.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveFrontLeft.config_IntegralZone(0, 300);
        driveFrontLeft.setInverted(false);
        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveFrontLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontLeft.enableVoltageCompensation(true);

        driveMiddleLeft.setInverted(false);
        driveMiddleLeft.setNeutralMode(NeutralMode.Brake);
        driveMiddleLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveMiddleLeft.enableVoltageCompensation(true);
        driveMiddleLeft.follow(driveFrontLeft);

        driveBackLeft.setInverted(false);
        driveBackLeft.setNeutralMode(NeutralMode.Brake);
        driveBackLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackLeft.enableVoltageCompensation(true);
        driveBackLeft.follow(driveFrontLeft);


        driveFrontRight.setSensorPhase(true);
        driveFrontRight.selectProfileSlot(0, 0);
        driveFrontRight.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveFrontRight.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveFrontRight.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveFrontRight.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveFrontRight.config_IntegralZone(0, 300);
        driveFrontRight.setInverted(true);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveFrontRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontRight.enableVoltageCompensation(true);

        driveMiddleRight.setInverted(true);
        driveMiddleRight.setNeutralMode(NeutralMode.Brake);
        driveMiddleRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveMiddleRight.enableVoltageCompensation(true);
        driveMiddleRight.follow(driveFrontRight);

        driveBackRight.setInverted(true);
        driveBackRight.setNeutralMode(NeutralMode.Brake);
        driveBackRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackRight.enableVoltageCompensation(true);
        driveBackRight.follow(driveFrontRight);

    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            
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

    public synchronized void setGyroLock(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.GYRO_LOCK) {
            System.out.println("Switching to open loop");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.GYRO_LOCK;
        }
        periodic.gyro_pid_angle = periodic.gyro_heading.getDegrees();
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
    }

    /**
     * Configure talons for Angle PID control
     */
    public synchronized void setAnglePidLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.ANGLE_PID) {
            System.out.println("Switching to angle control");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.ANGLE_PID;
        }
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
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
        periodic.left_feedforward = feedforward.getLeft();
        periodic.right_feedforward = feedforward.getRight();


    }


    public void setMotorPower(double MP) {
        periodic.climber_power = MP;
    }

    
    //TODO change tra to Pure Presuit
    public boolean isDoneWithTrajectory() {
        /*if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;*/
        return false;
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Drive/Drive State", mDriveControlState.toString());

        SmartDashboard.putNumber("Drive/Error/X", periodic.error.getTranslation().x());
        SmartDashboard.putNumber("Drive/Error/Y", periodic.error.getTranslation().y());
        SmartDashboard.putNumber("Drive/Error/Theta", periodic.error.getRotation().getDegrees());

        SmartDashboard.putNumber("Drive/Left/Demand", periodic.left_demand);
        SmartDashboard.putNumber("Drive/Left/Talon Velocity", periodic.left_velocity_ticks_per_100ms);
        SmartDashboard.putNumber("Drive/Left/Talon Error", periodic.left_error);
        SmartDashboard.putNumber("Drive/Left/Talon Voltage Out", driveFrontLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Left/Encoder Counts", periodic.left_pos_ticks);
        //SmartDashboard.putNumber("Drive/Misc/Left FeedForward", periodic.left_feedforward);
        //SmartDashboard.putNumber("Drive/Misc/Left Acceleration", periodic.left_accl);


        SmartDashboard.putNumber("Drive/Right/Demand", periodic.right_demand);
        SmartDashboard.putNumber("Drive/Right/Talon Velocity", periodic.right_velocity_ticks_per_100ms);
        SmartDashboard.putNumber("Drive/Right/Talon Error", periodic.right_error);
        SmartDashboard.putNumber("Drive/Right/Talon Voltage Out", driveFrontRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Right/Encoder Counts", periodic.right_pos_ticks);
        //SmartDashboard.putNumber("Drive/Misc/Right FeedForward", periodic.right_feedforward);
        //SmartDashboard.putNumber("Drive/Misc/Right Acceleration", periodic.right_accl);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,                                                              
        PROFILING_TEST,
        GYRO_LOCK,
        ANGLE_PID;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public static class PeriodicIO {
        // INPUTS
        int left_pos_ticks;
        int right_pos_ticks;
        int left_velocity_ticks_per_100ms;
        int right_velocity_ticks_per_100ms;
        Rotation2d gyro_heading = Rotation2d.identity();
        Rotation2d gyro_offset = Rotation2d.identity();
        Pose2d error = Pose2d.identity();
        boolean flaggyflag = false;
        boolean B1 = false;
        double right_error = 0;
        double left_error = 0;
        double gyro_pid_angle = 0;

        // OUTPUTS
        double ramp_Up_Counter = 0;
        double left_accl = 0.0;
        double left_demand = 0.0;
        double left_distance = 0.0;
        double left_feedforward = 0.0;

        double right_accl = 0.0;
        double right_demand = 0.0;
        double right_distance = 0.0;
        double right_feedforward = 0.0;

        double climber_power = 0;

        
    }

    /**
     * internal methods beyond this point
     **/

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
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    private static double inchesPerSecondToRadiansPerSecond(double in_sec) {
        return in_sec / (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI) * 2 * Math.PI;
    }

    private static double rpmToTicksPer100ms(double rpm) {
        return ((rpm * 512.0) / 75.0);
    }

    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }
}