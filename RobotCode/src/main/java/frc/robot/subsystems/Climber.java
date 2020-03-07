package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Util;
import frc.robot.Constants;

public class Climber extends Subsystem {
    private ClimberIO periodic;

    private DoubleSolenoid kickstand;
    private DoubleSolenoid pin;

    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private Climber() {
        kickstand = new DoubleSolenoid(Constants.ID_KICKSTAND_LOW, Constants.ID_KICKSTAND_HIGH);
        pin = new DoubleSolenoid(Constants.ID_PIN_LOW, Constants.ID_PIN_HIGH);

        leftMotor = new TalonFX(Constants.ID_CLIMBER_LEFT);
        rightMotor = new TalonFX(Constants.ID_CLIMBER_RIGHT);

        reset();
    }

    private static Climber instance = new Climber();
    public static Climber getInstance() {
        return instance;
    }

    @Override public void readPeriodicInputs() {}

    @Override public void writePeriodicOutputs() {
        kickstand.set(periodic.kickstandState);
        pin.set(periodic.pinState);

        leftMotor.set(ControlMode.PercentOutput, periodic.leftMotorDemand);
        rightMotor.set(TalonFXControlMode.PercentOutput, periodic.rightMotorDemand);
    }

    /*private double getTurretAngle(){
        return SmartDashboard.getNumber("turretsim", 0);
    }*/

    private boolean shooterReady(){
        return Util.epsilonEquals(Shooter.getInstance().getShooterAngle(), 270, 25) || // 270, 10
        Util.epsilonEquals(Shooter.getInstance().getShooterAngle(), 90, 25); // 90, 10
    }

    // Setters
    public void setKickstand(Value value) {
        periodic.kickstandState = value;
    }

    public void setPin(Value value) {
        periodic.pinState = value;
    }
    // TODO Implement gyro climb
    // TODO Implement DriveToggleAction
    public void setLeftMotorDemand(double leftMotorDemand) {
        periodic.leftMotorDemand = leftMotorDemand;
    }

    public void setRightMotorDemand(double rightMotorDemand) {
        periodic.rightMotorDemand = rightMotorDemand;
    }

    /**
     * Stores data for the ReflectingLogger to grab and dump.
     */
    public class ClimberIO extends Subsystem.PeriodicIO {
        public Value kickstandState = Value.kReverse;
        public Value pinState = Value.kForward;

        public double leftMotorDemand;
        public double rightMotorDemand;
    }
    public LogData getLogger() {
        return periodic;
    }

    @Override public void reset() {
        periodic = new ClimberIO();

        kickstand.set(Value.kReverse);
        pin.set(Value.kForward);
    }

    @Override public void outputTelemetry() {
//        SmartDashboard.putBoolean("Climb/Want Unfolded", wantUnfold);
//        SmartDashboard.putBoolean("Climb/Want Extended", wantClimb);
//        SmartDashboard.putBoolean("Climb/Unfolded", unfoldBoolean);
//        SmartDashboard.putBoolean("Climb/Climbed", climbBoolean);
        SmartDashboard.putBoolean("Climb/ShooterReady", shooterReady());
    }
}
