package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Subsystem {

    private static Limelight m_limelightInstance = new Limelight();

    private static Limelight getInstance() {
        return m_limelightInstance;
    }

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    private double x = 0.0;
    private double y = 0.0;
    private double area = 0.0;

    @Override
    public synchronized void readPeriodicInputs() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vision/LimelightX", x);
        SmartDashboard.putNumber("Vision/LimelightY", y);
        SmartDashboard.putNumber("Vision/LimelightArea", area);
    }

    @Override
    public void reset() {

    }

}
