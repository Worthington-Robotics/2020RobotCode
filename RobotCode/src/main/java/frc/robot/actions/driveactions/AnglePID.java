package frc.robot.actions.driveactions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class AnglePID extends Action {

    public void onStart() {
        double angleOffset = SmartDashboard.getNumber("vision/angleOffset", -1000);

        if (angleOffset >= -180 && angleOffset <= 180) {
            double currentAngle = Drive.getInstance().getHeading().getDegrees();
            double desiredAngle = (angleOffset + currentAngle);
            if (desiredAngle > 180) {
                desiredAngle -= 360;
            } else if (desiredAngle < -180) {
                desiredAngle += 360;
            }
            SmartDashboard.putNumber("vision/Start Angle", currentAngle);
            SmartDashboard.putNumber("vision/Start Angle Offset", angleOffset);
            SmartDashboard.putNumber("vision/Desired Angle", desiredAngle);

            Drive.getInstance().setAnglePidLoop(DriveSignal.NEUTRAL, desiredAngle);
        }
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
            SmartDashboard.putBoolean("vision/On Target", Drive.getInstance().getPIDOnTarget());
            return Drive.getInstance().getPIDOnTarget();
    }

    @Override
    public void onStop() {
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }

}
