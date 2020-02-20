package frc.robot.actions.waitactions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.statemachine.Action;
import frc.lib.util.Util;
import frc.robot.subsystems.PoseEstimator;

public class WaitPointCloud extends Action {
    private boolean xPassed;
    private boolean yPassed;
    private boolean thetaPassed;

    private double x;
    private double y;
    private double theta;

    private double epsilonX;
    private double epsilonY;
    private double epsilonTheta;

    public WaitPointCloud(Pose2d pose, double epsilonX, double epsilonY, double epsilonTheta) {
        x = pose.getTranslation().x();
        y = pose.getTranslation().y();
        theta = pose.getRotation().getDegrees();

        this.epsilonX = epsilonX;
        this.epsilonY = epsilonY;
        this.epsilonTheta = epsilonTheta;
    }

    @Override public void onStart() {
        xPassed = false;
        yPassed = false;
        thetaPassed = false;
    }

    public void onLoop() {
        double mX = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
        double mY = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        double mTheta = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        xPassed = Util.epsilonEquals(x, mX, epsilonX);
        yPassed = Util.epsilonEquals(y, mY, epsilonY);
        thetaPassed = Util.epsilonEquals(theta, mTheta, epsilonTheta);
        SmartDashboard.putBoolean("isX", xPassed);
        SmartDashboard.putBoolean("isY", yPassed);
        SmartDashboard.putBoolean("isTheta", thetaPassed);
    }

    public boolean isFinished() {
        return (xPassed && yPassed) && thetaPassed;
    }

    public void onStop() {

    }
}
