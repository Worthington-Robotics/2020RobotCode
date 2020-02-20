package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.PoseEstimator;

public class WaitLineCross extends Action {
    private double lineValue;
    private Axis axis;

    private boolean pastLine;
    private boolean backwardsCross;

    public WaitLineCross(double lineValue, Axis axis, boolean backwardsCross) {
        this.lineValue = lineValue;
        this.axis = axis;

        pastLine = false;
        this.backwardsCross = backwardsCross;
    }

    @Override public void onStart() {}

    @Override public void onLoop() {
        double threshold;

        switch (axis) {
            case X:
                threshold = getCurrentX();
                break;
            case Y:
                threshold = getCurrentY();
                break;
            default:
                throw new IllegalStateException();
        }

        if (backwardsCross ? lineValue < threshold : lineValue > threshold) {
            pastLine = true;
        }
    }

    @Override public boolean isFinished() {
        return pastLine;
    }

    @Override public void onStop() {}

    private double getCurrentX() {
        return PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
    }

    private double getCurrentY() {
        return PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
    }

    /**
     * Determines which axis line values are for. Probably should go in a separate class.
     */
    public enum Axis {
        X, Y
    }
}
