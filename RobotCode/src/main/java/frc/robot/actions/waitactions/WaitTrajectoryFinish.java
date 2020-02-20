package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class WaitTrajectoryFinish extends Action {
    @Override public void onStart() {}

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return Drive.getInstance().isDoneWithTrajectory();
    }

    @Override public void onStop() {}
}
