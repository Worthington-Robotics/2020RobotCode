package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class WaitForPathFinish extends Action {
    @Override public void onStart() {}

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return Drive.getInstance().isDoneWithTrajectory();
    }

    @Override public void onStop() {}
}
