package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class Inverse extends Action {

    @Override
    public void onStart() {
        Drive.getInstance().setInverse(true);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Drive.getInstance().setInverse(false);
    }
}
