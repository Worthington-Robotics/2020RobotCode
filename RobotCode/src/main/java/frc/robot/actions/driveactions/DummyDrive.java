package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class DummyDrive extends Action {

    @Override
    public void onStart() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0.25, 0.25));
    }

    @Override
    public void onLoop() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0.15, 0.15));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }
}
