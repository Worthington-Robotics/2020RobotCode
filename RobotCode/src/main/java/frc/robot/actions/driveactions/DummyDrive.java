package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class DummyDrive extends Action {

    private boolean forward = true;

    public DummyDrive(boolean forward) {
        this.forward = forward;
    }

    @Override
    public void onStart() {
        if (forward) {
            Drive.getInstance().setOpenLoop(new DriveSignal(0.25, 0.25));
        } else {
            Drive.getInstance().setOpenLoop(new DriveSignal(-0.25, -0.25));
        }
    }

    @Override
    public void onLoop() {
        if (forward) {
            Drive.getInstance().setOpenLoop(new DriveSignal(0.5, 0.5));
        } else {
            Drive.getInstance().setOpenLoop(new DriveSignal(-0.5, -0.5));
        }
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
