package frc.robot.actions.driveactions;

import frc.lib.control.Path;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class DriveTra extends Action {
    private Path path;
    private boolean reversed;

    public DriveTra(Path path, boolean reversed) {
        this.path = path;
        this.reversed = reversed;
    }

    @Override
    public void onStart() {
        Drive.getInstance().followPath(path, reversed);
    }

    @Override public void onLoop() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {}
}
