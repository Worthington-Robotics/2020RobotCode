package frc.robot.actions.driveactions;

import frc.lib.control.Path;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class DriveTra extends Action {
    private Path mPath;
    private boolean mReversed;
    public DriveTra(Path path, boolean reversed) {
        mPath = path;
        mReversed = reversed;
    }
    @Override
    public void onStart() {
        Drive.getInstance().followPath(mPath, mReversed);
    }

    @Override public void onLoop() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {}

    
}
