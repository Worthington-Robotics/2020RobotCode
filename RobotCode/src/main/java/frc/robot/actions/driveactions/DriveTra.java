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

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {

    }

    
}
