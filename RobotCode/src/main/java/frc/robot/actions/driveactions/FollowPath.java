package frc.robot.actions.driveactions;

import frc.lib.control.Path;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class FollowPath extends Action {

    private Path path;

    public FollowPath(Path p) {
        path = p;
    }

    @Override
    public void onStart() {
        Drive.getInstance().followPath(path, false);

    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub

    }
    
}