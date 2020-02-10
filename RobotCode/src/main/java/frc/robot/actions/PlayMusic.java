package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Music;

public class PlayMusic extends Action {

    @Override
    public void onStart() {
        Music.getInstance().addMusic(Music.getInstance().music[0]);
        Music.getInstance().setIsEnabled(true);
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
        Music.getInstance().setIsEnabled(false);

    }
    
}