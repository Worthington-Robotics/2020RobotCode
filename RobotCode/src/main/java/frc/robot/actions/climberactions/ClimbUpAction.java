package frc.robot.actions.climberactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class ClimbUpAction extends Action {
    private boolean done;
    @Override
    public void onStart() {
        Climber.getInstance().setExtend(true);
        done = true;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void onStop() {
        
    }
}