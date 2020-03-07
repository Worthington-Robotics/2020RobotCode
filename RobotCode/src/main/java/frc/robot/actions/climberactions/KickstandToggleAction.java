package frc.robot.actions.climberactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;

public class KickstandToggleAction extends Action {

    @Override
    public void onStart() {
        Climber.getInstance().setKickstand(DoubleSolenoid.Value.kForward);
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
        Climber.getInstance().setKickstand(DoubleSolenoid.Value.kReverse);
    }
}