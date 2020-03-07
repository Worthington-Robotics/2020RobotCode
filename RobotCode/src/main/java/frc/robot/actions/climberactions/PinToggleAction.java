package frc.robot.actions.climberactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class PinToggleAction extends Action {
    @Override
    public void onStart() {
        Climber.getInstance().setPin(DoubleSolenoid.Value.kReverse);
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
        Climber.getInstance().setPin(DoubleSolenoid.Value.kForward);
    }
}