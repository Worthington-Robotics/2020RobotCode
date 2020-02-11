package frc.robot.actions.superaction;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class Shoot extends Action {
    private Superstructure superstructure;

    public Shoot() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setShoot();
    }

    @Override public void onLoop() {
    }

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
    }
}
