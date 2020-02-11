package frc.robot.actions.superaction;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class Dump extends Action {
    private Superstructure superstructure;

    public Dump() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setDump();
    }

    @Override public void onLoop() {
    }

    @Override public boolean isFinished() {
        return true;
    }

    @Override public void onStop() {
        superstructure.setInit();
    }
}
