package frc.robot.actions.superaction;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class IntakeAction extends Action {
    private Superstructure superstructure;

    public IntakeAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        if (superstructure.getArmExtension() != DoubleSolenoid.Value.kOff) {
            superstructure.setIntakeDemand(Constants.HIGH_BELT_DEMAND);
        }
    }

    @Override public void onLoop() {
    }

    @Override public boolean isFinished() {
        return Constants.DISTANCE_STOP_MM >= superstructure.getIntakeDistance();
    }

    @Override public void onStop() {
        superstructure.setIntakeDemand(Constants.STOP_BELT_DEMAND);
    }
}
