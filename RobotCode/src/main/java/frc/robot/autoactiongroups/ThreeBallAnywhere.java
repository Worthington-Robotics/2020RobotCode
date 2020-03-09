package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootBallAction;

public class ThreeBallAnywhere extends StateMachineDescriptor {
    public ThreeBallAnywhere() {
        addParallel(new Action[] {new DummyDrive(true), new TurretPIDControl()}, 2000);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl()}, 4000);
    }
}