package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootBallAction;
import frc.robot.actions.waitactions.LineCrossWait;

public class get10Impossible extends StateMachineDescriptor {
    public get10Impossible() {
        addSequential(new DummyDrive(false), 250);
        addParallel(new Action[] {new DummyDrive(true), new TurretPIDControl()}, 2500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl(), new DummyDrive(true), new LineCrossWait(108, false, false)}, 9500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl()}, 3000);
    }
}