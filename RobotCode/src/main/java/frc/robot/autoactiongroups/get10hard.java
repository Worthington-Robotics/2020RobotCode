package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootBallAction;

public class get10hard extends StateMachineDescriptor {
    public get10hard() {
        addSequential(new DummyDrive(false), 250);
        addParallel(new Action[] {new DummyDrive(true), new TurretPIDControl()}, 2500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl(), new DummyDrive(true)}, 9500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl()}, 3000);
    }
}