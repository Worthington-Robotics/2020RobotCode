package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;

public class get10hard extends StateMachineDescriptor {
    public get10hard() {
        addSequential(new DummyDrive(false), 250);
        addParallel(new Action[] {new DummyDrive(true), new TurretPIDControl(), new IntakeAction()}, 2500);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl(), new DummyDrive(true), new IntakeAction()}, 9500);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl(), new IntakeAction()}, 3000);
    }
}