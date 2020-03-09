package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.climberactions.ClimbDownAction;
import frc.robot.actions.climberactions.ClimbUpAction;
import frc.robot.actions.climberactions.FoldAction;
import frc.robot.actions.climberactions.UnfoldAction;
import frc.robot.actions.colorwheelactions.LightsStateTest;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.driveactions.Shift;
import frc.robot.actions.shooteraction.Recenter;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.*;

public class SystemsCheck extends StateMachineDescriptor {
    public SystemsCheck() {
        addParallel(new Action[] {new DummyDrive(true), new IntakeAction(), new LightsStateTest(1)}, 5000);
        addParallel(new Action[] {new UnfoldAction(), new ClimbUpAction(), new DummyDrive(true), new Shift(), new LightsStateTest(2)}, 5000);
        addParallel(new Action[] {new ArmAction(), new LightsStateTest(3)}, 5000);
        addParallel(new Action[] {new ClimbDownAction(), new FoldAction(), new LightsStateTest(4)}, 5000);
        addParallel(new Action[] {new SetManualFlywheel(), new Recenter(0), new ArmActionUp(), new LightsStateTest(5)}, 5000);  
        addParallel(new Action[] {new ShootBallAction(), new LightsStateTest(6)}, 5000);
        addParallel(new Action[] {new LightsStateTest(7)}, 5000);
    }
}