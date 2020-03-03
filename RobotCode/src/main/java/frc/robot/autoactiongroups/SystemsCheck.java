package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.climberactions.ClimbDownAction;
import frc.robot.actions.climberactions.ClimbUpAction;
import frc.robot.actions.climberactions.FoldAction;
import frc.robot.actions.climberactions.UnfoldAction;
import frc.robot.actions.colorwheelactions.LightsStateTest;
import frc.robot.actions.colorwheelactions.colorWheelManual;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.driveactions.Shift;
import frc.robot.actions.shooteraction.Recenter;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.shooteraction.TurretToStop;
import frc.robot.actions.superaction.ArmAction;
import frc.robot.actions.superaction.ArmActionUp;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAction;
import frc.robot.actions.superaction.SuperstructureDisable;

public class SystemsCheck extends StateMachineDescriptor {
    public SystemsCheck() {
        addParallel(new Action[] {new DummyDrive(), new TurretToStop(false), new IntakeAction(), new LightsStateTest(1)}, 5000);
        addParallel(new Action[] {new UnfoldAction(), new ClimbUpAction(), new DummyDrive(), new Shift(), new LightsStateTest(2)}, 5000);
        addParallel(new Action[] {new ArmAction(), new TurretToStop(true), new colorWheelManual(false), new LightsStateTest(3)}, 5000);
        addParallel(new Action[] {new ClimbDownAction(), new FoldAction(), new colorWheelManual(true), new LightsStateTest(4)}, 5000);
        addParallel(new Action[] {new SetManualFlywheel(), new Recenter(0), new ArmActionUp(), new LightsStateTest(5)}, 5000);
        addParallel(new Action[] {new ShootAction(), new LightsStateTest(6)}, 5000);
        addParallel(new Action[] {new SuperstructureDisable(), new LightsStateTest(7)}, 5000);
    }
}