package frc.robot.autoactiongroups;

import java.util.ArrayList;

import frc.lib.control.Path;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.FollowPath;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;

public class ElevenTrenchRun extends StateMachineDescriptor {
    public ElevenTrenchRun() {
        ArrayList<Path.Waypoint> first = new ArrayList<>();
        first.add(new Path.Waypoint(new Translation2d(0,0), 48));
        first.add(new Path.Waypoint(new Translation2d(-111, 0), 48));
        first.add(new Path.Waypoint(new Translation2d(0, -79), 48));
        first.add(new Path.Waypoint(new Translation2d(-208, -79), 48));
        ArrayList<Path.Waypoint> last = new ArrayList<>();
        last.add(new Path.Waypoint(new Translation2d(-208, -79), 48));
        last.add(new Path.Waypoint(new Translation2d(-247, -79), 48));
        last.add(new Path.Waypoint(new Translation2d(-211, -79), 48));
        addSequential(new TurretPIDControl(), 2000);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl(), new IntakeAction(), new FollowPath(new Path(first))}, 8000);
        addParallel(new Action[] {new IntakeAction(), new FollowPath(new Path(last))}, 2000);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl()}, 3000);
    }
}