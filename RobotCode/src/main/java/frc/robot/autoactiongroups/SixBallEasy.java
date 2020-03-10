package frc.robot.autoactiongroups;

import java.util.ArrayList;

import frc.lib.control.Path;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.FollowPath;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootAllAction;

public class SixBallEasy extends StateMachineDescriptor {
    public SixBallEasy() {
        ArrayList<Path.Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Path.Waypoint(new Translation2d(0, 0), 48));
        waypoints.add(new Path.Waypoint(new Translation2d(-120, 0), 48));
        addSequential(new TurretPIDControl(), 2000);
        addParallel(new Action[] { new FollowPath(new Path(waypoints)), new TurretPIDControl(), new ShootAllAction() },
                15000);
    }
}