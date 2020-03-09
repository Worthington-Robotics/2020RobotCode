package frc.robot.autoactiongroups;

import java.util.Arrays;
import java.util.List;

import frc.lib.control.Path;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.FollowPath;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;

public class SixBallEasy extends StateMachineDescriptor {
    public SixBallEasy() {
        addParallel(new Action[] {new FollowPath(new Path(pathAsWaypoints())), new IntakeAction(), new TurretPIDControl()}, 15000);
    }

    private List<Path.Waypoint> pathAsWaypoints() {
        return Arrays.asList(
            new Path.Waypoint(new Translation2d(0,0), 5),
            new Path.Waypoint(new Translation2d(-120, 0), 5)
        );
    }
}