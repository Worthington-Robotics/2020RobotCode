package frc.robot.autoactiongroups;

import frc.lib.control.Path;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.superaction.TargetShootAction;
import frc.robot.actions.waitactions.WaitPointCloud;

import java.util.Arrays;
import java.util.List;

public class ThirtyEightSpecial extends StateMachineDescriptor {
    public ThirtyEightSpecial() {
        //getangle,checkpos,shoot
        //driveforwards,drivebackwards,get,check,shootagain

        addSequential(new TargetShootAction(), 2000);
        addSequential(new TargetShootAction(), 2000);
        addSequential(new TargetShootAction(), 2000);

        addParallel(new Action[] {
                new DriveTra(new Path(pathAsWaypoints()), false)//,
                //new WaitPointCloud(new Pose2d())
        }, 2);
    }

    private List<Path.Waypoint> pathAsWaypoints() {
        return Arrays.asList(
                new Path.Waypoint(new Translation2d(0,0), 0),
                new Path.Waypoint(new Translation2d(-91,-66), 0),
                new Path.Waypoint(new Translation2d(-259,-66), 0),
                new Path.Waypoint(new Translation2d(-193,-66), 0)
        );
    }
}
