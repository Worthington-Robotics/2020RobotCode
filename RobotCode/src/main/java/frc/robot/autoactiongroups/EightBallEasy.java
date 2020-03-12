package frc.robot.autoactiongroups;

import java.util.ArrayList;
import java.util.Arrays;

import frc.lib.control.Path;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.driveactions.FollowPath;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;

public class EightBallEasy extends StateMachineDescriptor {
    ArrayList<Path.Waypoint> waypointsBack = new ArrayList<>(Arrays
            .asList(new Path.Waypoint(new Translation2d(0, 0), 72), new Path.Waypoint(new Translation2d(228, 0), 72)));
    ArrayList<Path.Waypoint> waypointsForward = new ArrayList<>(Arrays.asList(
            new Path.Waypoint(new Translation2d(228, 0), 72), new Path.Waypoint(new Translation2d(180, 0), 72)));

    public EightBallEasy() {
        addParallel(new Action[] { new TurretPIDControl(), new FollowPath(new Path(waypointsBack)) }, 1500);
        addParallel(new Action[] { new TurretPIDControl(), new ShootAllAction(), new IntakeAction() }, 5000);
        addParallel(new Action[] { new TurretPIDControl(), new IntakeAction() }, 2000);
        addParallel(new Action[] { new DummyDrive(false), new TurretPIDControl() }, 2500);
        addParallel(new Action[] { new TurretPIDControl(), new ShootAllAction() }, 2000);
    }
}