/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Looper;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.*;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    private SubsystemManager Manager = new SubsystemManager(Arrays.asList(
            Drive.getInstance(),
            PoseEstimator.getInstance()
    ), false);
    private Looper EnabledLoops = new Looper();
    private Looper DisabledLoops = new Looper();
    private OI Oi = new OI();

    @Override
    public void robotInit() {
        Manager.registerEnabledLoops(EnabledLoops);
        Manager.registerDisabledLoops(DisabledLoops);

    }

    public void robotPeriodic() {
        Manager.outputTelemetry();
    }

    @Override
    public void disabledInit() {
        // publishes the auto list to the dashboard "Auto Selector"
        SmartDashboard.putStringArray("Auto List", AutoSelector.buildArray());
        StateMachine.getInstance().assertStop();
        Drive.getInstance().overrideTrajectory(true);
        //Stop the disabled looper
        DisabledLoops.stop();
        //Start the enabled looper
        EnabledLoops.start();
    }

    @Override
    public void autonomousInit() {
        //Stop the disabled looper
        DisabledLoops.stop();
        //Reset all important subsystems
        PoseEstimator.getInstance().reset();
        Drive.getInstance().reset();

        //Start the enabled looper
        EnabledLoops.start();

        //pulls auto selector from labview DB
        final String[] autoList = AutoSelector.buildArray();
        //Default to last entry if Dashboard not found
        final String autoSelected = SmartDashboard.getString("Auto Selector", autoList[autoList.length - 1]);
        //get selected auto as a state machine descriptor
        final StateMachineDescriptor auto = AutoSelector.autoSelect(autoSelected);
        //perform a null check on the auto to see if it is valid
        if (auto != null) StateMachine.getInstance().runMachine(auto);
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        //Stop the disabled looper
        DisabledLoops.stop();

        //TODO INVESTIGATE but likely a bad thing to do
        //May want to remove these given that the robot was likely already running. may cause issues on field mode switch
        //PoseEstimator.getInstance().reset();
        //Drive.getInstance().reset();

        //Start the enabled looper
        EnabledLoops.start();

        //for saftey reasons switch drivetrain into open loop forcibly
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        //Stop the disabled looper
        DisabledLoops.stop();

        //May want to remove these given that the robot was likely already running. may cause issues on field mode switch
        PoseEstimator.getInstance().reset();
        Drive.getInstance().reset();

        //Start the enabled looper
        EnabledLoops.start();

    }

    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

}
