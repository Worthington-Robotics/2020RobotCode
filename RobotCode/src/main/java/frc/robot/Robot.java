/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Looper;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.DriveSignal;
import frc.lib.util.VersionData;
import frc.robot.subsystems.*;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.colorwheelactions.*;
import frc.robot.actions.climberactions.*;
import frc.robot.actions.shooteraction.*;
import frc.robot.actions.superaction.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SubsystemManager manager = new SubsystemManager(Arrays.asList(
            // register subsystems here
            PoseEstimator.getInstance(), 
            Drive.getInstance(), 
            ColorWheel.getInstance(), 
            Climber.getInstance(),
            Superstructure.getInstance(),
            Shooter.getInstance()), true);;
    private Looper enabledLooper, disabledLooper;

    private JoystickButton gyroLock = new JoystickButton(Constants.MASTER, 1);
    private JoystickButton shift = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton deliveryWheel = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton delivery = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton indexer = new JoystickButton(Constants.MASTER, 5);
    private JoystickButton intake = new JoystickButton(Constants.MASTER, 6);
    private JoystickButton indexerOut = new JoystickButton(Constants.MASTER, 8);
    private JoystickButton foldClimb = new JoystickButton(Constants.MASTER, 9);
    private JoystickButton unfoldClimb = new JoystickButton(Constants.MASTER, 10);
    private JoystickButton climbDown = new JoystickButton(Constants.MASTER, 11);
    private JoystickButton climbUp = new JoystickButton(Constants.MASTER, 12);

    private JoystickButton inverse = new JoystickButton(Constants.SECOND, 2);
    private JoystickButton colorWheelManual = new JoystickButton(Constants.SECOND, 3);
    private JoystickButton colorWheelManualCCW = new JoystickButton(Constants.SECOND, 4);
    private JoystickButton releaseIntake = new JoystickButton(Constants.SECOND, 5);
    private JoystickButton shootOne = new JoystickButton(Constants.SECOND, 6);
    private JoystickButton turretControl = new JoystickButton(Constants.SECOND, 7);
    private JoystickButton flyWheelPID = new JoystickButton(Constants.SECOND, 9);
    private JoystickButton manualFlyWheel = new JoystickButton(Constants.SECOND, 11);
    private JoystickButton turretPIDControl = new JoystickButton(Constants.SECOND, 12);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // create the master looper threads
        enabledLooper = new Looper();
        disabledLooper = new Looper();

        // register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        // add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(StateMachine.getInstance()));

        // publish the auto list to the dashboard "Auto Selector"
        SmartDashboard.putStringArray("Auto List", AutoSelector.buildArray());

        // create buttons and register actions
        turretPIDControl.whenPressed(Action.toCommand(new TurretPIDControl(false)));
        manualFlyWheel.whenPressed(Action.toCommand(new SetManualFlywheel()));
        flyWheelPID.whenPressed(Action.toCommand(new SetFlywheelPID(false)));
        turretControl.whenPressed(Action.toCommand(new ManualTurretControl()));
        colorWheelManual.whileHeld(Action.toCommand(new colorWheelManual(false)));
        colorWheelManualCCW.whileHeld(Action.toCommand(new colorWheelManual(true)));
        foldClimb.whenPressed(Action.toCommand(new FoldAction()));
        unfoldClimb.whenPressed(Action.toCommand(new UnfoldAction()));
        climbDown.whenPressed(Action.toCommand(new ClimbDownAction()));
        climbUp.whenPressed(Action.toCommand(new ClimbUpAction()));
        inverse.whileHeld(Action.toCommand(new Inverse()));
        shift.whileHeld(Action.toCommand(new Shift()));
        gyroLock.whileHeld(Action.toCommand(new GyroLock()));
        shootOne.whenPressed(Action.toCommand(new ShootAction()));
        deliveryWheel.whileHeld(Action.toCommand(new DeliveryWheelAction()));
        indexerOut.whileHeld(Action.toCommand(new IndexBeltAction(true)));
        delivery.whileHeld(Action.toCommand(new DeliveryBeltAction()));
        indexer.whileHeld(Action.toCommand(new IndexBeltAction(false)));
        intake.whileHeld(Action.toCommand(new IntakeAction()));
        releaseIntake.toggleWhenPressed(Action.toCommand(new ArmAction()));
        VersionData.WriteBuildInfoToDashboard();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        manager.outputTelemetry();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();

        //Run any reset code here
        StateMachine.getInstance().assertStop();

        disabledLooper.start();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();

        String[] autoList = AutoSelector.buildArray();

        //pulls auto selector from labview DB
        String autoSelected = SmartDashboard.getString("Auto Selector", autoList[autoList.length - 1]);

        //schedule the state machine to run the selected autonomous
        StateMachine.getInstance().runMachine(AutoSelector.autoSelect(autoSelected));
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }
}