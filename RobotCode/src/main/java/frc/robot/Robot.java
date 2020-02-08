/*----------------------------------------------------------------------------*/
/* Copyright (c) 1992-1993 FIRST. All Rights Reserved.                        */
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
import frc.robot.actions.superaction.DeliveryBeltAction;
import frc.robot.actions.superaction.IndexBeltAction;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAction;
import frc.robot.actions.superaction.ShootType;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Superstructure;
import frc.lib.util.DriveSignal;
import frc.robot.actions.driveactions.GyroLock;
import frc.robot.actions.driveactions.Shift;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PoseEstimator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SubsystemManager manager  = new SubsystemManager(Arrays.asList(
        //register subsystems here
        Lights.getInstance(),
        Superstructure.getInstance(),
        PoseEstimator.getInstance(),
        Drive.getInstance(),
        ColorWheel.getInstance()
    ), true);
    private Looper enabledLooper, disabledLooper;
    
    private JoystickButton shootAll = new JoystickButton(Constants.MASTER, 1);
    private JoystickButton shootOne = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton delivery = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton indexer = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton intake = new JoystickButton(Constants.MASTER, 5);
    private JoystickButton shift = new JoystickButton(Constants.MASTER, 6);
    private JoystickButton gyroLock = new JoystickButton(Constants.MASTER, 8);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit(){
        //create the master looper threads
        enabledLooper = new Looper();
        disabledLooper = new Looper();

        //register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        //add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(
            StateMachine.getInstance()
        ));

        // publish the auto list to the dashboard "Auto Selector"
        SmartDashboard.putStringArray("Auto List", AutoSelector.buildArray()); 

        //create buttons and register actions
        shift.whileHeld(Action.toCommand(new Shift()));
        gyroLock.whileHeld(Action.toCommand(new GyroLock()));

        shootAll.whenPressed(Action.toCommand(new ShootAction(ShootType.ALL)));
        shootOne.whenPressed(Action.toCommand(new ShootAction(ShootType.ONE)));
        delivery.whileHeld(Action.toCommand(new DeliveryBeltAction()));
        indexer.whileHeld(Action.toCommand(new IndexBeltAction()));
        intake.whileHeld(Action.toCommand(new IntakeAction()));
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