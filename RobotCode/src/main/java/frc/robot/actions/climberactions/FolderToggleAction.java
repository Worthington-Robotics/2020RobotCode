package frc.robot.actions.climberactions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class FolderToggleAction extends Action {
    double timestamp;
    boolean done = false;

    @Override
    public void onStart() {
        System.out.println("Starting Action");
        timestamp = Timer.getFPGATimestamp();
        if (!Climber.getInstance().getClimbed()) {
            if (Shooter.getInstance().getShooterAngle() <= 90) {
                Shooter.getInstance().setTurretCenter(70);
            } else if (Shooter.getInstance().getShooterAngle() > 270) {
                Shooter.getInstance().setTurretCenter(-70);
            }
            if (Shooter.getInstance().canUnfold()) {
                Climber.getInstance().setUnfold(true);
                // System.out.println("Setting unfold to true");
            }
        }

    }

    @Override
    public void onLoop() {
        if (Shooter.getInstance().canUnfold()) {
            Shooter.getInstance().setTurretDemand(0);
        } else {
            if (timestamp + 1 > Timer.getFPGATimestamp()) {
                done = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;

    }

    @Override
    public void onStop() {
        if (!Climber.getInstance().getClimbed()) {
            Shooter.getInstance().setTurretCenter(75);
            if (Shooter.getInstance().canUnfold()) {
                Shooter.getInstance().setTurretDemand(0);
                Climber.getInstance().setUnfold(false);
            }
        }
    }
}