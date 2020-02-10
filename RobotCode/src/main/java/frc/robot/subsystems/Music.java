package frc.robot.subsystems;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.Constants;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Music extends Subsystem {

    private Orchestra orchestra;
    private MusicIO periodic;
    private TalonFX[] fxs = {
        new TalonFX(0),
        new TalonFX(1),
        new TalonFX(2),
        new TalonFX(13),
        new TalonFX(14),
        new TalonFX(15)
    };

    public Music() {
        ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
        for(int i = 0; i < fxs.length; i++) {
            _instruments.add(fxs[i]);
        }
        orchestra = new Orchestra(_instruments);
    }

    private static Music m_musicInstance = new Music();

    public static Music getInstance() {
        return m_musicInstance;
    }

    public String[] music = {
        "spanishFlea.chrp"
    };

    public void addMusic(String song) {
        orchestra.loadMusic(song);
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub

    }

    @Override
    public void writePeriodicOutputs() {
        if(periodic.isEnabled) {
            orchestra.play();
        }
        else {
            orchestra.stop();
        }
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

    public void setIsEnabled(boolean enabled) {
        periodic.isEnabled = enabled;
	}

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

    public class MusicIO extends PeriodicIO {
        public boolean isEnabled = false;
    }

	
    
}
