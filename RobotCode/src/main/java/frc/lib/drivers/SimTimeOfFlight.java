package frc.lib.drivers;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
 
public class SimTimeOfFlight implements AutoCloseable{

    private TimeOfFlight tof;
    private SimDevice sim;
    private SimDouble simDist, simSigma, simAmbient;
    private SimEnum sensorStatus;
    private SimBoolean readingValid;

    public SimTimeOfFlight(int sensorId){
        sim = SimDevice.create("TOF sensor", sensorId);
        if(sim == null){
            tof = new TimeOfFlight(sensorId);
        } else {
            simDist = sim.createDouble("Distance", false, 0.0);
            simSigma = sim.createDouble("Range Sigma", false, 0.0);
            simAmbient = sim.createDouble("Ambient Light", false, 0.0);
            String[] status = new String[Status.values().length];
            for(int i = 0; i < status.length; i++){
                status[i] = Status.values()[i].toString();
            }
            sensorStatus = sim.createEnum("Status", false, status, 0);
            readingValid = sim.createBoolean("Reading valid", false, true);
        }
    }

    public double getRange(){
        if(sim == null){
            return tof.getRange();
        } else {
            return simDist.get();
        }
    }

    public void setRangingMode(RangingMode mode, double sampleTime){
        if(sim == null){
            tof.setRangingMode(mode, sampleTime);
        }
    } 

    public Status getStatus(){
        if(sim == null){
            return tof.getStatus();
        }
        return Status.values()[sensorStatus.get()];
        
    }

    public double getAmbientLight(){
        if(sim == null){
            return tof.getAmbientLightLevel();
        }
        return simAmbient.get();
        
    }

    public double getRangeSigma(){
        if(sim == null){
            return tof.getRangeSigma();
        }
        return simSigma.get();
    }

    public boolean isRangeValid(){
        if(sim == null){
            return tof.isRangeValid();
        }
        return readingValid.get();
    }

    @Override
	public void close() throws Exception {
		if(sim == null){
            tof.close();
        }
	}

}