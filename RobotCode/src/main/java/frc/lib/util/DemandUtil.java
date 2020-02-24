package frc.lib.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class DemandUtil {
    public static void setDemand(double demand, TalonSRX... motors) {
        for (TalonSRX motor : motors) {
            motor.set(ControlMode.PercentOutput, demand);
        }
    }

    public static void setFullDemand(TalonSRX... motors) {
        for (TalonSRX motor : motors) {
            motor.set(ControlMode.PercentOutput, Constants.FULL_BELT_DEMAND);
        }
    }

    public static void disable(TalonSRX... motors) {
        for (TalonSRX motor : motors) {
            motor.set(ControlMode.PercentOutput, Constants.STOP_DEMAND);
        }
    }

    public static void setFullBackDemand(TalonSRX... motors) {
        for (TalonSRX motor : motors) {
            motor.set(ControlMode.PercentOutput, -Constants.FULL_BELT_DEMAND);
        }
    }
}
