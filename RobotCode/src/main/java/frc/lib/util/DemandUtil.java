package frc.lib.util;

public class DemandUtil {
    /**
     * Arcade method for calculating drivetrain output.
     * <p>
     * defined as positive forward on both outputs and turning right yields positive
     * left output and negative right output
     *
     * @param xSpeed    desired travel velocity
     * @param zRotation desired rotational velocity
     * @return a drivesignal for open loop use
     */
    public static DriveSignal arcadeToSignal(double xSpeed, double zRotation) {
        final double maxInput = Math.max(Math.max(Math.abs(xSpeed - zRotation), Math.abs(xSpeed + zRotation)), 1);

        final double rightMotorOutput = (xSpeed + zRotation) / maxInput;
        final double leftMotorOutput = (xSpeed - zRotation) / maxInput;

        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }
}
