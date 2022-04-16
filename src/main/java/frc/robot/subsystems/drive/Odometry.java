package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import frc.com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import frc.robot.Constants.DriveConstants;


public abstract class Odometry {

    public static final int k_gearDriveRatio = DriveConstants.k_gearDriveRatio;

    public static int k_resolution = 2048;

    public static int k_wheelRadius = 2;


    /** 
     * @param meters Value in meters to be converted to encoder counts. 
     * 
     * @return Meter distance to encoder counts. 
    */
    public static int distancetoEncoderCount(double meters){

        double wheelRotations = meters/(2 * Math.PI * Units.inchesToMeters(k_wheelRadius));

        double motorRotations = wheelRotations * k_gearDriveRatio;

        int sensorCounts = (int) motorRotations * k_resolution;

        return sensorCounts;
    }

    /** 
     * @param velocity Velocity in meters per second
     * 
     * @return Encoder counts per 100 milliseconds
     */
    public static int velocityToTicksPer100ms(double velocity){

        double wheelRotPerSecond = velocity/(2 * Math.PI * Units.inchesToMeters(k_wheelRadius));

        double motorRotPerSecond = wheelRotPerSecond * k_gearDriveRatio;

        double motorRotPer100ms = motorRotPerSecond / 10;

        int ticksPer100ms = (int) motorRotPer100ms * k_resolution;

        return ticksPer100ms;
    }

    /**
     * @param ticksPer100ms Velocity in ticks per 100 ms 
     * 
     * @return The velocity in m/s
     */
    public static double ticksPer100msToVelocity(double ticksPer100ms){

        return (int) Math.pow((double) velocityToTicksPer100ms(ticksPer100ms), -1);

    }

    /**
     * @param encoderCount Distance in encoder counts
     * 
     * @return Distance in meters 
     */
    public static double encoderCountToDistance(double encoderCount){

        double motorRotations = (double) encoderCount / 2048; 
        
        double wheelRotations = motorRotations / k_gearDriveRatio; 
        
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2)); 
        
        return positionMeters;

    }

    public static void configWheelRadius(int radius){
        k_wheelRadius = radius;
    }
    /**
     * 
     */
    // public static int getAverageEncoderCount(WPI_TalonFX left, WPI_TalonFX right){

    // }
}
