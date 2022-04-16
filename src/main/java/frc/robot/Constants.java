// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import frc.com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Placeholder channel and geometry values for swerve drive related constants. */
    public static final class DriveConstants {
        //FIXME these values 
        public static final int k_gearDriveRatio = (int) SdsModuleConfigurations.MK4I_L1.getDriveReduction();
        public static final int k_gearTurnRaatio = (int) SdsModuleConfigurations.MK4I_L1.getSteerReduction();

        public static final int frontLeftDrive = 0;
        public static final int frontLeftTurn = 1;
        public static final int frontLeftEncoder = 8;

        public static final int backLeftDrive = 2;
        public static final int backLeftTurn = 3;
        public static final int backLeftEncoder = 9;

        public static final int frontRightDrive = 4;
        public static final int frontRightTurn = 5;
        public static final int frontRightEncoder = 10;

        public static final int backRightDrive = 6;
        public static final int backRightTurn = 7;
        public static final int backRightEncoder = 11;

        public static final double k_width = 0.5; //FIXME
        public static final double k_wheelBase = 0.5; //FIXME
        public static final double k_maxVoltage = 12.0; //FIXME

        public static final double k_maxVelocityMetersPerSecond = 6380.0 / 60 
            * SdsModuleConfigurations.MK4I_L1.getDriveReduction() 
            * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
            
        public static final double k_maxAngularVelocityPerSecond = k_maxVelocityMetersPerSecond /
            Math.hypot(k_width/2, k_wheelBase/2);
    }
}
