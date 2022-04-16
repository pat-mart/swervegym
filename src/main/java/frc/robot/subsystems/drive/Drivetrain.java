// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import frc.com.swervedrivespecialties.swervelib.SwerveModule;
import frc.com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private ChassisSpeeds chassisSpeeds;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //Front Left
    new Translation2d(DriveConstants.k_width/2, Constants.DriveConstants.k_wheelBase/2),
    //Front Right
    new Translation2d(DriveConstants.k_width/2, -DriveConstants.k_wheelBase/2),
    //Back Left
    new Translation2d(-DriveConstants.k_width/2, DriveConstants.k_wheelBase/2),
    //Back Right
    new Translation2d(-DriveConstants.k_width/2, -DriveConstants.k_wheelBase/2)

  );
  
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getNegativeGyroRotation());

  private Pose2d pose;

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
    Mk4iSwerveModuleHelper.GearRatio.L1, 
    DriveConstants.frontLeftDrive, 
    DriveConstants.frontLeftTurn, 
    DriveConstants.frontLeftEncoder, 
    0 //FIXME change steer offsets 
  );

  private final SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
    Mk4iSwerveModuleHelper.GearRatio.L1, 
    DriveConstants.frontRightDrive, 
    DriveConstants.frontRightTurn, 
    DriveConstants.frontRightEncoder, 
    0 //FIXME change steer offsets 
  );

  private final SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
    Mk4iSwerveModuleHelper.GearRatio.L1, 
    DriveConstants.backLeftDrive, 
    DriveConstants.backLeftTurn, 
    DriveConstants.backLeftEncoder, 
    0 //FIXME change steer offsets (make sure is in radians)
  );

  private final SwerveModule backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
    Mk4iSwerveModuleHelper.GearRatio.L1, 
    DriveConstants.backRightDrive, 
    DriveConstants.backRightTurn, 
    DriveConstants.backRightEncoder, 
    0 //FIXME change steer offsets (make sure is in radians)
  );

  private SwerveModule[] modules = new SwerveModule[4];

  public Drivetrain() {
    modules[0] = frontLeftModule;
    modules[1] = frontRightModule;
    modules[2] = backLeftModule;
    modules[3] = backRightModule;

    pose = new Pose2d();
  }

  public void zeroGyroscope(){
    gyro.reset();
  }

  public Rotation2d getGyroRotation(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }
  
  public Rotation2d getNegativeGyroRotation(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getPose(){
    return pose;
  }

  /**
   * @param speeds X speed, Y speed, twist speed, gyro angle in a ChassisSpeeds instance 
   */
  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.k_maxVelocityMetersPerSecond);

    modules[0].set(states[0].speedMetersPerSecond / DriveConstants.k_maxVelocityMetersPerSecond * DriveConstants.k_maxVoltage, states[0].angle.getRadians());
    modules[1].set(states[1].speedMetersPerSecond / DriveConstants.k_maxVelocityMetersPerSecond * DriveConstants.k_maxVoltage, states[1].angle.getRadians());
    modules[2].set(states[2].speedMetersPerSecond / DriveConstants.k_maxVelocityMetersPerSecond * DriveConstants.k_maxVoltage, states[2].angle.getRadians());
    modules[3].set(states[3].speedMetersPerSecond / DriveConstants.k_maxVelocityMetersPerSecond * DriveConstants.k_maxVoltage, states[3].angle.getRadians());

    var gyroAngle = getNegativeGyroRotation();
    pose = odometry.update(gyroAngle, states[0], states[1], states[2], states[3]);
  }

}