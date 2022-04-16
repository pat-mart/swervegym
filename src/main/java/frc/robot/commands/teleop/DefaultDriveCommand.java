// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class DefaultDriveCommand extends CommandBase {

  private final double k_rateLimiterX = 3.0;
  private final double k_rateLimiterY = 3.5;

  private final SlewRateLimiter limiterX = new SlewRateLimiter(k_rateLimiterX);
  private final SlewRateLimiter limiterY = new SlewRateLimiter(k_rateLimiterY);
  
  private final Drivetrain drivetrain;
  
  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier twist;

  /**
   * 
   * @param x Strafing left/right movement
   * @param y Forwards/backwards movement
   * @param twist Rotational movement 
   * @param drivetrain
   */
  public DefaultDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, Drivetrain drivetrain) {
    this.x = x;
    this.y = y;
    this.twist = twist;

    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        limiterX.calculate(x.getAsDouble()), 
        limiterY.calculate(y.getAsDouble()), 
        twist.getAsDouble(), 
        drivetrain.getGyroRotation() //FIXME add toggleable wheel-centric rotation
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
