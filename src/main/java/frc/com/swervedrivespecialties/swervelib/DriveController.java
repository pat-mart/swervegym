package frc.com.swervedrivespecialties.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
