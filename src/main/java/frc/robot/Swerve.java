package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Swerve {
    private SwerveDriveKinematics kinematics;
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    public Swerve(Pigeon2 gyro) {
        
        this.gyro = gyro;
    }
}
