package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public SwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;
  public Pigeon2 pigeon;

  public double[] velocity = new double[3];
  public double[] position = new double[3];
  // public double

  public Swerve() {
    pigeon = new Pigeon2(0);
    pigeon.configFactoryDefault();
    zeroGyro();

    swerveModules = new SwerveModule[] {
        new SwerveModule(Constants.s_frontLeft),
        new SwerveModule(Constants.s_frontRight),
        new SwerveModule(Constants.s_backLeft),
        new SwerveModule(Constants.s_backRight),
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getModulePositions());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));

    // normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
    }

    short[] xyz = new short[3];
    pigeon.getBiasedAccelerometer(xyz);

    for (int i = 0; i < 3; i++) {
      velocity[i] += xyz[i] / 16384d * 9.81d * 0.02d;
      position[i] += velocity[i] * 0.02;
    }

    SmartDashboard.putNumber("x acc", position[0]);
    SmartDashboard.putNumber("y acc", position[1]);
    SmartDashboard.putNumber("z acc", position[2]);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println("[Swerve::resetOdometry] Passed pose: " + pose.getX() + ", " + pose.getY());
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    pigeon.setYaw(0);
    pigeon.zeroGyroBiasNow();
  }

  private Rotation2d getYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public void resetIntegratedToAbsolute() {
    for (SwerveModule mod : swerveModules) {
      mod.resetAngleToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    SmartDashboard.putNumber("odometry-x", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry-y", swerveOdometry.getPoseMeters().getY());

    for (SwerveModule module : swerveModules) {
      module.debug();
    }
  }
}
