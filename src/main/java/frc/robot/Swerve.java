package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometer;
  private SwerveModule[] modules;
  private final Pigeon2 gyro;

  public Swerve(Pigeon2 gyro) {
    this.gyro = gyro;
    this.modules = new SwerveModule[4];
    Translation2d[] offsets = new Translation2d[4];

    for (int i = 0; i < this.modules.length; i++) {
      Util.SwerveModuleConfig config = Constants.SwerveConfig.modules[i];
      SwerveModule module = new SwerveModule(config);

      this.modules[module.moduleNumber()] = module;
      offsets[module.moduleNumber()] = new Translation2d(config.xOffset, config.yOffset);
    }

    this.kinematics = new SwerveDriveKinematics(offsets);
    this.odometer = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getYaw()), modulePositions());

    // pause initialization to give phoenix server time to boot
    // - prevents dropped can frames
    Timer.delay(1.0);

  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println("[Swerve::resetOdometry] Passed pose: " + pose.getX() + ", " + pose.getY());
    odometer.resetPosition(Rotation2d.fromDegrees(getYaw()), modulePositions(), pose);
  }

  public SwerveModule[] modules() {
    return this.modules;
  }

  public SwerveModuleState[] moduleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : this.modules) {
      states[module.moduleNumber()] = module.state();
    }
    return states;
  }

  public double getYaw() {
    return this.gyro.getYaw();
  }

  public SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule module : modules) {
      positions[module.moduleNumber()] = module.position();
    }
    return positions;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (SwerveModule module : modules) {
      module.setState(states[module.moduleNumber()]);
    }
  }

  public void resetModuleAnglesToAbsolute() {
    for (SwerveModule module : modules) {
      module.resetAngleToAbsolute();
    }
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] states;

    if (fieldRelative) {
      states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          Rotation2d.fromDegrees(getYaw())));
    } else {
      states = kinematics.toSwerveModuleStates(new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation));
    }

    // normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConfig.maxSpeed);

    for (SwerveModule module : this.modules) {
      module.setState(states[module.moduleNumber()]);
    }
  }

  @Override
  public void periodic() {
    odometer.update(Rotation2d.fromDegrees(getYaw()), modulePositions());
  }
}
