package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.vision.LimelightWrapper;

public class Swerve extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public SwerveModule[] swerveModules;
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private LimelightWrapper limelight;
  private Pigeon2 pigeon;

  public Swerve(Pigeon2 pigeon, LimelightWrapper limelight) {
    this.pigeon = pigeon;
    this.limelight = limelight;

    this.swerveModules = new SwerveModule[] {
        new SwerveModule(Constants.s_frontLeft),
        new SwerveModule(Constants.s_backLeft),
        new SwerveModule(Constants.s_backRight),
        new SwerveModule(Constants.s_frontRight),
    };

    // Pause initialization for one second, to wait for the pheonix server to start
    // up. Prevents CAN frames from being dropped on init.
    Timer.delay(1.0);
    resetIntegratedToAbsolute();

    this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getYaw(), getModulePositions(), limelight.getRobotPose().get().toPose2d());
  }

  public void drive(Translation2d translationMetersPerSecond, double rotationRadiansPerSecond, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translationMetersPerSecond.getX(),
            translationMetersPerSecond.getY(),
            rotationRadiansPerSecond,
            getYaw())
            : new ChassisSpeeds(
                translationMetersPerSecond.getX(),
                translationMetersPerSecond.getY(),
                rotationRadiansPerSecond));

    // normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for (SwerveModule mod : this.swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
    }
  }

  public void enterXStance() {
    this.swerveModules[0].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)));
    this.swerveModules[1].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)));
    this.swerveModules[2].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)));
    this.swerveModules[3].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)));
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);

    for (SwerveModule mod : this.swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public Pose2d getPose() {
    return this.swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println("[Swerve::resetOdometry] Passed pose: " + pose.getX() + ", " + pose.getY());
    this.swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModule[] getModules() {
    return this.swerveModules;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : this.swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : this.swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(this.pigeon.getYaw());
  }

  public void resetIntegratedToAbsolute() {
    for (SwerveModule mod : this.swerveModules) {
      mod.resetAngleToAbsolute();
    }
  }

  @Override
  public void periodic() {
    Optional<Pose3d> llPose = limelight.getRobotPose();
    if (llPose.isPresent()) {
      swerveDrivePoseEstimator.addVisionMeasurement(llPose.get().toPose2d(), Timer.getFPGATimestamp());
    }
    swerveDrivePoseEstimator.update(getYaw(), getModulePositions());
    
    SmartDashboard.putNumber("theta", getYaw().getDegrees());
    SmartDashboard.putNumber("odometry-x", this.swerveDrivePoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odometry-y", this.swerveDrivePoseEstimator.getEstimatedPosition().getY());

    for (SwerveModule module : this.swerveModules) {
      module.debug();
    }
  }
}
