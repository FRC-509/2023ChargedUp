
package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.IOException;

import com.ctre.phoenix.sensors.Pigeon2;

public class Odometry extends SubsystemBase {
  private Pigeon2 gyro;
  private Field2d field;

  private Translation2d odometryTranslation;
  private Translation2d offset;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private LimelightWrapper[] cameras;

  public enum CameraType {
    FRONT,
    BACK,
    NONE
  }

  public enum TargetType {
    SUBSTATION,
    CUBE,
    CONE,
    NONE
  }

  public Odometry(Pigeon2 pigeon) {
    this.gyro = pigeon;
    this.field = new Field2d();

    this.odometryTranslation = new Translation2d();
    this.offset = new Translation2d();

    this.cameras = new LimelightWrapper[2];

    this.cameras[0] = new LimelightWrapper(Constants.backCameraId);
    this.cameras[1] = new LimelightWrapper(Constants.frontCameraId);

    try {
      // aprilTagFieldLayout = new
      // AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json"));
    } catch (IOException e) {
      DriverStation.reportError("failed to load april tag field layout json",
          true);
    }
    // Display robot field position
    Shuffleboard.getTab("Robot Field Position").add(this.field);
  }

  // Gets the robot position
  public Pose2d getRobotPose() {
    return new Pose2d(
        this.odometryTranslation.plus(this.offset),
        Rotation2d.fromDegrees(this.gyro.getYaw()));
  }

  // Returns if any camera has a target
  public boolean hasTarget() {
    for (LimelightWrapper camera : this.cameras) {
      if (camera.hasTarget()) {
        return true;
      }
    }
    return false;
  }

  public void updatePose(SwerveDriveOdometry swerveOdometry) {
    this.odometryTranslation = swerveOdometry
        .getPoseMeters()
        .getTranslation();

    if (!this.hasTarget()) {
      return;
    }

    // Run through cameras and average their translations

    Translation2d newPose = new Translation2d();
    for (LimelightWrapper camera : this.cameras) {
      if (!camera.hasTarget()) {
        continue;
      }

      Translation2d cameraPose = camera
          .getRobotTransform()
          .get();

      newPose = newPose.plus(cameraPose);
    }

    newPose.div(this.cameras.length);

    // set offset to delta of translation
    this.offset = newPose.minus(this.odometryTranslation);
  }

  @Override
  public void periodic() {
  }
}