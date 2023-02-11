
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.Pigeon2;

// Odometry
public class Odometry {

  // Properties
  private Pigeon2 gyro;
  private Field2d field;
  private Pose3d pose;
  private Transform3d offset;
  private AprilTagFieldLayout aprilTagFieldLayout;

  // Cameras
  private LimelightWrapper frontCamera;
  private LimelightWrapper backCamera;

  // Camera type
  public enum CameraType {
    FRONT,
    BACK,
    NONE
  }

  // Target type
  public enum TargetType {
    SUBSTATION,
    CUBE,
    CONE,
    NONE
  }

  // Constructor
  public Odometry(Pigeon2 pigeon) {

    this.gyro = pigeon;
    this.field = new Field2d();
    this.pose = new Pose3d();
    this.offset = new Transform3d();

    this.frontCamera = new LimelightWrapper("limelight_front");
    this.backCamera = new LimelightWrapper("limelight_back");

    // Display robot field position
    Shuffleboard.getTab("Robot Field Position").add(this.field);
  }

  // Gets best robot pose
  public Pose3d getBestRobotPose() {
    return this.pose;
  }

  // Updates robot position and offset
  public void updatePosition(Pose3d odometryPose) {

    // Get pose from cameras
    Optional<Pose3d> possibleFrontCamPose = this.frontCamera.getRobotPose();
    Optional<Pose3d> possibleBackCamPose = this.backCamera.getRobotPose();

    if (possibleFrontCamPose.isPresent() && possibleBackCamPose.isPresent()) {

      // Both cameras have pose
      Pose3d frontPose = possibleFrontCamPose.get();
      Pose3d backPose = possibleBackCamPose.get();

      // Set pose to average of camera poses
      Transform3d transform = new Transform3d(frontPose, backPose);
      Transform3d averageTransform = transform.div(2);
      this.pose = frontPose.plus(averageTransform);

    } else if (possibleFrontCamPose.isPresent() && possibleBackCamPose.isEmpty()) {

      // Front camera has pose
      // Set pose to front camera pose
      this.pose = possibleFrontCamPose.get();

    } else if (possibleFrontCamPose.isEmpty() && possibleBackCamPose.isPresent()) {

      // Back camera has pose
      this.pose = possibleBackCamPose.get();

    } else {

      // No cameras have pose
      this.pose = odometryPose;
    }

    // Update the field on the dashboard
    this.field.setRobotPose(this.pose.toPose2d());

    // Set pose offset
    this.offset = pose.minus(odometryPose);
  }
}