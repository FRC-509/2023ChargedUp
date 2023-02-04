
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.ctre.phoenix.sensors.Pigeon2;

// Odometry
public class Odometery {

  // Properties
  private Pigeon2 gyro;
  private Field2d field;
  private Pose2d pose;
  private Transform2d offset;

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
    CUBE_HIGH,
    CUBE_LOW,
    CONE_POLE_HIGH,
    CONE_POLE_LOW,
    NONE
  }

  // Constructor
  public Odometery(Pigeon2 pigeon) {

    this.gyro = pigeon;
    field = new Field2d();
    pose = new Pose2d();
    offset = new Transform2d();

    frontCamera = new LimelightWrapper("limelight_front");
    backCamera = new LimelightWrapper("limelight_back");

    // Display robot field position
    Shuffleboard.getTab("Robot Field Position").add(field);
  }

  // Gets best robot pose
  public Pose2d getBestRobotPose() {
    return pose;
  }

  // Finds closest target type, position and camera
  public void closestTarget(TargetType returnTargetType, CameraType returnCameraType, Pose3d returnPosition) {

    // Try get camera results
    if (frontCamera.hasTarget()) {

      // Front camera
      returnCameraType = CameraType.FRONT;

      // Get april tag ID
      int tagID = frontCamera.getBestAprilTagID();

      // Find target type by tag ID
      if (tagID == 7 || tagID == 2 || tagID == 6 || tagID == 3 || tagID == 8 || tagID == 1) {

        // Cube
        // targetHeight = 0.36;
        returnTargetType = TargetType.CUBE_HIGH;

      } else if (tagID == 5 || tagID == 4) {

        // Substation
        // targetHeight = 0.59;
        returnTargetType = TargetType.SUBSTATION;

      } else {

        // None
        returnTargetType = TargetType.NONE;
      }

    } else if (backCamera.hasTarget()) {

      // Back camera
      returnCameraType = CameraType.BACK;

      // Get april tag ID
      int tagID = backCamera.getBestAprilTagID();

      // Find target type by tag ID
      if (tagID == 7 || tagID == 2 || tagID == 6 || tagID == 3 || tagID == 8 || tagID == 1) {

        // Cube
        // targetHeight = 0.36;
        returnTargetType = TargetType.CUBE_HIGH;

      } else if (tagID == 5 || tagID == 4) {

        // Substation
        // targetHeight = 0.59;
        returnTargetType = TargetType.SUBSTATION;

      } else {

        // None
        returnTargetType = TargetType.NONE;
      }
    } else {

      // Could not get cam
      returnCameraType = CameraType.NONE;
      returnTargetType = TargetType.NONE;
    }
  }

  // Updates robot position and offset
  public void updatePosition(Pose2d odometryPose) {

    // Get pose from cameras
    Optional<Pose3d> possibleFrontCamPose = frontCamera.getRobotPose();
    Optional<Pose3d> possibleBackCamPose = backCamera.getRobotPose();

    if (possibleFrontCamPose.isPresent() && possibleBackCamPose.isPresent()) {

      // Both cameras have pose
      // Unwrap poses
      Pose2d frontPose = possibleFrontCamPose.get().toPose2d();
      Pose2d backPose = possibleBackCamPose.get().toPose2d();

      // Set pose to average of camera poses
      Transform2d transform = new Transform2d(frontPose, backPose);
      Transform2d averageTransform = transform.div(2);
      pose = frontPose.plus(averageTransform);

    } else if (possibleFrontCamPose.isPresent() && possibleBackCamPose.isEmpty()) {

      // Front camera has pose
      // Set pose to front camera pose
      pose = possibleFrontCamPose.get().toPose2d();

    } else if (possibleFrontCamPose.isEmpty() && possibleBackCamPose.isPresent()) {

      // Back camera has pose
      // Set pose to back camera pose
      pose = possibleBackCamPose.get().toPose2d();

    } else {

      // No cameras have pose
      // Set pose to odometry pose
      pose = odometryPose;
    }

    // Set pose offset
    offset = pose.minus(odometryPose);
  }
}