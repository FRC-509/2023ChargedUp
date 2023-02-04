
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.Pigeon2;

// Odometry
public class Odometery {

  // Properties
  private Pigeon2 gyro;
  private Field2d field;
  private Pose2d pose;
  private Transform2d offset;
  private AprilTagFieldLayout aprilTagfieldLayout;

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

  // Finds closest and highest scoring target of desired type and offset for robot
  // to fit near target
  public Optional<Pose3d> getBestTargetLocation(TargetType targetType) {

    // Find best target pose
    if (targetType == TargetType.CUBE) {

      // Cube target
      // Get nearest april tag ID with correct alliance
      Pose3d tagPose = new Pose3d();
      int frontCamBestTagID = frontCamera.getBestAprilTagID();
      int backCamBestTagID = backCamera.getBestAprilTagID();

      // Get pose to best cube target
      if (DriverStation.getAlliance() == Alliance.Blue) {
        if (frontCamBestTagID == 6 || frontCamBestTagID == 7 || frontCamBestTagID == 8) {

          // Valid front cam tag
          // Get correct tag pose from front camera
          tagPose = aprilTagfieldLayout.getTagPose(frontCamBestTagID).get();

        } else if (backCamBestTagID == 6 || backCamBestTagID == 7 || backCamBestTagID == 8) {

          // Valid back cam tag
          // Get correct tag pose from front camera
          tagPose = aprilTagfieldLayout.getTagPose(backCamBestTagID).get();
        }

        // Get and return tag pose with safety buffer
        double tagPoseXBuffer = tagPose.getX() + Constants.safetyBuffer;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, tagPose.getY(), tagPose.getZ(),
            tagPose.getRotation());
        return Optional.of(finalPose);

      } else if (DriverStation.getAlliance() == Alliance.Red) {
        if (frontCamBestTagID == 3 || frontCamBestTagID == 2 || frontCamBestTagID == 1) {

          // Valid front cam tag
          // Get correct tag pose from front camera
          tagPose = aprilTagfieldLayout.getTagPose(frontCamBestTagID).get();

        } else if (backCamBestTagID == 3 || backCamBestTagID == 2 || backCamBestTagID == 1) {

          // Valid back cam tag
          // Get correct tag pose from front camera
          tagPose = aprilTagfieldLayout.getTagPose(backCamBestTagID).get();
        }

        // Get and return tag pose with safety buffer
        double tagPoseXBuffer = tagPose.getX() - Constants.safetyBuffer;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, tagPose.getY(), tagPose.getZ(),
            tagPose.getRotation());
        return Optional.of(finalPose);
      }

    } else if (targetType == TargetType.CONE) {

      // Cone target

    }
  }

  // Gets final positioning for scoring target
  public void getBestTargetPlacement(TargetType targetType) {

    // Find best target pose
    if (targetType == TargetType.CUBE) {

      // Cube target

    } else if (targetType == TargetType.CONE) {

      // Cone target

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