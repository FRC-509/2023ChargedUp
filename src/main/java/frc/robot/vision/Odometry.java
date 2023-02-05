
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

  // Finds closest and highest scoring target of desired type and offset for robot
  // to fit near target
  public Optional<Pose3d> getBestTargetLocation(TargetType targetType) {

    // Find best target pose
    if (targetType == TargetType.CUBE) {

      // Cube target
      // Get nearest april tag ID with correct alliance
      Pose3d targetTagPose = new Pose3d();
      int frontCamBestTagID = this.frontCamera.getBestAprilTagID();
      int backCamBestTagID = this.backCamera.getBestAprilTagID();

      // Get pose to best target tag
      if (DriverStation.getAlliance() == Alliance.Blue) {
        if (frontCamBestTagID == Constants.blueAllianceTargetTagIDs[0]
            || frontCamBestTagID == Constants.blueAllianceTargetTagIDs[1]
            || frontCamBestTagID == Constants.blueAllianceTargetTagIDs[2]) {

          // Valid front cam tag
          // Get correct tag pose from front camera
          targetTagPose = this.aprilTagFieldLayout.getTagPose(frontCamBestTagID).get();

        } else if (backCamBestTagID == Constants.blueAllianceTargetTagIDs[0]
            || backCamBestTagID == Constants.blueAllianceTargetTagIDs[1]
            || backCamBestTagID == Constants.blueAllianceTargetTagIDs[2]) {

          // Valid back cam tag
          // Get correct tag pose from front camera
          targetTagPose = this.aprilTagFieldLayout.getTagPose(backCamBestTagID).get();
        }

        // Get and return target tag pose of cube with safety buffer
        double tagPoseXBuffer = targetTagPose.getX() + Constants.safetyBuffer;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, targetTagPose.getY(), targetTagPose.getZ(),
            targetTagPose.getRotation());
        return Optional.of(finalPose);

      } else if (DriverStation.getAlliance() == Alliance.Red) {
        if (frontCamBestTagID == Constants.redAllianceTargetTagIDs[0]
            || frontCamBestTagID == Constants.redAllianceTargetTagIDs[1]
            || frontCamBestTagID == Constants.redAllianceTargetTagIDs[2]) {

          // Valid front cam tag
          // Get correct tag pose from front camera
          targetTagPose = this.aprilTagFieldLayout.getTagPose(frontCamBestTagID).get();

        } else if (backCamBestTagID == Constants.redAllianceTargetTagIDs[0]
            || backCamBestTagID == Constants.redAllianceTargetTagIDs[1]
            || backCamBestTagID == Constants.redAllianceTargetTagIDs[2]) {

          // Valid back cam tag
          // Get correct tag pose from front camera
          targetTagPose = this.aprilTagFieldLayout.getTagPose(backCamBestTagID).get();
        }

        // Get and return target tag pose of cube with safety buffer
        double tagPoseXBuffer = targetTagPose.getX() - Constants.safetyBuffer;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, targetTagPose.getY(), targetTagPose.getZ(),
            targetTagPose.getRotation());
        return Optional.of(finalPose);
      }

    } else if (targetType == TargetType.CONE) {

      // Cone target
      // Get nearest april tag ID with correct alliance
      Pose3d targetPose = new Pose3d();
      int frontCamBestTagID = this.frontCamera.getBestAprilTagID();
      int backCamBestTagID = this.backCamera.getBestAprilTagID();

      // Get pose to best target tag
      if (DriverStation.getAlliance() == Alliance.Blue) {
        if (frontCamBestTagID == Constants.blueAllianceTargetTagIDs[0]
            || frontCamBestTagID == Constants.blueAllianceTargetTagIDs[1]
            || frontCamBestTagID == Constants.blueAllianceTargetTagIDs[2]) {

          // Valid front cam tag
          // Set final target pose
          targetPose = this.frontCamera.getBestReflectiveTargetPose(getBestRobotPose()).get();

        } else if (backCamBestTagID == Constants.blueAllianceTargetTagIDs[0]
            || backCamBestTagID == Constants.blueAllianceTargetTagIDs[1]
            || backCamBestTagID == Constants.blueAllianceTargetTagIDs[2]) {

          // Valid back cam tag
          // Set final target pose
          targetPose = this.backCamera.getBestReflectiveTargetPose(getBestRobotPose()).get();
        }

        // Get and return target pose of cone with safety buffer
        double tagPoseXBuffer = targetPose.getX() + Constants.safetyBuffer * 2;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, targetPose.getY(), targetPose.getZ(), targetPose.getRotation());
        return Optional.of(finalPose);

      } else if (DriverStation.getAlliance() == Alliance.Red) {
        if (frontCamBestTagID == Constants.redAllianceTargetTagIDs[0]
            || frontCamBestTagID == Constants.redAllianceTargetTagIDs[1]
            || frontCamBestTagID == Constants.redAllianceTargetTagIDs[2]) {

          // Valid front cam tag
          // Set final target pose
          targetPose = this.frontCamera.getBestReflectiveTargetPose(getBestRobotPose()).get();

        } else if (backCamBestTagID == Constants.redAllianceTargetTagIDs[0]
            || backCamBestTagID == Constants.redAllianceTargetTagIDs[1]
            || backCamBestTagID == Constants.redAllianceTargetTagIDs[2]) {

          // Valid back cam tag
          // Set final target pose
          targetPose = this.backCamera.getBestReflectiveTargetPose(getBestRobotPose()).get();
        }

        // Get and return target pose of cone with safety buffer
        double tagPoseXBuffer = targetPose.getX() - Constants.safetyBuffer * 2;
        Pose3d finalPose = new Pose3d(tagPoseXBuffer, targetPose.getY(), targetPose.getZ(), targetPose.getRotation());
        return Optional.of(finalPose);
      }
    }

    // No target
    return Optional.empty();
  }

  // Gets final positioning for scoring target
  public Optional<Pose3d> getBestTargetPlacement(TargetType targetType) {

    // Find best target pose
    if (targetType == TargetType.CUBE) {

      // Cube target
      // Get best absolute pose of target and return
      return this.aprilTagFieldLayout.getTagPose(frontCamera.getBestAprilTagID());

    } else if (targetType == TargetType.CONE) {

      // Cone target
      // Get best absolute pose of target and return
      return this.frontCamera.getBestReflectiveTargetPose(getBestRobotPose());
    }

    // No target
    return Optional.empty();
  }

  // Updates robot position and offset
  public void updatePosition(Pose3d odometryPose) {

    // Get pose from cameras
    Optional<Pose3d> possibleFrontCamPose = this.frontCamera.getRobotPose();
    Optional<Pose3d> possibleBackCamPose = this.backCamera.getRobotPose();

    if (possibleFrontCamPose.isPresent() && possibleBackCamPose.isPresent()) {

      // Both cameras have pose
      // Unwrap poses
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
      // Set pose to back camera pose
      this.pose = possibleBackCamPose.get();

    } else {

      // No cameras have pose
      // Set pose to odometry pose
      this.pose = odometryPose;
    }

    // Update the field on the dashboard.
    this.field.setRobotPose(this.pose.toPose2d());

    // Set pose offset
    this.offset = pose.minus(odometryPose);
  }
}