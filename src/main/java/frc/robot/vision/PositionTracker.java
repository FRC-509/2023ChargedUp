
package frc.robot.vision;

// import java.io.IOException;
import java.util.Optional;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.ctre.phoenix.sensors.Pigeon2;

/*
 * 
 * TO DO
 * 
 * - Set camera names and make sure front and back cameras initalized correctly
 * 
 * - begin objective based vision
 * - get positions of cube placments and be able to find cone placment positions with tape
 * 
 * 
 */

// PositionTracker
public class PositionTracker {

  // Field
  private Field2d field;

  // Gyro
  private Pigeon2 gyro;

  // Cameras
  private LimelightWrapper frontCamera;
  private LimelightWrapper backCamera;

  // Velocity and position
  public Translation3d velocity;
  public Pose3d pose;
  double deltaTime;
  double currentTime;

  double lastAprilTime;
  Pose3d lastAprilPose;
  double aprilTagTimeout;

  // Constants
  /// NOTE - Set static front and back cam to center transforms
  /// DOUBLE NOTE: These should be configured in each limelight's settings
  // interface.
  // private Transform3d frontCamToCenter;
  // private Transform3d backCamToCenter;
  // private AprilTagFieldLayout aprilTagFieldLayout;

  // // Alliance
  // public enum Alliance {
  // ALLIANCE_RED,
  // ALLIANCE_BLUE,
  // ALLIANCE_NONE
  // }

  // Camera type
  public enum CameraType {
    CAMERA_FRONT,
    CAMERA_BACK,
    CANERA_NONE
  }

  // Target type
  public enum TargetType {
    TARGET_SUBSTATION,
    TARGET_CUBE,
    TARGET_CONE_HIGH,
    TARGET_CONE_LOW,
    TARGET_NONE
  }

  // Constructor
  public PositionTracker(Pigeon2 gyroscope, Pose2d initialPose) {

    gyro = gyroscope;
    field = new Field2d();

    frontCamera = new LimelightWrapper("limelight_front");
    backCamera = new LimelightWrapper("limelight_back");

    pose = new Pose3d(initialPose);

    // Set april tag field layout
    // try {
    // aprilTagFieldLayout =
    // AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    // } catch (IOException err) {
    // DriverStation.reportError("[Position::Position] Failed to load AprilTag
    // field layout: " + err, false);
    // }

    // Display robot field position
    Shuffleboard.getTab("Robot Field Position").add(field);
    lastAprilPose = pose;
    lastAprilTime = System.currentTimeMillis() - aprilTagTimeout;

    update();
  }

  // Gets gyro rotation
  public Rotation3d getGyroRotation() {

    // Get gyro rotation
    return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
  }

  public Translation3d getGyroAccel() {

    // Get gyro accel
    short[] accel = new short[3];
    gyro.getBiasedAccelerometer(accel);
    return new Translation3d(accel[0] / 16384.0d, accel[1] / 16384.0d, accel[2] / 16384.0d);
  }

  // Finds closest april tag target type (cube or substation) and which camera
  public void closestTargetTypeAprilTag(TargetType returnTargetType, CameraType returnCameraType) {

    // Try get camera results
    if (frontCamera.hasTarget()) {

      // Front camera
      returnCameraType = CameraType.CAMERA_FRONT;

      // Get april tag ID
      int tagID = frontCamera.getBestAprilTagID();

      // Find target type by tag ID
      if (tagID == 7 || tagID == 2 || tagID == 6 || tagID == 3 || tagID == 8 || tagID == 1) {

        // Cube
        // targetHeight = 0.36;
        returnTargetType = TargetType.TARGET_CUBE;

      } else if (tagID == 5 || tagID == 4) {

        // Substation
        // targetHeight = 0.59;
        returnTargetType = TargetType.TARGET_SUBSTATION;

      } else {

        // None
        returnTargetType = TargetType.TARGET_NONE;
      }
    } else if (backCamera.hasTarget()) {

      // Back camera
      returnCameraType = CameraType.CAMERA_BACK;

      // Get april tag ID
      int tagID = backCamera.getBestAprilTagID();

      // Find target type by tag ID
      if (tagID == 7 || tagID == 2 || tagID == 6 || tagID == 3 || tagID == 8 || tagID == 1) {

        // Cube
        // targetHeight = 0.36;
        returnTargetType = TargetType.TARGET_CUBE;

      } else if (tagID == 5 || tagID == 4) {

        // Substation
        // targetHeight = 0.59;
        returnTargetType = TargetType.TARGET_SUBSTATION;

      } else {

        // None
        returnTargetType = TargetType.TARGET_NONE;
      }
    } else {

      // Could not get cam
      returnCameraType = CameraType.CANERA_NONE;
      returnTargetType = TargetType.TARGET_NONE;
    }
  }

  // Updates positioning
  public void update() {

    // Update times
    deltaTime = System.currentTimeMillis() - currentTime;
    currentTime = System.currentTimeMillis();

    // Get robot position with tags
    Optional<Pose3d> possiblePose = frontCamera.botPose();
    if (possiblePose.isPresent()) {

      // Valid robot pose
      // Get robot pose
      pose = possiblePose.get();

      // Make sure tag not timed out
      if (currentTime - lastAprilTime < aprilTagTimeout) {

        // Get position difference
        Translation3d deltaPos = pose.getTranslation().minus(lastAprilPose.getTranslation());

        // Update gyro with april tag
        velocity = deltaPos.div(currentTime - lastAprilTime);

        /// NOTE - SET GYRO ROTATION HERE WITH COMPLETE ROTATION
        gyro.setYaw(pose.getY());
      }

      // Save last april tag pose and time
      lastAprilPose = pose;
      lastAprilTime = currentTime;

    } else {

      // Could not get robot pose
      // Update velocity and position with gyro and not tag
      velocity = velocity.plus(getGyroAccel());
      Translation3d positionChange = velocity.times(deltaTime);
      pose = new Pose3d(pose.getTranslation().plus(positionChange), getGyroRotation());
    }
  }
}