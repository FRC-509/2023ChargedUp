
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Limelight Wrapper
public class LimelightWrapper {

  // Properties
  private String limelightName;
  public Pose3d cameraPose;

  // Constructor
  public LimelightWrapper(String name) {
    limelightName = name;
    setLEDState(true);
  }

  // Set LED state to on (true) or off (false)
  public void setLEDState(boolean state) {
    NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("ledMode")
        .setNumber(state ? 1 : 0);
  }

  // Get offset funcs
  public double getXOffset() {
    return NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("tx")
        .getDouble(0);
  }

  public double getYOffset() {
    return NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("ty")
        .getDouble(0);
  }

  public double getZOffset() {
    return NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("tz")
        .getDouble(0);
  }

  // Checks if camera has target
  public boolean hasTarget() {
    return NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("tv")
        .getDouble(0) != 0;
  }

  /// NOTE - WE HAVE NO CLUE WHAT THIS DOES
  public double[] getCameraTransform() {
    return NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("camtran")
        .getDoubleArray((double[]) null);
  }

  // Gets best april tag ID
  public int getBestAprilTagID() {
    Double rawTagID = NetworkTableInstance
        .getDefault()
        .getTable(limelightName)
        .getEntry("tid")
        .getDouble(0);

    return rawTagID.intValue();
  }

  // Gets robot position of field using april tags
  public Optional<Pose3d> getRobotPose() {

    // Make sure has targets
    if (!hasTarget()) {
      return Optional.empty();
    }

    // Get raw robot position
    double[] poseData;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      poseData = NetworkTableInstance
          .getDefault()
          .getTable(limelightName)
          .getEntry("botpose_wpiblue")
          .getDoubleArray(new double[6]);
    } else {
      poseData = NetworkTableInstance
          .getDefault()
          .getTable(limelightName)
          .getEntry("botpose_wpired")
          .getDoubleArray(new double[6]);
    }

    // Get and return final robot position
    Translation3d translation = new Translation3d(poseData[0], poseData[1], poseData[2]);
    Rotation3d rotation = new Rotation3d(poseData[3], poseData[4], poseData[5]);
    Pose3d robotPose = new Pose3d(translation, rotation);
    return Optional.of(robotPose);
  }
}