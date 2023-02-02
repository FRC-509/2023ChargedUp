package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Utils {
  /*
   * public static double CANcoderToDegrees(double positionCounts, double
   * gearRatio) {
   * return positionCounts * (360.0 / (gearRatio * 4096.0));
   * }
   */

  /**
   * @param counts    Falcon Position Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees   Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Position Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM       RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
   *                  RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity      Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
   *                      Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param positionCounts Falcon Position Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Wheel
   * @return Meters
   */
  public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
    return positionCounts * (circumference / (gearRatio * 2048.0));
  }

  /**
   * @param meters        Meters
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Wheel
   * @return Falcon Position Counts
   */
  public static double metersToFalcon(double meters, double circumference, double gearRatio) {
    return meters / (circumference / (gearRatio * 2048.0));
  }

  public static double serializeNumber(String key, double val) {
    SmartDashboard.setDefaultNumber(key, val);
    return SmartDashboard.getNumber(key, 0.0);
  }

  public static boolean serializeBoolean(String key, boolean val) {
    SmartDashboard.setDefaultBoolean(key, val);
    return SmartDashboard.getBoolean(key, false);
  }

  public static final class PIDConstants {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public PIDConstants(double kP, double kI, double kD, double kF) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
    }
  }
}