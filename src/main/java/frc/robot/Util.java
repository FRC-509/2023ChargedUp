package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

public final class Util {
  // The following conversion utilites were written by team 364:
  // https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/math/Conversions.java
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

  public interface ConfigBuilder<T> {
    T build();
  }

  public static final class TalonFxConfig implements ConfigBuilder<TalonFX> {
    public final int motorId;
    public final String canbus;
    public final PIDConstants pidConstants;
    public final NeutralMode neutralMode;
    public final boolean inverted;

    public TalonFxConfig(int motorId, String canbus, PIDConstants pidConstants, NeutralMode neutralMode,
        boolean inverted) {
      this.motorId = motorId;
      this.canbus = canbus;
      this.pidConstants = pidConstants;
      this.neutralMode = neutralMode;
      this.inverted = inverted;
    }

    @Override
    public TalonFX build() {
      TalonFX motor = new TalonFX(motorId, canbus);
      motor.setNeutralMode(neutralMode);
      motor.setInverted(inverted);
      motor.setSelectedSensorPosition(0);
      motor.config_kP(0, pidConstants.kP);
      motor.config_kI(0, pidConstants.kI);
      motor.config_kD(0, pidConstants.kD);
      motor.config_kF(0, pidConstants.kF);

      return motor;
    }
  }

  public static final class SolenoidConfig implements ConfigBuilder<DoubleSolenoid> {
    public final int hubModuleId;
    public final PneumaticsModuleType pneumaticType;
    public final int forwradChannelId;
    public final int reverseChannelId;

    public SolenoidConfig(int hubModuleId, PneumaticsModuleType pneumaticType, int forwardChannelId,
        int reverseChannelId) {
      this.hubModuleId = hubModuleId;
      this.pneumaticType = pneumaticType;
      this.forwradChannelId = forwardChannelId;
      this.reverseChannelId = reverseChannelId;
    }

    @Override
    public DoubleSolenoid build() {
      return new DoubleSolenoid(hubModuleId, pneumaticType, forwradChannelId, reverseChannelId);
    }
  }

  public static final class SparkConfig implements ConfigBuilder<CANSparkMax> {
    public final int sparkId;
    public final MotorType motorType;
    public final IdleMode neutralMode;
    public final boolean inverted;

    public SparkConfig(int sparkId, MotorType motorType, IdleMode neutralMode, boolean inverted) {
      this.sparkId = sparkId;
      this.motorType = motorType;
      this.neutralMode = neutralMode;
      this.inverted = inverted;
    }

    @Override
    public CANSparkMax build() {
      CANSparkMax spark = new CANSparkMax(sparkId, motorType);
      spark.setIdleMode(neutralMode);
      spark.setInverted(inverted);

      return spark;
    }
  }

  public static final class SwerveModuleConfig {
    public final String name;
    public final int moduleId;
    public final int CANCoderId;
    public final int driveMotorId;
    public final int steerMotorId;
    public final double CANCoderOffset;
    public final double xOffset;
    public final double yOffset;

    public SwerveModuleConfig(
        String name,
        int moduleId,
        int CANCoderId,
        int driveMotorId,
        int steerMotorId,
        double CANCoderOffset,
        double xOffset,
        double yOffset) {
      this.name = name;
      this.moduleId = moduleId;
      this.CANCoderId = CANCoderId;
      this.driveMotorId = driveMotorId;
      this.steerMotorId = steerMotorId;
      this.CANCoderOffset = CANCoderOffset;
      this.xOffset = xOffset;
      this.yOffset = yOffset;
    }
  }
}
