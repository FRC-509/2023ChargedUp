package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private final int moduleId;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder angleEncoder;

  private double lastAngle;

  public SwerveModule(Util.SwerveModuleConfig config) {
    this.moduleId = config.moduleId;

    // Angle Encoder Config
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.magnetOffsetDegrees = config.CANCoderOffset;
    this.angleEncoder = new CANCoder(config.CANCoderId, Constants.Canivore);
    this.angleEncoder.configAllSettings(canCoderConfiguration);

    // Angle Motor Config
    this.steerMotor = new TalonFX(config.steerMotorId, Constants.Canivore);
    this.steerMotor.setNeutralMode(NeutralMode.Coast);
    this.steerMotor.config_kP(0, Constants.SwerveConfig.steerPID.kP);
    this.steerMotor.config_kI(0, Constants.SwerveConfig.steerPID.kI);
    this.steerMotor.config_kD(0, Constants.SwerveConfig.steerPID.kD);
    this.steerMotor.config_kF(0, Constants.SwerveConfig.steerPID.kF);

    // Drive Motor Config
    this.driveMotor = new TalonFX(config.driveMotorId, Constants.Canivore);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.driveMotor.setSelectedSensorPosition(0);
    this.driveMotor.config_kP(0, Constants.SwerveConfig.drivePID.kP);
    this.driveMotor.config_kI(0, Constants.SwerveConfig.drivePID.kI);
    this.driveMotor.config_kD(0, Constants.SwerveConfig.drivePID.kD);
    this.driveMotor.config_kF(0, Constants.SwerveConfig.drivePID.kF);

    // scale 'full output' to 12 Volts for all control modes.
    this.driveMotor.configVoltageCompSaturation(12);
    this.driveMotor.enableVoltageCompensation(true);
    this.lastAngle = angle();
  }

  public int moduleNumber() {
    return moduleId;
  }

  public double angle() {
    return Util.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.SwerveConfig.angleGearRatio);
  }

  public double absoluteAngle() {
    return angleEncoder.getAbsolutePosition();
  }

  public SwerveModulePosition position() {
    return new SwerveModulePosition(
        Util.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            Constants.SwerveConfig.wheelCircumference,
            Constants.SwerveConfig.driveGearRatio),
        Rotation2d.fromDegrees(absoluteAngle()));
  }

  public SwerveModuleState state() {
    return new SwerveModuleState(
        Util.falconToMPS(
            this.driveMotor.getSelectedSensorVelocity(),
            Constants.SwerveConfig.wheelCircumference,
            Constants.SwerveConfig.driveGearRatio),
        Rotation2d.fromDegrees(this.absoluteAngle()));
  }

  public void resetAngleToAbsolute() {
    steerMotor.setSelectedSensorPosition(absoluteAngle());
    this.lastAngle = angle();
  }

  public void setState(SwerveModuleState state) {
    SmartDashboard.putNumber("CANCoder " + this.moduleId, this.absoluteAngle());
    double sign = 1;

    // target angle [-180, 180]
    double targetAngle = state.angle.getDegrees();
    double delta = (targetAngle - angle()) % 360;

    // limit to [-90, 90] rotation
    if (delta > 90) {
      delta -= 180;
      sign = -1;
    } else if (delta < -90) {
      delta += 180;
      sign = -1;
    }

    // calculate final target angle and set motor position
    double optimalTargetAngle = angle() + delta;

    if (Math.abs(optimalTargetAngle - this.lastAngle) < Constants.SwerveConfig.maxAngularSpeed * 0.01) {
      optimalTargetAngle = this.lastAngle;
    }

    lastAngle = optimalTargetAngle;

    double falconTarget = Util.degreesToFalcon(optimalTargetAngle, Constants.SwerveConfig.angleGearRatio);
    steerMotor.set(ControlMode.Position, falconTarget);

    if (Constants.SwerveConfig.driveClosedLoopVelocity) {
      double velocity = Util.MPSToFalcon(
          Math.abs(state.speedMetersPerSecond),
          Constants.SwerveConfig.wheelCircumference,
          Constants.SwerveConfig.driveGearRatio);

      driveMotor.set(ControlMode.Velocity, sign * velocity);
    } else {
      double percentOutput = Math.abs(state.speedMetersPerSecond) / Constants.SwerveConfig.maxSpeed;
      this.driveMotor.set(ControlMode.PercentOutput, sign * percentOutput);
    }
  }
}
