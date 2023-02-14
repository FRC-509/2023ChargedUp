package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LazyTalonFX;
import frc.robot.util.Utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
  public int moduleNumber;

  // Motor Controllers for Drive/Steer and Steering Encoder
  private LazyTalonFX angleMotor;
  private LazyTalonFX driveMotor;
  private CANCoder angleEncoder;

  // The previously set steer angle.
  private double lastSteerAngle;

  // Construct a new Swerve Module using a preset Configuration
  public SwerveModule(Constants.SwerveModuleConfigurations configs) {
    this.moduleNumber = configs.moduleNumber;

    // Angle Encoder Config
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.magnetOffsetDegrees = configs.angleEncoderOffset;
    this.angleEncoder = new CANCoder(configs.angleEncoderId, Constants.CANIVORE);
    this.angleEncoder.configAllSettings(canCoderConfiguration);

    // Angle Motor Config
    this.angleMotor = new LazyTalonFX(configs.angleMotorId, Constants.CANIVORE);
    this.angleMotor.setNeutralMode(NeutralMode.Coast);
    this.angleMotor.config_kP(0, configs.steerPID.kP);
    this.angleMotor.config_kI(0, configs.steerPID.kI);
    this.angleMotor.config_kD(0, configs.steerPID.kD);
    this.angleMotor.config_kF(0, configs.steerPID.kF);

    // Drive Motor Config
    this.driveMotor = new LazyTalonFX(configs.driveMotorId, Constants.CANIVORE);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.driveMotor.setSelectedSensorPosition(0);
    this.driveMotor.config_kP(0, configs.drivePID.kP);
    this.driveMotor.config_kI(0, configs.drivePID.kI);
    this.driveMotor.config_kD(0, configs.drivePID.kD);
    this.driveMotor.config_kF(0, configs.drivePID.kF);
    // "full output" will now scale to 12 Volts for all control modes.
    this.driveMotor.configVoltageCompSaturation(12);
    this.driveMotor.enableVoltageCompensation(true);
    this.lastSteerAngle = getDegrees();
  }

  public void resetAngleToAbsolute() {
    double falconAngle = Utils.degreesToFalcon(this.angleEncoder.getAbsolutePosition(), Constants.angleGearRatio);
    this.angleMotor.setSelectedSensorPosition(falconAngle);
  }

  // Debug swerve module information to SmartDashboard
  public void debug() {
    SmartDashboard.putNumber(moduleNumber + " CANCoder",
        angleEncoder.getAbsolutePosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Utils.falconToMeters(
            this.driveMotor.getSelectedSensorPosition(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        this.getCanCoder());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Utils.falconToMPS(
            this.driveMotor.getSelectedSensorVelocity(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        this.getCanCoder());
  }

  public void supplyVoltage(double percentOutput) {
    this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public double getDegrees() {
    double ticks = this.angleMotor.getSelectedSensorPosition();
    return Utils.falconToDegrees(ticks, Constants.angleGearRatio);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    this.debug();
    // target angle [-180, 180]
    double targetAngle = desiredState.angle.getDegrees();
    double delta = (targetAngle - this.getDegrees()) % 360;
    double invertSpeed = 1;

    // limit to [-180, 180] rotation
    if (delta > 180) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }

    // limit to [-90, 90] rotation
    if (delta > 90) {
      delta -= 180;
      invertSpeed = -1;
    } else if (delta < -90) {
      delta += 180;
      invertSpeed = -1;
    }

    // calculate final target angle and set motor position to it
    double optimalTargetAngle = this.getDegrees() + delta;
    if (Math.abs(optimalTargetAngle - this.lastSteerAngle) < Constants.maxAngularVelocity * 0.01) {
      optimalTargetAngle = this.lastSteerAngle;
    }

    this.lastSteerAngle = optimalTargetAngle;

    double falconTarget = Utils.degreesToFalcon(optimalTargetAngle, Constants.angleGearRatio);
    this.angleMotor.set(ControlMode.Position, falconTarget);

    if (Constants.closedLoopDriveVelocity) {
      double velocity = Utils.MPSToFalcon(
          Math.abs(desiredState.speedMetersPerSecond),
          Constants.wheelCircumference,
          Constants.driveGearRatio);

      this.driveMotor.set(ControlMode.Velocity, velocity * invertSpeed);
    } else {
      double out = Math.abs(desiredState.speedMetersPerSecond) / Constants.maxSpeed;
      this.driveMotor.set(ControlMode.PercentOutput, invertSpeed * out);
    }
  }
}
