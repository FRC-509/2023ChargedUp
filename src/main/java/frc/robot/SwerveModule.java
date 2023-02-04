package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleConfigurations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
  public int moduleNumber;

  // motor IDs
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  // module variables
  private double lastAngle;

  // feed forward
  // private SimpleMotorFeedforward feedforward = new
  // SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,
  // Constants.driveKA);

  // Construct a new Swerve Module using a preset Configuration
  public SwerveModule(Constants.SwerveModuleConfigurations configs) {
    this.moduleNumber = configs.moduleNumber;

    // Angle Encoder Config
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.magnetOffsetDegrees = configs.angleEncoderOffset;
    this.angleEncoder = new CANCoder(configs.angleEncoderId);

    this.angleEncoder.configAllSettings(canCoderConfiguration);

    // TalonFXConfiguration falconConfiguration = new TalonFXConfiguration();

    // Angle Motor Config
    this.angleMotor = new TalonFX(configs.angleMotorId);
    this.angleMotor.setNeutralMode(NeutralMode.Coast);
    this.angleMotor.config_kP(0, configs.steerPID.kP);
    this.angleMotor.config_kI(0, configs.steerPID.kI);
    this.angleMotor.config_kD(0, configs.steerPID.kD);
    this.angleMotor.config_kF(0, configs.steerPID.kF);

    // Drive Motor Config
    this.driveMotor = new TalonFX(configs.driveMotorId);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.driveMotor.setSelectedSensorPosition(0);

    this.driveMotor.config_kP(0, configs.drivePID.kP);
    this.driveMotor.config_kI(0, configs.drivePID.kI);
    this.driveMotor.config_kD(0, configs.drivePID.kD);
    this.driveMotor.config_kF(0, configs.drivePID.kF);
    // this.driveMotor.configVoltageCompSaturation(12); // "full output" will now
    // scale to 11 Volts for all control modes
    // // when enabled.
    // this.driveMotor.enableVoltageCompensation(true); // turn on/off feature

    lastAngle = 0.0d;
  }

  public void resetAngleToAbsolute() {
    double falconAngle = Utils.degreesToFalcon(angleEncoder.getAbsolutePosition(), Constants.angleGearRatio);
    angleMotor.setSelectedSensorPosition(falconAngle);
  }

  // Debug swerve module information to SmartDashboard
  public void debug() {
    // SmartDashboard.putNumber(moduleNumber + " CANCoder",
    // angleEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber(moduleNumber + " Integrated",
    // Utils.falconToDegrees(angleMotor.getSelectedSensorPosition(),
    // Constants.angleGearRatio));

    if (true /* if tune PID */) {
      // driveMotor.config_kP(0, Utils.serializeNumber(moduleNumber + " driveP",
      // joe.drivePID.kP));
      // driveMotor.config_kI(0, Utils.serializeNumber(moduleNumber + " driveI",
      // joe.drivePID.kI));
      // driveMotor.config_kD(0, Utils.serializeNumber(moduleNumber + " driveD",
      // joe.drivePID.kD));
      // driveMotor.config_kF(0, Utils.serializeNumber(moduleNumber + " driveF",
      // joe.drivePID.kF));
      // feedforward = new SimpleMotorFeedforward(
      // Utils.serializeNumber(moduleNumber + "KS", feedforward.ks),
      // Utils.serializeNumber(moduleNumber + "KV", feedforward.kv),
      // Utils.serializeNumber(moduleNumber + "KA", feedforward.ka));

      // var v = Utils.falconToMeters(driveMotor.getSelectedSensorPosition(),
      // Constants.wheelCircumference,
      // Constants.driveGearRatio);
      // SmartDashboard.putNumber(moduleNumber + "vel", v);
      // SmartDashboard.putNumber(moduleNumber + " BusV", driveMotor.getBusVoltage());
    }

  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Utils.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        getCanCoder());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Utils.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            Constants.wheelCircumference,
            Constants.driveGearRatio),
        getCanCoder());
  }

  public void supplyVoltage(double PercentOutput) {
    driveMotor.set(ControlMode.PercentOutput, PercentOutput);
  }

  public double getDegrees() {
    double ticks = angleMotor.getSelectedSensorPosition();
    return Utils.falconToDegrees(ticks, Constants.angleGearRatio);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    debug();
    // target angle [-180, 180]
    double targetAngle = desiredState.angle.getDegrees();
    double delta = (targetAngle - getDegrees()) % 360;
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
    double optimalTargetAngle = getDegrees() + delta;
    if (Math.abs(optimalTargetAngle - lastAngle) < Constants.maxAngularVelocity * 0.01) {
      optimalTargetAngle = lastAngle;
    }

    lastAngle = optimalTargetAngle;

    double falconTarget = Utils.degreesToFalcon(optimalTargetAngle, Constants.angleGearRatio);
    angleMotor.set(ControlMode.Position, falconTarget);

    if (Constants.closedLoopDriveVelocity) {
      double velocity = Utils.MPSToFalcon(
          Math.abs(desiredState.speedMetersPerSecond),
          Constants.wheelCircumference,
          Constants.driveGearRatio);

      driveMotor.set(ControlMode.Velocity, velocity * invertSpeed);
    } else {
      double out = Math.abs(desiredState.speedMetersPerSecond) / Constants.maxSpeed;
      driveMotor.set(ControlMode.PercentOutput, invertSpeed * out);
    }
  }
}
