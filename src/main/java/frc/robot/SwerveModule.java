package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Device;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.math.Conversions;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

	private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

	// Construct a new Swerve Module using a preset Configuration
	public SwerveModule(Constants.SwerveModuleConfigurations configs) {
		this.moduleNumber = configs.moduleNumber;

		// Angle Encoder Config
		CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
		canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		canCoderConfiguration.magnetOffsetDegrees = configs.angleEncoderOffset;
		this.angleEncoder = new CANCoder(configs.angleEncoderId, Device.CanBus);
		this.angleEncoder.configAllSettings(canCoderConfiguration);

		// Angle Motor Config
		this.angleMotor = new LazyTalonFX(configs.angleMotorId, Device.CanBus);
		this.angleMotor.setNeutralMode(NeutralMode.Brake);
		this.angleMotor.config_kP(0, configs.steerPID.getP());
		this.angleMotor.config_kI(0, configs.steerPID.getI());
		this.angleMotor.config_kD(0, configs.steerPID.getD());
		this.angleMotor.config_kF(0, configs.steerPID.getF());

		// Drive Motor Config
		this.driveMotor = new LazyTalonFX(configs.driveMotorId, Device.CanBus);
		this.driveMotor.setNeutralMode(NeutralMode.Brake);
		this.driveMotor.setSelectedSensorPosition(0);
		this.driveMotor.config_kP(0, configs.drivePID.getP());
		this.driveMotor.config_kI(0, configs.drivePID.getI());
		this.driveMotor.config_kD(0, configs.drivePID.getD());
		this.driveMotor.config_kF(0, configs.drivePID.getF());

		/*
		 * SupplyCurrentLimitConfiguration driveConf = new
		 * SupplyCurrentLimitConfiguration();
		 * driveConf.enable = true;
		 * driveConf.triggerThresholdCurrent = 80;
		 * driveConf.triggerThresholdTime = 0.1;
		 * driveConf.currentLimit = 0;
		 * driveMotor.configSupplyCurrentLimit(driveConf);
		 */

		/*
		 * SupplyCurrentLimitConfiguration steerConf = new
		 * SupplyCurrentLimitConfiguration();
		 * steerConf.enable = true;
		 * steerConf.triggerThresholdCurrent = 20;
		 * steerConf.triggerThresholdTime = 0.1;
		 * steerConf.currentLimit = 0;
		 * angleMotor.configSupplyCurrentLimit(steerConf);
		 */

		// "full output" will now scale to 12 Volts for all control modes.
		this.driveMotor.configVoltageCompSaturation(12);
		this.driveMotor.enableVoltageCompensation(true);
		this.lastSteerAngle = getDegrees();
	}

	public void resetAngleToAbsolute() {
		double falconAngle = Conversions.degreesToFalcon(this.angleEncoder.getAbsolutePosition(),
				Constants.angleGearRatio);
		this.angleMotor.setSelectedSensorPosition(falconAngle);
	}

	// Debug swerve module information to SmartDashboard
	public void debug() {
		SmartDashboard.putNumber(moduleNumber + " CANCoder",
				angleEncoder.getAbsolutePosition());
		SmartDashboard.putNumber(moduleNumber + " vel",
				driveMotor.getSelectedSensorVelocity());
	}

	public Rotation2d getCanCoder() {
		return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				Conversions.falconToMeters(
						this.driveMotor.getSelectedSensorPosition(),
						Constants.wheelCircumference,
						Constants.driveGearRatio),
				this.getCanCoder());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
				Conversions.falconToMPS(
						this.driveMotor.getSelectedSensorVelocity(),
						Constants.wheelCircumference,
						Constants.driveGearRatio),
				this.getCanCoder());
	}

	public void supplyVoltage(double percentOutput) {
		this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void supplyVelocity(double velocity) {
		double velocityFalcon = Conversions.MPSToFalcon(velocity, Constants.wheelCircumference,
				Constants.driveGearRatio);
		this.driveMotor.set(ControlMode.Velocity, velocityFalcon);
	}

	public double getDegrees() {
		double ticks = this.angleMotor.getSelectedSensorPosition();
		return Conversions.falconToDegrees(ticks, Constants.angleGearRatio);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
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

		double falconTarget = Conversions.degreesToFalcon(optimalTargetAngle, Constants.angleGearRatio);
		this.angleMotor.set(ControlMode.Position, falconTarget);

		if (Constants.closedLoopDriveVelocity) {
			double velocity = Conversions.MPSToFalcon(
					Math.abs(desiredState.speedMetersPerSecond),
					Constants.wheelCircumference,
					Constants.driveGearRatio);
			SmartDashboard.putNumber("TargetV" + moduleNumber, desiredState.speedMetersPerSecond);
			SmartDashboard.putNumber("SensorV" + moduleNumber, getState().speedMetersPerSecond);

			double kError = desiredState.speedMetersPerSecond - getState().speedMetersPerSecond;
			SmartDashboard.putNumber("PushBy" + moduleNumber, kError / desiredState.speedMetersPerSecond);
			SmartDashboard.putNumber("Err" + moduleNumber, kError);

			double velocityMps = Math.abs(desiredState.speedMetersPerSecond) * invertSpeed;
			Conversions.MPSToFalcon(velocityMps, Constants.wheelCircumference, Constants.driveGearRatio);
			// this.driveMotor.set(ControlMode.Velocity, velocity * invertSpeed,
			// DemandType.ArbitraryFeedForward,
			// feedforward.calculate(velocityMps));
			this.driveMotor.set(ControlMode.Velocity, velocity * invertSpeed);
		} else {
			double out = Math.abs(desiredState.speedMetersPerSecond) / Constants.maxSpeed;
			this.driveMotor.set(ControlMode.PercentOutput, invertSpeed * out);
		}
	}

	public double getBusVoltage() {
		return driveMotor.getBusVoltage();
	}
}
