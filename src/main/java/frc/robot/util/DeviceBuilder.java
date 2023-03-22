package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;
import frc.robot.util.interfaces.IDeviceBuilder;

public class DeviceBuilder {
	public static class NeoBuilder implements IDeviceBuilder<NEOSparkMax> {
		final int id;
		final IdleMode neutralMode;
		final boolean isReverse;

		public NeoBuilder(int id, IdleMode neutralMode, boolean isReverse) {
			this.id = id;
			this.neutralMode = neutralMode;
			this.isReverse = isReverse;
		}

		@Override
		public NEOSparkMax build() {
			NEOSparkMax spark = new NEOSparkMax(id);
			spark.setIdleMode(neutralMode);
			spark.setInverted(isReverse);
			return spark;
		}
	}

	public static class DoubleSolenoidBuilder implements IDeviceBuilder<DoubleSolenoid> {
		final int id;
		final int forwardChannel;
		final int reverseChannel;
		final PneumaticsModuleType type;

		public DoubleSolenoidBuilder(int id, int forwardChannel, int reverseChannel, PneumaticsModuleType type) {
			this.id = id;
			this.forwardChannel = forwardChannel;
			this.reverseChannel = reverseChannel;
			this.type = type;
		}

		@Override
		public DoubleSolenoid build() {
			return new DoubleSolenoid(id, type, forwardChannel, reverseChannel);
		}

	}

	public static class CANCoderBuilder implements IDeviceBuilder<CANCoder> {
		final int id;
		final String canBus;
		final double magnetOffset;

		public CANCoderBuilder(int id, String canBus, double magnetOffset) {
			this.id = id;
			this.canBus = canBus;
			this.magnetOffset = magnetOffset;
		}

		@Override
		public CANCoder build() {
			CANCoder CANcoder = new CANCoder(id, canBus);
			CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
			CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
			CANcoder.configMagnetOffset(magnetOffset);

			return CANcoder;
		}
	}

	public static class FalconBuilder implements IDeviceBuilder<LazyTalonFX> {
		final int id;
		final String canBus;
		final PIDWrapper constants;
		final NeutralMode neutralMode;
		final boolean isReverse;
		final FeedbackDevice feedbackDevice;
		final double gearRatio;
		final double supplyCurrentLimit;

		public FalconBuilder(int id, String canbus, PIDWrapper constants, NeutralMode neutralMode,
				boolean isReverse, FeedbackDevice feedbackDevice) {
			this.id = id;
			this.canBus = canbus;
			this.constants = constants;
			this.neutralMode = neutralMode;
			this.isReverse = isReverse;
			this.feedbackDevice = feedbackDevice;
			this.gearRatio = 1.0;
			supplyCurrentLimit = -1;
		}

		public FalconBuilder(int id, String canbus, PIDWrapper constants, NeutralMode neutralMode,
				boolean isReverse, FeedbackDevice feedbackDevice, double gearRatio) {
			this.id = id;
			this.canBus = canbus;
			this.constants = constants;
			this.neutralMode = neutralMode;
			this.isReverse = isReverse;
			this.feedbackDevice = feedbackDevice;
			this.gearRatio = gearRatio;
			supplyCurrentLimit = -1;
		}

		public FalconBuilder(int id, String canbus, PIDWrapper constants, NeutralMode neutralMode,
				boolean isReverse, FeedbackDevice feedbackDevice, double gearRatio, double supCurLim) {
			this.id = id;
			this.canBus = canbus;
			this.constants = constants;
			this.neutralMode = neutralMode;
			this.isReverse = isReverse;
			this.feedbackDevice = feedbackDevice;
			this.gearRatio = gearRatio;
			this.supplyCurrentLimit = supCurLim;
		}

		@Override
		public LazyTalonFX build() {
			LazyTalonFX falcon = new LazyTalonFX(id, canBus, gearRatio);
			falcon.setNeutralMode(neutralMode);
			falcon.setConstants(constants);
			falcon.setInverted(isReverse);
			falcon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
			falcon.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
			falcon.configSelectedFeedbackSensor(feedbackDevice);

			if (supplyCurrentLimit != -1) {
				SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration();
				currentLimitConfiguration.enable = true;
				currentLimitConfiguration.triggerThresholdCurrent = supplyCurrentLimit;
				currentLimitConfiguration.triggerThresholdTime = 0.1;
				currentLimitConfiguration.currentLimit = 0;
				falcon.configSupplyCurrentLimit(currentLimitConfiguration);
			}

			return falcon;
		}
	}
}
