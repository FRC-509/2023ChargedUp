package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;

public class DeviceBuilder {
	private interface IDeviceBuilder<T> {
		public T build();
	}

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

	public static class FalconBuilder implements IDeviceBuilder<LazyTalonFX> {
		final int id;
		final String canbus;
		final PIDConstants constants;
		final NeutralMode neutralMode;
		final boolean isReverse;

		public FalconBuilder(int id, String canbus, PIDConstants constants, NeutralMode neutralMode,
				boolean isReverse) {
			this.id = id;
			this.canbus = canbus;
			this.constants = constants;
			this.neutralMode = neutralMode;
			this.isReverse = isReverse;
		}

		@Override
		public LazyTalonFX build() {
			LazyTalonFX falcon = new LazyTalonFX(id, canbus);
			falcon.setNeutralMode(neutralMode);
			falcon.config_kP(0, constants.kP);
			falcon.config_kI(0, constants.kI);
			falcon.config_kD(0, constants.kD);
			falcon.config_kF(0, constants.kF);
			falcon.setInverted(isReverse);
			falcon.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

			return falcon;
		}
	}
}
