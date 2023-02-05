/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Utils.PIDConstants;

public class Arm extends SubsystemBase {
  private final TalonFX pivotMotor1 = new TalonFX(0);
  private final TalonFX pivotMotor2 = new TalonFX(0);

  private final CANSparkMax extensionMotor = new CANSparkMax(0, MotorType.kBrushless);
  private AbsoluteEncoder extensionEncoder;

  private static final double pivotGearRatio = 1.0d;
  private static final PIDConstants pivotConstants = new PIDConstants(1.0, 0, 0, 0);

  public Arm() {
    pivotMotor1.config_kP(0, pivotConstants.kP);
    pivotMotor1.config_kI(0, pivotConstants.kI);
    pivotMotor1.config_kD(0, pivotConstants.kD);
    pivotMotor1.config_kF(0, pivotConstants.kF);
    pivotMotor2.config_kP(0, pivotConstants.kP);
    pivotMotor2.config_kI(0, pivotConstants.kI);
    pivotMotor2.config_kD(0, pivotConstants.kD);
    pivotMotor2.config_kF(0, pivotConstants.kF);
    pivotMotor1.setSelectedSensorPosition(0);
    pivotMotor2.setSelectedSensorPosition(0);
    pivotMotor1.setNeutralMode(NeutralMode.Brake);
    pivotMotor2.setNeutralMode(NeutralMode.Brake);
    extensionEncoder = extensionMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotMotor2.follow(pivotMotor1, FollowerType.AuxOutput1);
  }

  public void setPivotAngle(double angle) {
    double lastAngle = Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), pivotGearRatio);
    double delta = angle - (lastAngle % 360.0d);
    double outputAngle = Utils.degreesToFalcon(lastAngle + delta, pivotGearRatio);

    // double lastAngle2 =
    // Utils.falconToDegrees(pivotMotor2.getSelectedSensorPosition(),
    // pivotGearRatio);
    // double delta2 = angle - (lastAngle2 % 360.0d);
    // double outputAngle2 = Utils.degreesToFalcon(lastAngle2 + delta2,
    // pivotGearRatio);

    pivotMotor1.set(ControlMode.Position, outputAngle);

    // pivotMotor2.set(ControlMode.Position, outputAngle2);
  }

  public double getPivotAngle() {
    return Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), pivotGearRatio) % 360.0d;
  }
}
*/