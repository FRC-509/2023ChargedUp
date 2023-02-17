package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
  private TalonFX leftPivotMotor;
  private TalonFX rightPivotMotor;
  private CANSparkMax extensionMotor;

  public Arm() {
    this.leftPivotMotor = Constants.leftPivotMotor.build();
    this.rightPivotMotor = Constants.rightPivotMotor.build();
    this.extensionMotor = Constants.extensionMotor.build();

    this.extensionMotor.setSmartCurrentLimit(15);
  }

  public void setPivotOutput(double output) {
    SmartDashboard.putNumber("Arm output current",
        extensionMotor.getOutputCurrent());

    SmartDashboard.putNumber("Arm output position",
        extensionMotor.getEncoder().getPosition());

    output = MathUtil.clamp(output, -1.0d, 1.0d);
    leftPivotMotor.set(ControlMode.PercentOutput, output);
    rightPivotMotor.set(ControlMode.PercentOutput, output);
  }

  public void setExtensionOutput(double output) {
    output = MathUtil.clamp(output, -1.0d, 1.0d);
    extensionMotor.set(output);
  }
}
