package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    private TalonFX leftPivotMotor;
    private TalonFX rightPivotMotor;
    private CANSparkMax extensionMotor;

    public Arm() {
        this.leftPivotMotor = new TalonFX(Constants.DeviceId.leftPivotMotor);
        this.rightPivotMotor = new TalonFX(Constants.DeviceId.rightPivotMotor);
        this.extensionMotor = new CANSparkMax(Constants.DeviceId.extensionMotor, MotorType.kBrushless);
        extensionMotor.setSmartCurrentLimit(15);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        
        // Zero pivot encoders
        leftPivotMotor.setSelectedSensorPosition(0);
        rightPivotMotor.setSelectedSensorPosition(0);
        leftPivotMotor.setInverted(Constants.pivot0Inverse);
        rightPivotMotor.setInverted(Constants.pivot1Inverse);
        leftPivotMotor.setNeutralMode(NeutralMode.Brake);
        rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setPivotOutput(double output) {
        SmartDashboard.putNumber("Arm output current", extensionMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm output current", extensionMotor.getEncoder().getPosition());
        output = MathUtil.clamp(output, -1.0d, 1.0d);
        leftPivotMotor.set(ControlMode.PercentOutput, output);
        rightPivotMotor.set(ControlMode.PercentOutput, output);
    }

    public void setExtensionOutput(double output) {
        output = MathUtil.clamp(output, -1.0d, 1.0d);
        extensionMotor.set(output);
    }
}
