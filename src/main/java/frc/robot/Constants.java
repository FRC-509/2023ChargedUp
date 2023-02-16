package frc.robot;

public class Constants {
    public static String Canivore = "509CANIvore";

    public static final class DeviceId {
        // swerve module angle encoders
        public static final int frontLeftCANCoder = 0;
        public static final int backLeftCANCoder  = 1;
        public static final int backRightCANCoder = 2;
        public static final int frontRightCANCoder = 3;

        // swerve module drive motors
        public static final int frontLeftDriveMotor = 4;
        public static final int backLeftDriveMotor = 5;
        public static final int backRightDriveMotor = 6;
        public static final int frontRightDriveMotor = 7;

        // swerve module steer motors
        public static final int frontLeftSteerMotor = 8;
        public static final int backLeftSteerMotor = 9;
        public static final int backRightSteerMotor = 10;
        public static final int frontRightSteerMotor = 11;

        // arm motors
        public static final int extensionMotor = 12;
        public static final int leftPivotMotor = 12;
        public static final int rightPivotMotor = 13;

        // intake motors
        public static final int leftPrimaryIntakeMotor = 14;
        public static final int rightSecondaryIntakeMotor = 15;
    }

    public static final class SwerveModuleConfig {
        public int moduleId;
        public int CANCoderId;
        public int driveMotorId;
        public int steerMotorId;
        public double CANCoderOffset;
        public PIDConstants drivePID;
        public PIDConstants steerPID;

        public SwerveModuleConfig(int moduleId, int CANCoderId, 
            int driveMotorId, int steerMotorId, double CANCoderOffset, 
            PIDConstants drivePID, PIDConstants steerPID) 
        {
            this.moduleId = moduleId;
            this.CANCoderId = CANCoderId;
            this.driveMotorId = driveMotorId;
            this.steerMotorId = steerMotorId;
            this.CANCoderOffset = CANCoderOffset;
            this.drivePID = drivePID;
            this.steerPID = steerPID;
        }
      }

    public static final double intakeSpinVelocity = 0.0d;



    public static final boolean pivot0Inverse = false;
    public static final boolean pivot1Inverse =  true;
    public static final double pivotGearRatio = 20.0d;

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

}
