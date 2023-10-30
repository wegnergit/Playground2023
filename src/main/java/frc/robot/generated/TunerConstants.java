package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
    static class CustomSlotGains extends Slot0Configs {
        public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
        }
    }

    private static final CustomSlotGains steerGains = new CustomSlotGains(100, 0, 0.05, 0, 0);
    private static final CustomSlotGains driveGains = new CustomSlotGains(3, 0, 0, 0, 0);

    private static final double kCoupleRatio = 0.0;
    
    private static final double kDriveGearRatio = 6.538461538;
    private static final double kSteerGearRatio = 15.42857143;
    private static final double kWheelRadiusInches = 4;
    private static final int kPigeonId = 13;
    private static final boolean kSteerMotorReversed = false;
    private static final String kCANbusName = "CANBUS";
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;


    private static double kSteerInertia = 0.00001;
    private static double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(800)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(6) // Theoretical free speed is 10 meters per second at 12v applied output
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio drive turns
            .withSteerMotorInverted(kSteerMotorReversed);

    private static final int kFrontLeftDriveMotorId = 9;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 8;
    private static final double kFrontLeftEncoderOffset = -0.722900390625;

    private static final double kFrontLeftXPosInches = 11;
    private static final double kFrontLeftYPosInches = 11.25;
    private static final int kFrontRightDriveMotorId = 11;
    private static final int kFrontRightSteerMotorId = 10;
    private static final int kFrontRightEncoderId = 10;
    private static final double kFrontRightEncoderOffset = -0.150390625;

    private static final double kFrontRightXPosInches = 11;
    private static final double kFrontRightYPosInches = -11.25;
    private static final int kBackLeftDriveMotorId = 2;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 1;
    private static final double kBackLeftEncoderOffset = -0.2060546875;

    private static final double kBackLeftXPosInches = -11;
    private static final double kBackLeftYPosInches = 11.25;
    private static final int kBackRightDriveMotorId = 18;
    private static final int kBackRightSteerMotorId = 17;
    private static final int kBackRightEncoderId = 17;
    private static final double kBackRightEncoderOffset = -0.299072265625;

    private static final double kBackRightXPosInches = -11;
    private static final double kBackRightYPosInches = -11.25;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
