package frc.robot.subsystems;

import java.util.Spliterators.AbstractIntSpliterator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.RelativeEncoder;     
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private SparkMax DriveMotor;
    private SparkMax AngleMotor;

    private SparkMaxConfig DriveConfig;
    private SparkMaxConfig AngleConfig;

    private RelativeEncoder DriveEncoder;
    private RelativeEncoder AngleEncoder;

    private PIDController AnglePID;

    private CANcoder AbsoluteEncoder;
    private boolean ReversedAbsoluteEncoder;
    private double OffsetAbsoluteEncoder;

    public SwerveModule(int driveMotorId, int angleMotorId, int absoluteEncoderId, double absoluteEncoderOffset, boolean driveMotorReversed, boolean angleMotorReversed, boolean absoluteEncoderReversed){
        OffsetAbsoluteEncoder = absoluteEncoderOffset;
        AbsoluteEncoder = new CANcoder(absoluteEncoderId);

        DriveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        AngleMotor = new SparkMax(angleMotorId, MotorType.kBrushless);

        DriveConfig = new SparkMaxConfig();
        AngleConfig = new SparkMaxConfig();

        DriveConfig.inverted(driveMotorReversed);
        DriveConfig.encoder
            .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MPS);
        
        AngleConfig.inverted(angleMotorReversed);
        AngleConfig.encoder
            .positionConversionFactor(ModuleConstants.kAngleEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.kAngleEncoderRPM2MPS);

        DriveMotor.configure(DriveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AngleMotor.configure(AngleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        DriveEncoder = DriveMotor.getEncoder();
        AngleEncoder = AngleMotor.getEncoder();

        AnglePID = new PIDController(ModuleConstants.kP_Angle, 0, 0);
        AnglePID.enableContinuousInput(0, 2 * Math.PI);

        resetEncoders();
    }

    public double getDrivePositionRad(){
        return (2 * Math.PI) * (DriveEncoder.getPosition() / ModuleConstants.kDriveMotorGearRatio);
    }

    public double getAnglePositionRad(){
        return (2 * Math.PI) * (AngleEncoder.getPosition() / ModuleConstants.kAngleMotorGearRatio);
    }

    public double getDriveVelocityMPS(){
        
        return DriveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MPS;
    }

    public double getAngleVelocityMPS(){
        return AngleEncoder.getVelocity() * ModuleConstants.kAngleEncoderRPM2MPS;
    }

    public double getAbsoluteEncoderRad(){
        // Range is [0-1), unsigned.
        // Return position with offset.
        
        return (((AbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * (2*Math.PI)) - OffsetAbsoluteEncoder));
        //return AbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void resetEncoders(){
        DriveEncoder.setPosition(0);
        AngleEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocityMPS(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.01){
            stop();
            return;
        }

        SmartDashboard.putNumber("Pre-Optimize AnglePIDOutput-"+AngleMotor.getDeviceId(), AnglePID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        state.optimize(getState().angle);
        
        DriveMotor.set(state.speedMetersPerSecond / DriveConstants.kRobotMaxSpeed);
        AngleMotor.set(
            (AnglePID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()))*10);

        // AnglePID.calculate(getAnglePosition(), state.angle.getRadians()
        
    }

    public void stop(){
        DriveMotor.set(0);
        AngleMotor.set(0);
    }
}
