package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule FrontLeftModule = new SwerveModule(
        DriveConstants.kFrontLeftDriveId, 
        DriveConstants.kFrontLeftAngleId, 
        DriveConstants.kFrontLeftAbsoluteEncoderId,
        DriveConstants.kFrontLeftAbsoluteEncoderOffset,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftAngleReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule FrontRightModule = new SwerveModule(
        DriveConstants.kFrontRightDriveId, 
        DriveConstants.kFrontRightAngleId, 
        DriveConstants.kFrontRightAbsoluteEncoderId,
        DriveConstants.kFrontRightAbsoluteEncoderOffset,
        DriveConstants.kFrontRightDriveReversed,
        DriveConstants.kFrontRightAngleReversed,
        DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule BackLeftModule = new SwerveModule(
        DriveConstants.kBackLeftDriveId, 
        DriveConstants.kBackLeftAngleId, 
        DriveConstants.kBackLeftAbsoluteEncoderId,
        DriveConstants.kBackLeftAbsoluteEncoderOffset,
        DriveConstants.kBackLeftDriveReversed,
        DriveConstants.kBackLeftAngleReversed,
        DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule BackRightModule = new SwerveModule(
        DriveConstants.kBackRightDriveId, 
        DriveConstants.kBackRightAngleId, 
        DriveConstants.kBackRightAbsoluteEncoderId,
        DriveConstants.kBackRightAbsoluteEncoderOffset,
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightAngleReversed,
        DriveConstants.kBackRightAbsoluteEncoderReversed);
    
    private Pigeon2 gyro = new Pigeon2(17);
    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d() {
        return new Pose2d();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putNumber("Absolute Encoder 0", FrontLeftModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder 1", FrontRightModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder 2", BackLeftModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder 3", FrontRightModule.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("gyro AccumX", gyro.getMagneticFieldX().getValueAsDouble());

    }

    public void stopModules(){
        FrontLeftModule.stop();
        FrontRightModule.stop();
        BackLeftModule.stop();
        BackRightModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kRobotMaxSpeed);

        FrontLeftModule.setDesiredState(desiredStates[0]);
        FrontRightModule.setDesiredState(desiredStates[1]);
        BackLeftModule.setDesiredState(desiredStates[2]);
        BackRightModule.setDesiredState(desiredStates[3]);
    }
}
