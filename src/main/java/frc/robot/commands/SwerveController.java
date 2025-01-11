package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveController extends Command{
    private final SwerveSubsystem SwerveDrive;
    private final  Supplier<Double> xSpd, ySpd, angleSpd;
    private final Supplier<Boolean> fieldOriented;

    private final SlewRateLimiter xLimiter, yLimiter, aLimiter;

    public SwerveController(SwerveSubsystem SwerveDrive, Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> angleSpd, Supplier<Boolean> fieldOriented){
        this.SwerveDrive = SwerveDrive;
        this.xSpd = xSpd;
        this.ySpd = ySpd;
        this.angleSpd = angleSpd;
        this.fieldOriented = fieldOriented;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kRobotMaxSpeed);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kRobotMaxSpeed);
        this.aLimiter = new SlewRateLimiter(DriveConstants.kRobotMaxSpeed);

        addRequirements(SwerveDrive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("xSpd", xSpd.get());
        SmartDashboard.putNumber("ySpd", ySpd.get());
        SmartDashboard.putNumber("aSpd", angleSpd.get());
        double xSpeed = xSpd.get();
        double ySpeed = ySpd.get();
        double angleSpeed = angleSpd.get();

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        //angleSpeed = aLimiter.calculate(angleSpeed);


        xSpeed *= DriveConstants.kSpeedLimiter;
        ySpeed *= DriveConstants.kSpeedLimiter;
        angleSpeed *= DriveConstants.kSpeedLimiter;

        // Speed processing goes here

        ChassisSpeeds chassisSpeeds;
        if(fieldOriented.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, angleSpeed, SwerveDrive.getRotation2d()); // NOTE: ANGLE SPEED FOR CHASIS SPEED SHOULD BE IN RADIANS PER SECOND!!
        }else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
        }


        // 28, 32 / 14, 16
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putNumber("moduleStates[0]-mps", moduleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("moduleStates[1]-mps", moduleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("moduleStates[2]-mps", moduleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("moduleStates[3]-mps", moduleStates[3].speedMetersPerSecond);

        SmartDashboard.putNumber("moduleStates[0]-a", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("moduleStates[1]-a", moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("moduleStates[2]-a", moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("moduleStates[3]-a", moduleStates[3].angle.getDegrees());

        

        SwerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        SwerveDrive.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
