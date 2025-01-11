package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import java.util.List;

import javax.sound.midi.Track;

public class ApriltagTracker extends Command{
    private PhotonCamera TrackerCamera;
    private List<PhotonPipelineResult> Results;
    private PhotonPipelineResult CurrentResult;
    private PhotonTrackedTarget Target;

    public ApriltagTracker(String cameraName){
        TrackerCamera = new PhotonCamera(cameraName);
    }

    @Override
    public void initialize(){
        if(!TrackerCamera.isConnected()){
            DriverStation.reportWarning("TRACKER CAMERA IS NOT CONNECTED", true);
        }
    }

    @Override
    public void execute(){
        
        Results = TrackerCamera.getAllUnreadResults();
        CurrentResult = Results.get(0);

        if(!CurrentResult.hasTargets()){
            return;
        }

        Target = CurrentResult.getBestTarget();

        SmartDashboard.putNumber("Target Yaw", Target.yaw);
        SmartDashboard.putNumber("Target Pitch", Target.pitch);
        SmartDashboard.putNumber("Target Skew", Target.skew);
        SmartDashboard.putNumber("Target Area", Target.area);

    }

    @Override
    public void end(boolean interrupted){
        TrackerCamera.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
