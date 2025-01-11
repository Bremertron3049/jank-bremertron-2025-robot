package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.TrackingConstants;

public class AprilTrackerSubsystem {

    private PhotonCamera Camera;
    
    public AprilTrackerSubsystem(String cameraName){
        Camera = new PhotonCamera(cameraName);


        
    }

}