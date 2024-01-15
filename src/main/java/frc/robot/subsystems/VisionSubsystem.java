// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.LinkedList;
import java.util.List;


public class VisionSubsystem extends SubsystemBase
{

    private final List<RobotCamera> cameras;

    /** Creates a new ExampleSubsystem. */
    public VisionSubsystem() {
        cameras = List.of(
                new RobotCamera(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0), "ms-lifecame-studio", true),
                new RobotCamera(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0), "ms-lifecame-3000", false)
                );
    }


    public List<AprilTagResult> getAprilTagResults(){
        List<AprilTagResult> output = new LinkedList<>();

        for(RobotCamera camera : cameras){
            if(camera.isAprilTag()){
                PhotonPipelineResult result = camera.getCamera().getLatestResult();
                if(result.hasTargets()){
                    List<PhotonTrackedTarget> targets = result.getTargets();
                    for(PhotonTrackedTarget target : targets) {
                        if(target.getFiducialId() > -1) {
                            output.add(new AprilTagResult(camera, target.getBestCameraToTarget(), target.getFiducialId(), target.getPoseAmbiguity()));
                        }
                    }
                }
            }
        }

        return output;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
    
    
    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
