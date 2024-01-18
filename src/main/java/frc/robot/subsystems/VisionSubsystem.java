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

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


public class VisionSubsystem extends SubsystemBase
{

    //PhotonLib provides us a "PoseAmbiguity". According to their docs "Numbers above 0.2 are likely to be ambiguous".
    private static final double POSE_CUTOFF = 0.2;

    private final List<RobotCamera> cameras;

    /** Creates a new ExampleSubsystem. */
    public VisionSubsystem() {
        cameras = List.of(
                new RobotCamera(new Translation3d(-0.3, 0, 0), new Rotation3d(0, 0, Math.PI), "alpha-studio", true),
                new RobotCamera(new Translation3d(0.3, 0, 0), new Rotation3d(0, 0, 0), "alpha-3000", true)
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
                        if(target.getFiducialId() > -1 && target.getPoseAmbiguity() <= POSE_CUTOFF && target.getPoseAmbiguity() != -1) {
                            output.add(new AprilTagResult(camera, target.getBestCameraToTarget(), target.getFiducialId(), target.getPoseAmbiguity(), result.getTimestampSeconds()));
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
