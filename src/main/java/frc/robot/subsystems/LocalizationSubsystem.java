package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.Optional;

//Code pulled from - https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
public class LocalizationSubsystem extends SubsystemBase {

    private AprilTagFieldLayout fieldLayout;
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;

    private Field2d field = new Field2d();

    public LocalizationSubsystem(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            //TODO - Is this the best way to orient the field?
            Optional<DriverStation.Alliance> allianceOption = DriverStation.getAlliance();
            if(allianceOption.isPresent()){
                DriverStation.Alliance alliance = allianceOption.get();
                fieldLayout.setOrigin(alliance == DriverStation.Alliance.Blue ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            }
        } catch (IOException e) {
            fieldLayout = null;
            e.printStackTrace();
        }
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        //TODO - Is there a better guess at initial pose?
        this.poseEstimator = new SwerveDrivePoseEstimator(swerveSubsystem.getKinematics(), swerveSubsystem.getGyroRotation(), swerveSubsystem.getModulePositions(), new Pose2d());

        setupShuffleboard(field);

        setupPathPlanner();
    }

    public void periodic() {
        if(fieldLayout != null){
            for(AprilTagResult result : visionSubsystem.getAprilTagResults()){
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(result.getId());
                if(tagPose.isPresent()){
                    Pose3d camPose = tagPose.get().transformBy(result.getAprilTagLocation().inverse());
                    Pose3d robotPose = camPose.transformBy(result.getCamera().getRobotLocation().inverse());
                    poseEstimator.addVisionMeasurement(robotPose.toPose2d(), result.getTimestamp());
                }
            }

            poseEstimator.update(swerveSubsystem.getGyroRotation(), swerveSubsystem.getModulePositions());
        }
        else {
            System.err.println("Unable to localize. Field Layout not loaded.");
        }

        field.setRobotPose(getCurrentPose());
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
                this::getCurrentPose, // Robot pose supplier
                swerveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveSubsystem::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerveSubsystem::driveRobotOriented, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Translation PID constants
                        swerveSubsystem.getPID(),
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerveSubsystem.getDriveBaseRadius(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                        // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command buildPath(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                1.0,
                1.0,
                3.14159,
                3.14159);

        Command path = AutoBuilder.pathfindToPose(targetPose, constraints);
        path.addRequirements(this);

        return path;
    }

    private void setupShuffleboard(Field2d field) {
        ShuffleboardTab tab = Shuffleboard.getTab("position");

        tab.add(field)
                .withPosition(2,0)
                .withSize(5,3);

        ShuffleboardLayout position = tab.getLayout("Robot position", BuiltInLayouts.kList)
                .withPosition(0,0)
                .withSize(2,2);


        position.addDouble("Robot X", ()-> getCurrentPose().getX());
        position.addDouble("Robot Y", ()-> getCurrentPose().getY());
        position.addDouble("Robot rotation", ()-> swerveSubsystem.getHeading().getDegrees());

    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
