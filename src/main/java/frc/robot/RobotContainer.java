// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopDriveCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory() + "/swerve"));

    private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(visionSubsystem, swerveSubsystem);
    
    // Replace with CommandPS4Controller or CommandJoystick if needed

    private final XboxController xbox = new XboxController(1);

    private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(swerveSubsystem,
            ()->-xbox.getLeftY(), ()->-xbox.getLeftX(), ()->-xbox.getRightX());
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        new Trigger(xbox::getAButtonPressed)
                .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        new Trigger(xbox::getYButtonPressed).onTrue(
                new InstantCommand((swerveSubsystem::zeroOdometry)));

        new Trigger(xbox::getBButton).whileTrue(localizationSubsystem.buildPath(
                new Pose2d(1, 0, new Rotation2d(0))));

        new Trigger(xbox::getXButton).whileTrue(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test path")));

        new Trigger(()-> xbox.getPOV() == 0).whileTrue(
                localizationSubsystem.buildPath(Constants.speakerPark));

        new Trigger(()-> xbox.getPOV() == 90).whileTrue(
                localizationSubsystem.buildPath(Constants.ampPark));

        new Trigger(()-> xbox.getPOV() == 180).whileTrue(
                localizationSubsystem.buildPath(Constants.leftStagePark));

        new Trigger(()-> xbox.getPOV() == 270).whileTrue(
                localizationSubsystem.buildPath(Constants.rightStagePark));

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, teleopDriveCommand);
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

}
