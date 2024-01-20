package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPathCommand extends Command {

    LocalizationSubsystem localizationSubsystem;

    private Command path;

    public TestPathCommand(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem) {

        this.localizationSubsystem = localizationSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d pose = localizationSubsystem.getCurrentPose();
        double x = pose.getX() + 1;
        double y = pose.getY();
        Rotation2d rotation = pose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

        path = localizationSubsystem.buildPath(new Pose2d(x, y, rotation));

        CommandScheduler.getInstance().schedule(path);
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(path);
    }
}
