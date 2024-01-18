package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class TeleopDriveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier angleVel;

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier x,
                         DoubleSupplier y, DoubleSupplier angleVel) {

        this.swerveSubsystem = swerveSubsystem;

        this.x = x;
        this.y = y;
        this.angleVel = angleVel;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double x0 = x.getAsDouble();
        double y0 = y.getAsDouble();
        double angleV = angleVel.getAsDouble();

        double bandedx = Math.abs(x0) < driveDeadBand ? 0 : x0;
        double bandedy = Math.abs(y0) < driveDeadBand ? 0 : y0;

        double bandedAngle = Math.abs(angleV) < turnDeadBand ? 0 : angleV;

        ChassisSpeeds speeds = new ChassisSpeeds(bandedx * swerveSubsystem.maximumSpeed,
                bandedy * swerveSubsystem.maximumSpeed,
                bandedAngle * swerveSubsystem.maximumAngularVel);

        swerveSubsystem.driveFieldOriented(speeds);

    }}
