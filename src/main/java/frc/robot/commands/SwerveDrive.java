package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
    private final Drivetrain driveBase;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public SwerveDrive(Drivetrain driveBase,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.driveBase = driveBase;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {

        System.out.println("X Power: " + translationXSupplier.getAsDouble());
        System.out.println("Y Power: " + translationYSupplier.getAsDouble());
        System.out.println("Rotation Power: " + rotationSupplier.getAsDouble());

        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        rotationSupplier.getAsDouble(),
                        driveBase.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0.0, 0.0, 0.0);
    }
}