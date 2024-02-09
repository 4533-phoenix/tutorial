package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static RunCommand getRunShooterCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runShooter(),
            Shooter.getInstance()
        );
    }

    public static InstantCommand getStopShooterCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopShooter(),
            Shooter.getInstance()
        );
    }
}