package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Climber;

public final class ClimbCommands {
    public static RunCommand getRunClimberUpCommand(){
        return new RunCommand(
            () -> Climber.getInstance().runClimberUp(),
            Climber.getInstance()
        );
    }

    public static RunCommand getRunClimberDownCommand() {
        return new RunCommand(
            () -> Climber.getInstance().runClimberDown(),
            Climber.getInstance()
        );
    }

    public static InstantCommand getStopClimberCommand() {
        return new InstantCommand(
            () -> Climber.getInstance().stopClimber(),
            Climber.getInstance()
        );
    }
}