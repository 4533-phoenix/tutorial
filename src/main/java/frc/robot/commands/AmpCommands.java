package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Amp;

public class AmpCommands {
    public static RunCommand getPutArmForwardCommand() {
        return new RunCommand(
            () -> Amp.getInstance().putArmForward(),
            Amp.getInstance()
        );
    }

    public static RunCommand getPutArmBackwardCommand() {
        return new RunCommand(
            () -> Amp.getInstance().putArmBackward(),
            Amp.getInstance()
        );
    }

    public static InstantCommand getStopArmCommand() {
        return new InstantCommand(
            () -> Amp.getInstance().stopArm(),
            Amp.getInstance()
        );
    }

    public static Command getFunctionalCommandOne() {
        return new FunctionalCommand(
            () -> {},
            () -> Amp.getInstance().putArmForward(), 
            (isFinished) -> Amp.getInstance().stopArm(), 
            () -> Amp.getInstance().getSwitchOne(), 
            Amp.getInstance()
        );
    }

    public static Command getFunctionalCommandTwo() {
        return new FunctionalCommand(
            () -> {},
            () -> Amp.getInstance().putArmBackward(),
            (isFinished) -> Amp.getInstance().stopArm(),
            () -> Amp.getInstance().getSwitchTwo(),
            Amp.getInstance()
        );
    }
}
