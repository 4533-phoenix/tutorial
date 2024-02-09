package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AmpCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Constants.ControllerConstants;;

public final class RobotContainer {
    private static final XboxController controller = new XboxController(0);

    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("Tutorial Auto", AutoCommands.tutorialAuto())
    );

    private static final Map<String, Pose2d> autoPositions = Map.ofEntries(
        Map.entry("Tutorial Auto", new Pose2d())
    );

    public static void registerButtons() {
        JoystickButton runIntakeForwardButton = new JoystickButton(controller, 0);
        runIntakeForwardButton.whileTrue(IntakeCommands.getRunIntakeForwardCommand());
        runIntakeForwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

        JoystickButton runIntakeBackwardButton = new JoystickButton(controller, 1);
        runIntakeBackwardButton.whileTrue(IntakeCommands.getRunIntakeBackwardCommand());
        runIntakeBackwardButton.onFalse(IntakeCommands.getStopIntakeCommand());

        JoystickButton runShooter = new JoystickButton(controller, 2);
        runShooter.whileTrue(ShooterCommands.getRunShooterCommand());
        runShooter.onFalse(ShooterCommands.getStopShooterCommand());

        JoystickButton runClimberUp = new JoystickButton(controller, ControllerConstants.BUTTON_RB);
        runClimberUp.whileTrue(ClimbCommands.getRunClimberUpCommand());
        runClimberUp.onFalse(ClimbCommands.getStopClimberCommand());

        JoystickButton runClimberDown = new JoystickButton(controller, ControllerConstants.BUTTON_LB);
        runClimberDown.whileTrue(ClimbCommands.getRunClimberDownCommand());
        runClimberDown.onFalse(ClimbCommands.getStopClimberCommand());

        JoystickButton runAmpForward = new JoystickButton (controller, ControllerConstants.BUTTON_X);
        runAmpForward.whileTrue(AmpCommands.getFunctionalCommandOne());

        JoystickButton runAmpBackward = new JoystickButton(controller, ControllerConstants.BUTTON_Y);
        runAmpBackward.whileTrue(AmpCommands.getFunctionalCommandTwo());
    }

    public static void registerSubsystems() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        commandScheduler.registerSubsystem(Swerve.getInstance(), Intake.getInstance(), Shooter.getInstance(), Climber.getInstance(), Amp.getInstance());

        commandScheduler.setDefaultCommand(
            Swerve.getInstance(), 
            SwerveCommands.getDefaultDriveCommand()
        );
    }

    public static XboxController getController() {
        return controller;
    }

    public static Command getAutonomous(String key) {
        return autoCommands.get(key);
    }

    public static Pose2d getAutonomousPosition(String key) {
        return autoPositions.get(key);
    }
}
