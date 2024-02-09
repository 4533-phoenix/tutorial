package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.ClimberMotorConstants;

public class Climber extends SubsystemBase {
    private static Climber climber = null;

    private final CANSparkMax climbMotorOne = new CANSparkMax(ClimberMotorConstants.CLIMBER_MOTOR_ONE_ID, MotorType.kBrushed);
    private final CANSparkMax climbMotorTwo = new CANSparkMax(ClimberMotorConstants.CLIMBER_MOTOR_TWO_ID, MotorType.kBrushed);

    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber(); 
        }

        return climber;
    }

    private Climber() {}

    public void runClimberUp() {
        climbMotorOne.setVoltage(ClimberMotorConstants.CLIMBER_VOLTAGE);
        climbMotorTwo.setVoltage(ClimberMotorConstants.CLIMBER_VOLTAGE);
    }

    public void runClimberDown() {
        climbMotorOne.setVoltage(-ClimberMotorConstants.CLIMBER_VOLTAGE);
        climbMotorTwo.setVoltage(-ClimberMotorConstants.CLIMBER_VOLTAGE);
    }

    public void stopClimber() {
        climbMotorOne.setVoltage(0.0);
        climbMotorTwo.setVoltage(0.0);
    }
}
