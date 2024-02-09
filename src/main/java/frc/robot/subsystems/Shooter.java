package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.ShooterConstants; 

public final class Shooter extends SubsystemBase {
    public static Shooter shooter = null; 
    
    private final CANSparkMax shooterMotor = new CANSparkMax (ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }

        return shooter; 
    }

    private Shooter() {}

    public void runShooter() {
        shooterMotor.setVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
    }
    
    public void stopShooter() {
        shooterMotor.setVoltage(0.0);
    }
}
