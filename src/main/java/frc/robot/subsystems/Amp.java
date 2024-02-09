package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.AmpConstants;;

public class Amp extends SubsystemBase {
    private static Amp amp = null;

    DigitalInput ampSwitchOne = new DigitalInput(AmpConstants.AMP_SWITCH_ONE_CHANNEL);
    DigitalInput ampSwitchTwo = new DigitalInput(AmpConstants.AMP_SWITCH_TWO_CHANNEL);

    private final CANSparkMax ampMotorOne = new CANSparkMax(AmpConstants.AMP_MOTOR_ONE_ID, MotorType.kBrushless);

    public static Amp getInstance() {
        if (amp == null) {
            amp = new Amp();
        }

        return amp; 
    }

    private Amp(){}

    public void putArmForward() {
        ampMotorOne.setVoltage(AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    public void putArmBackward() {
        ampMotorOne.setVoltage(-AmpConstants.AMP_MOTOR_VOLTAGE);
    }

    public void stopArm() {
        ampMotorOne.setVoltage(0.0);
    }

    public boolean getSwitchOne() {
        return ampSwitchOne.get();
    }

    public boolean getSwitchTwo() {
        return ampSwitchTwo.get();
    }
}
