package frc.robot;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hammer {
    // Declaring Objects
    private CANSparkMax hammerMotor1;
    private CANSparkMax hammerMotor2;

    public Hammer(){
        // Init motors
        hammerMotor1 = new CANSparkMax(12, MotorType.kBrushless);
        hammerMotor2 = new CANSparkMax(13, MotorType.kBrushless);

        // Set the motors Coast
        hammerMotor1.setIdleMode(IdleMode.kCoast);
        hammerMotor2.setIdleMode(IdleMode.kCoast);

        // Set a follow group
        hammerMotor2.follow(hammerMotor1);
    }


    public void movement(double leftPower, double rightPower)     {
        hammerMotor1.set(rightPower - leftPower);
    }

}
