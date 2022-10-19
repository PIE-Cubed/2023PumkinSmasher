package frc.robot;

// Imports
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Hammer {
    // Declaring Objects
    private PWMTalonSRX hammerMotor;

    public Hammer(){
        hammerMotor = new PWMTalonSRX(1);
    }


    public void movement(double leftPower, double rightPower)     {
        hammerMotor.set(rightPower-leftPower);
    }

}
