package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final CANSparkMax intakeOne = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax intakeTwo = new CANSparkMax(0, Motortype.kBrushless);
    //LaserCan laser = new LaserCan();

    //LaserCan.Measurement measurement = laser.getMeasurement();

    public Intake(){
    
    }

    public void take() {
        intakeOne.set(0.5);
    }

    public void reverse(){
        intakeOne.set(-0.5);
    }

    public void


}
