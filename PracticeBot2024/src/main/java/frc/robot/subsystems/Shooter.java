package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final CANSparkMax transport1
    private final #motor transport2 = new #motor();
    private final #motor shooter = new #motor();

    public Shooter(){

    }

    public void shoot() {
        transport1.set(0.5);
        transport2.set(0.5);
        shooter.set(0.5);
    }

    public void reverseShoot() {
        transport1.set(-0.2);
        transport2.set(-0.2);
        shooter.set(-0.2);
    }

}

