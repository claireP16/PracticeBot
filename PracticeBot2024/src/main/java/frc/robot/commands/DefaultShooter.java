package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DefaultShooter extends Command{
    
    private final Joystick primaryJoystick;
    private final Shooter shootSubsystem;

    public DefaultShooter(Joystick primaryJoystick, Shooter shootSubsystem){
        addRequirements(shootSubsystem);
        this.primaryJoystick = primaryJoystick;
        this.shootSubsystem = shootSubsystem;
    }
}
