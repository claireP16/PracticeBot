package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class DefaultClimb extends Command {

    private final Joystick primaryJoystick;
    private final Climb climbSubsystem;

    public DefaultClimb(Joystick primaryJoystick, Climb climbSubsystem) {
        addRequirements(climbSubsystem);
        this.climbSubsystem = climbSubsystem;
        this.primaryJoystick = primaryJoystick;
        }
}
