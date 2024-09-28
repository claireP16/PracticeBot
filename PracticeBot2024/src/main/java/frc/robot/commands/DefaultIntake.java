package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DefaultIntake extends Command {

    private final Joystick primaryJoystick;
    private final Intake intakeSubsystem;

    public DefaultIntake(Joystick primaryJoystick, Intake intakeSubsystem){
        addRequirements(intakeSubsystem);
        this.primaryJoystick = primaryJoystick;
        this.intakeSubsystem = intakeSubsystem;
    }
}
