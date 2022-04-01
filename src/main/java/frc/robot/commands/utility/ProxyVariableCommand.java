package frc.robot.commands.utility;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ProxyVariableCommand extends CommandBase {
    Supplier<Command> commandFunction;
    Command command;

    public ProxyVariableCommand(Supplier<Command> commandFunction, Subsystem... requiredSubsystems) {
        this.commandFunction = commandFunction;
        addRequirements(requiredSubsystems);
    }

    @Override
    public void initialize() {
        command = commandFunction.get();
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}