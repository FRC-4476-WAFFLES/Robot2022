/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Add your docs here.
 */
public class CommandSwitch extends CommandBase {
    private int i = -1;
    private final Command[] list;

    public CommandSwitch(Command... commands) {
        list = commands;
    }

    @Override
    public void initialize() {
        i++;
        if (i >= list.length) {
            i = 0;
        }

        list[i].initialize();
    }

    @Override
    public void execute() {
        list[i].execute();
    }

    @Override
    public void end(boolean interrupted) {
        list[i].end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return list[i].isFinished();
    }
}