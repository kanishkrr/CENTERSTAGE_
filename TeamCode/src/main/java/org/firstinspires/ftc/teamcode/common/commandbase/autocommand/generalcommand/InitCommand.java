package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;

public class InitCommand extends SequentialCommandGroup {

    public InitCommand() {
        super(
            new ArmCommand(550)

        );
    }
}
