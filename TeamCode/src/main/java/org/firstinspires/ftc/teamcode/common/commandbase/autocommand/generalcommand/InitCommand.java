package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class InitCommand extends SequentialCommandGroup {

    public InitCommand() {
        super(
            new ArmCommand(550),
            new WaitCommand(2000),
            new PivotCommand(IntakeSubsystem.Mode.INIT)
        );
    }
}
