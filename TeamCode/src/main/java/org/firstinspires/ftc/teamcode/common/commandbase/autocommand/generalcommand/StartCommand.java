package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class StartCommand extends SequentialCommandGroup {

    public StartCommand() {
        super(
                new ArmCommand(200),
                new PivotCommand(IntakeSubsystem.Mode.REST)
        );
    }
}
