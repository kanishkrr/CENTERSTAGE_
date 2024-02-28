package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class ScoreCommand extends SequentialCommandGroup {

    public ScoreCommand() {
        super(
                new ExtensionCommand(0),
                new ArmCommand(320),
                new PivotCommand(IntakeSubsystem.Mode.SCORING),
                new ExtensionCommand(-250),
                new WaitCommand(500)
        );
    }
}
