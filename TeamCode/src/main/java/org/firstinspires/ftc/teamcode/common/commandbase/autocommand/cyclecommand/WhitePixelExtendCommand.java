package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class WhitePixelExtendCommand extends SequentialCommandGroup {

    public WhitePixelExtendCommand() {
        super(
                new ArmCommand(1420),
                new WaitCommand(300),
                new ExtensionCommand(-1000),
                new PivotCommand(IntakeSubsystem.Mode.STRAIGHT),
                new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.BOTH),
                new WaitCommand(1000)
        );
    }
}
