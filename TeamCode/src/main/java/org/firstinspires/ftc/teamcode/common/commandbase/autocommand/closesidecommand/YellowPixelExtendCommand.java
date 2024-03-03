package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class YellowPixelExtendCommand extends SequentialCommandGroup {

    public YellowPixelExtendCommand() {
        super(
                new WaitCommand(700),
                new ArmCommand(200),
                new WaitCommand(900),
                new PivotCommand(IntakeSubsystem.Mode.SCORING),
                new ExtensionCommand(-1120),
                new WaitCommand(1000),
                new ClawCommand(IntakeSubsystem.Mode.SHARP, IntakeSubsystem.Mode.RIGHT),
                new WaitCommand(250)
        );
    }
}
