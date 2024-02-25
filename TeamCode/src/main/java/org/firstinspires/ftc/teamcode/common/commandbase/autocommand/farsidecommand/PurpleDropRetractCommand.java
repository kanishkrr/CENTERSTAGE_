package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class PurpleDropRetractCommand extends SequentialCommandGroup {

    public PurpleDropRetractCommand() {
        super(
                new ExtensionCommand(0),
                new ArmCommand(180),
                new PivotCommand(IntakeSubsystem.Mode.REST),
                new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                new WaitCommand(1000)
        );
    }
}
