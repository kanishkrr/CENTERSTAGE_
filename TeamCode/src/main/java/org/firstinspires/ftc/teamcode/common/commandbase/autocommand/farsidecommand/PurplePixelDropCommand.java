package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class PurplePixelDropCommand extends SequentialCommandGroup {

    public PurplePixelDropCommand() {
        super(
                new ArmCommand(120),
                new WaitCommand(300),
                new ExtensionCommand(-1700),
                new PivotCommand(IntakeSubsystem.Mode.FLAT),
                new WaitCommand(600)
        );
    }
}
