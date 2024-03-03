package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class YellowPixelFarCommand extends SequentialCommandGroup {

    public YellowPixelFarCommand() {
        super(
                new PivotCommand(IntakeSubsystem.Mode.SCORING),
                new ArmCommand(273),
                new WaitCommand(500),
                new ExtensionCommand(-1300),
                new WaitCommand(800),
                new ClawCommand(IntakeSubsystem.Mode.SHARP, IntakeSubsystem.Mode.LEFT),
                new WaitCommand(500)
        );
    }
}
