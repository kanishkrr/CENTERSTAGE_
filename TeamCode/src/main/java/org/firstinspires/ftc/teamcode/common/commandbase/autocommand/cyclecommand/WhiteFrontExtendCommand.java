package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class WhiteFrontExtendCommand extends SequentialCommandGroup {

    public WhiteFrontExtendCommand() {
        super(
                new PivotCommand(IntakeSubsystem.Mode.LINED),
                new ArmCommand(285),
                new WaitCommand(300), //used to be 800 (change back incase doesn't pickup pixel)
                new ArmCommand(190), //used to have a wait command for 400 after this
                new WaitCommand(200),
                new ArmCommand(160),
                new ExtensionCommand(-1030),
                new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),
                new WaitCommand(800)
        );
    }
}
