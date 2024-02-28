package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class WhitePixelPlaceCommand extends SequentialCommandGroup {

    public WhitePixelPlaceCommand() {
        super(
          new ArmCommand(320),
          new PivotCommand(IntakeSubsystem.Mode.SCORING),
          new WaitCommand(400),
          new ExtensionCommand(-1550),
          new WaitCommand(700),
          new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),
          new WaitCommand(400)
        );
    }
}
