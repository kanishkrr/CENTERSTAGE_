package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class PickupPixelCommand extends SequentialCommandGroup {

    public PickupPixelCommand() {
        super(
                new ExtensionCommand(0),
                new WaitCommand(200),
                new ArmCommand(95),
                new WaitCommand(100),
                new ExtensionCommand(-1200),
                new PivotCommand(IntakeSubsystem.Mode.FLAT),
                new WaitCommand(250),
                new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.BOTH)
        );
    }
}
