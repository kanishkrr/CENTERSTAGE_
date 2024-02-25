package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class YellowPixelRetractCommand extends SequentialCommandGroup {

    public YellowPixelRetractCommand() {
        super(
                new ExtensionCommand(0),
                new WaitCommand(600),
                new ArmCommand(120),
                new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                new PivotCommand(IntakeSubsystem.Mode.REST)
        );
    }
}
