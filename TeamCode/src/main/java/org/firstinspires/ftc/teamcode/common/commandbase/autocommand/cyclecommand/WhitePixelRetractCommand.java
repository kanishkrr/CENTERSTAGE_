package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class WhitePixelRetractCommand extends SequentialCommandGroup {

    public WhitePixelRetractCommand() {
        super(
                new WaitCommand(500),
                new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                new WaitCommand(700),
                new ExtensionCommand(0),
                new ArmCommand(1350)
        );
    }
}
