package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class PurplePixelRetractCommand extends SequentialCommandGroup {

    public PurplePixelRetractCommand() {
        super(
                new WaitCommand(200),
                new ArmCommand(200),
                new WaitCommand(500),
                new PivotCommand(IntakeSubsystem.Mode.REST),
                new WaitCommand(1000),
                new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                new WaitCommand(1000)

        );
    }
}
