package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand;

import androidx.core.content.SharedPreferencesKt;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class WhiteFrontRetractCommand extends SequentialCommandGroup {

    public WhiteFrontRetractCommand() {
        super(
                new WaitCommand(300),
                new ConditionalCommand(
                        new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                        new ClawCommand(IntakeSubsystem.Mode.SHARP, IntakeSubsystem.Mode.RIGHT),
                        () -> {
                            return RobotHardware.getInstance().extension.armCurrent > 130;
                        }
                ),
                new WaitCommand(500),
                new PivotCommand(IntakeSubsystem.Mode.REST),
                new ExtensionCommand(0),
                new ArmCommand(200), //used to have the pivot command after this
                new WaitCommand(300) //used to wait for 700 ms
        );
    }
}
