package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class PivotCommand extends CommandBase {

    IntakeSubsystem.Mode mode;

    public PivotCommand(IntakeSubsystem.Mode mode) {
        this.mode = mode;
    }
    @Override
    public void initialize() {
        RobotHardware.getInstance().claw.changeAngleState(mode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
