package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class ClawCommand extends CommandBase {

    IntakeSubsystem.Mode clawMode, sideMode;

    public ClawCommand(IntakeSubsystem.Mode clawMode, IntakeSubsystem.Mode sideMode) {
        this.clawMode = clawMode;
        this.sideMode = sideMode;
    }

    @Override
    public void initialize() {
        RobotHardware.getInstance().claw.setClawState(clawMode, sideMode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
