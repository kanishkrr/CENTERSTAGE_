package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;

public class SlideRetractCommand extends SequentialCommandGroup {

    public SlideRetractCommand() {
        super(
                new ExtensionCommand(0)
        );
    }
}
