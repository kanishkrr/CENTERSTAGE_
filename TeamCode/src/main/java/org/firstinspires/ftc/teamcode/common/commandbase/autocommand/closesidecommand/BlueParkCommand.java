package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public class BlueParkCommand extends SequentialCommandGroup {

    public BlueParkCommand() {
        super(
                new PositionCommand(new Pose(-24, 6, Math.toRadians(-90)))
                        .alongWith(new PurpleDropRetractCommand()),

                new PositionCommand(new Pose(-36, 2, Math.toRadians(-85)))
        );
    }
}
