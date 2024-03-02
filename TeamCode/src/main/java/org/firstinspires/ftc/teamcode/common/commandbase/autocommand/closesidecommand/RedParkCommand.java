package org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public class RedParkCommand extends SequentialCommandGroup {

    public RedParkCommand() {
        super(
                new PositionCommand(new Pose(24, 7, Math.toRadians(90)))
                        .alongWith(new PurpleDropRetractCommand()),

                new PositionCommand(new Pose(36, 3, Math.toRadians(85)))
        );
    }
}
