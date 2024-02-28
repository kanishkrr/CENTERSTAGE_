package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhitePixelPlaceCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurplePixelDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.YellowPixelFarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import java.nio.file.Path;
import java.util.List;

@Config
@Autonomous(name = "blue far test")
public class BlueFarCycle extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    public static int loc = 1;

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {


        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        Location location = Location.LEFT;

        if (loc == 2) location = Location.CENTER;
        else if (loc == 3) location = Location.RIGHT;


        Pose purplePose = new Pose();
        Pose yellowPose = new Pose();
        Pose whitePose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;
        Globals.PATH = Location.SIDE;
        Globals.PARK = Location.RIGHT;

        Constants.MAX_LINEAR_SPEED = 0.5;
        Constants.MAX_ROTATIONAL_SPEED = 0.4;

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        CommandScheduler.getInstance().schedule(new ArmCommand(160));


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            robot.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch(location) {
            case LEFT:
                purplePose = new Pose(0.5, 3, Math.toRadians(33));
                yellowPose = new Pose(-78, 12, Math.toRadians(97));
                whitePose = new Pose(-81, 20, Math.toRadians(100));
                break;
            case CENTER:
                purplePose = new Pose(1, 9.2, Math.toRadians(16));
                yellowPose = new Pose(-78, 20, Math.toRadians(97));
                whitePose = new Pose(-80, 10, Math.toRadians(100));

                break;
            case RIGHT:
                purplePose = new Pose(1.5, 10, Math.toRadians(-12.5));
                yellowPose = new Pose(-80, 29, Math.toRadians(93));
                whitePose = new Pose(-78, 10, Math.toRadians(100));
                break;
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelDropCommand()),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(200),

                        new PurpleDropRetractCommand(),

                        //new PositionCommand(new Pose(purplePose.x, purplePose.y, Math.toRadians(0))), //for now

                        new PositionCommand(new Pose(5.5, 29.1, Math.toRadians(-92.5))),

                        new WhiteFrontExtendCommand(),

                        new WhiteFrontRetractCommand(),

                        new ConditionalCommand(
                                new PathCommand(new Pose(3, 1, Math.toRadians(-89)), new Pose(-60, -1, Math.toRadians(-87))),
                                new PathCommand(new Pose(-3, 48, Math.toRadians(-90)), new Pose(-60, 48, Math.toRadians(-90))),
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),

                        new WaitCommand(5000),

                        new PositionCommand(yellowPose),

                        new YellowPixelFarCommand(),

                        new YellowPixelRetractCommand(),

                        new PositionCommand(whitePose),

                        new WhitePixelPlaceCommand(),

                        new YellowPixelRetractCommand(),

                        new WaitCommand(500),

                        new ConditionalCommand(
                                new PathCommand(new Pose(-80, 0, Math.toRadians(90)), new Pose(-80, 0, Math.toRadians(-80))),
                                new PathCommand(new Pose(-88, 44, Math.toRadians(90)), new Pose(-88, 44, Math.toRadians(-80))),
                                () -> {
                                    return Globals.PARK == Location.LEFT;
                                }
                        )
                )
        );


        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            robot.update();
            CommandScheduler.getInstance().run();
        }

        robot.kill();

    }
}
