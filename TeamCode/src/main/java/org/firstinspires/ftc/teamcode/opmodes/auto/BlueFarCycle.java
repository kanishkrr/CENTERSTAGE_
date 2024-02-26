package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurplePixelDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

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

        if (loc == 2) {
            location = Location.CENTER;
        } else if (loc == 3) {
            location = Location.RIGHT;
        }

        Pose purplePose = new Pose();
        Pose yellowPose = new Pose();
        Pose whitePose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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
                purplePose = new Pose(2, 6, Math.toRadians(36));
                yellowPose = new Pose(-76, 18, Math.toRadians(90));
                whitePose = new Pose(-76, 32, Math.toRadians(90));
                break;
            case CENTER:
                purplePose = new Pose(1, 8.2, Math.toRadians(0));
                yellowPose = new Pose();
                break;
            case RIGHT:
                purplePose = new Pose(1.5, 8, Math.toRadians(-12.5));
                yellowPose = new Pose();
                break;
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelDropCommand()),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(200),

                        new PurpleDropRetractCommand(),

                        new PositionCommand(new Pose(purplePose.x, purplePose.y, Math.toRadians(-90))),

                        new PositionCommand(new Pose(5, 29.1, Math.toRadians(-90))),

                        new WhiteFrontExtendCommand(),

                        new WhiteFrontRetractCommand(),

                        new PathCommand(new Pose(3, 3, Math.toRadians(-90)), new Pose(-60, 1, Math.toRadians(-90))),

                        new PositionCommand(new Pose(-60,  1, Math.toRadians(89))),

                        new PositionCommand(yellowPose)
                                .alongWith(new ArmCommand(260)),

                        /*
                        add relocalize command
                         */

                        new ExtensionCommand(-1500)
                                .alongWith(new PivotCommand(IntakeSubsystem.Mode.SCORING).andThen(new WaitCommand(800))),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.LEFT),

                        new PositionCommand(whitePose),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(500),

                        new YellowPixelRetractCommand()
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
