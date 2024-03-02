package org.firstinspires.ftc.teamcode.common.commandbase.drivecommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.List;

@Config
public class PositionCommand extends CommandBase {

    private RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;
    public Pose targetPose;

    public static double xP = 0.13;
    public static double xD = 0.012;

    public static double yP = 0.13;
    public static double yD = 0.012;

    public static double hP = 2;
    public static double hD = 0.045;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.75;
    public static double ALLOWED_HEADING_ERROR = 0.02;

    private ElapsedTime timer;
    private ElapsedTime stable;
    public static double STABLE_MS = 250;
    public static double DEAD_MS = 2500;



    private final double  MAX_TRANSLATIONAL_POWER = Constants.MAX_LINEAR_SPEED;
    private final double  MAX_ROTATIONAL_POWER = Constants.MAX_ROTATIONAL_SPEED;
    private final double K_STATIC = 1.85;

    public PositionCommand(Pose targetPose) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;

        xController.reset();
        yController.reset();
        hController.reset();
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose current = drivetrain.localizer.getPose();
        Pose powers = getPower(current);

        drivetrain.set(powers);
    }

    public Pose getPower(Pose robotPose) {
        if(targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if(targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_shifted = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_shifted = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);
        x_shifted = Range.clip(x_shifted, -MAX_TRANSLATIONAL_POWER / K_STATIC, MAX_TRANSLATIONAL_POWER / K_STATIC);
        y_shifted = Range.clip(y_shifted, -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);

        return new Pose(x_shifted * K_STATIC, y_shifted, hPower);
    }

    @Override
    public void end(boolean interuppted) {
        drivetrain.setPowers(0, 0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        Pose current = drivetrain.localizer.getPose();
        Pose delta = targetPose.subtract(current);

        List<Double> velocities = drivetrain.localizer.getWheelVelocities();

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR || velocities.get(0) > 0.5 || velocities.get(1) > 0.5 || velocities.get(2) > 0.5) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;

    }
}
