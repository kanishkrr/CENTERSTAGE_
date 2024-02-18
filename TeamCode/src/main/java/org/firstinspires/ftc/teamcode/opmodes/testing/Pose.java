package org.firstinspires.ftc.teamcode.opmodes.testing;

public class Pose {

    public double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getHeading() {
        return this.heading;
    }

    public Pose add(Pose pose) {
        return new Pose(this.x + pose.x, this.y + pose.y, this.heading + pose.heading);
    }

    public Pose subtract(Pose pose) {
        return new Pose(this.x - pose.x, this.y - pose.y, this.heading - pose.heading);
    }
}
