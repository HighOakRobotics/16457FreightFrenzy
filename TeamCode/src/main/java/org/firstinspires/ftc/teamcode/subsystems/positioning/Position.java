package org.firstinspires.ftc.teamcode.subsystems.positioning;

import java.util.concurrent.TimeUnit;

public class Position {
    private final double time;

    private final boolean hasX;
    private final boolean hasY;
    private final boolean hasHeading;

    private final Double xPosition; // inches
    private final Double yPosition; // inches
    private final Double heading; // radians

    public Position() {
        this(0.0, 0.0, 0.0);
    }

    public Position(Double x, Double y, Double heading) {
        time = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.NANOSECONDS);

        hasX = x != null;
        hasY = y != null;
        hasHeading = heading != null;

        xPosition = x;
        yPosition = y;
        this.heading = heading;
    }

    public double getTime() {
        return time;
    }

    public Double getX() {
        return xPosition;
    }

    public Double getY() {
        return yPosition;
    }

    public Double getHeading() {
        return heading;
    }

    public boolean hasX() {
        return hasX;
    }

    public boolean hasY() {
        return hasY;
    }

    public boolean hasHeading() {
        return hasHeading;
    }

    public boolean hasAll() {return hasX && hasY && hasHeading;}
}