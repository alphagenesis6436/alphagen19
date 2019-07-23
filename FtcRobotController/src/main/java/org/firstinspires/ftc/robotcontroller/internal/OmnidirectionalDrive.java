package org.firstinspires.ftc.robotcontroller.internal;

public abstract class OmnidirectionalDrive extends DriveTrain {

    public abstract void moveRight(double velocity);

    public abstract void moveRight(double velocity, double revolutions);

    public abstract void moveForwardRight(double velocity);

    public abstract void moveForwardRight(double velocity, double revolutions);

    public abstract void moveForwardLeft(double velocity);

    public abstract void moveForwardLeft(double velocity, double revolutions);
}
