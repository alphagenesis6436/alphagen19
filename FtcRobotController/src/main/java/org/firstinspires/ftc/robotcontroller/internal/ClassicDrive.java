package org.firstinspires.ftc.robotcontroller.internal;
/** motor order: 0 -> (front)left, 1 -> (front)right, 2 -> back-left, 3 -> back-right */
public abstract class ClassicDrive extends DriveTrain {

    @Override
    public void move(double... velocities) {
        for (int i = 0; i < numOfMotors; i++)
            motors.get(i).setPower(velocities[i]);
    }

    public void move(double velocity) {
        for (int i = 0; i < numOfMotors; i++)
            motors.get(i).setPower(velocity);
    }

    @Override
    public void moveForward(double velocity) {
        move(velocity);
    }

    @Override
    public void turnClockwise(double angVelocity) {
        move(angVelocity, -angVelocity, angVelocity, -angVelocity);
    }

    @Override
    public void telemetry() {
        telemetry.addData(">", "Classic Drive Telemetry");
        telemetry.addData("Left Motor Power",
                String.format("%.2f", motors.get(0).getPower()));
        telemetry.addData("Right Motor Power",
                String.format("%.2f", motors.get(1).getPower()));
    }
}
