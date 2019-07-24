package org.firstinspires.ftc.robotcontroller.internal;

public class MecanumODrive extends OmnidirectionalDrive {
    @Override
    public void moveRight(double velocity) {

    }

    @Override
    public void moveRight(double velocity, double revolutions) {

    }

    @Override
    public void moveForwardRight(double velocity) {

    }

    @Override
    public void moveForwardRight(double velocity, double revolutions) {

    }

    @Override
    public void moveForwardLeft(double velocity) {

    }

    @Override
    public void moveForwardLeft(double velocity, double revolutions) {

    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }

    @Override
    public void update() {
        for (int i = 0; i < numOfMotors; i++) {
            double power = turnPwrMax;
            if (i % 2 == 0)
                power *= gamepad1.right_stick_x;
            else
                power *= -gamepad1.right_stick_x;
            switch (i) {
                case 0:
                case 3: power -= gamepad1.left_stick_x; break;
                case 1:
                case 2: power += gamepad1.left_stick_x;
            }
            power -= gamepad1.left_stick_y;
            power *= drivePwrMax;
            motors.get(i).setPower(clipPower(power));
        }
    }

    @Override
    public void telemetry() {

    }
}
