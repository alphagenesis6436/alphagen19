package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

/** motor order: 0 -> east, 1 -> north, 2 -> south, 3 -> west */
public class SlideDrive extends OmnidirectionalDrive {

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void moveForward(double velocity) {

    }

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
    public void turnClockwise(double angularVelocity) {

    }

    @Override
    public void update() {
        double northPwr, southPwr, westPwr, eastPwr;
        if (!(gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05)) {
            westPwr = -gamepad1.left_stick_y * drivePwrMax;
            eastPwr = -gamepad1.left_stick_y * drivePwrMax;
        } else {
            westPwr = 0;
            eastPwr = 0;
        }
        if (!(gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05)) {
            northPwr = gamepad1.left_stick_x * drivePwrMax;
            southPwr = gamepad1.left_stick_x * drivePwrMax;
        } else {
            northPwr = 0;
            southPwr = 0;
        }
        if (!(gamepad1.right_stick_x < 0.05 && gamepad1.right_stick_x > -0.05)) {
            northPwr = gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            southPwr = -gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            eastPwr = -gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            westPwr = gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
        }
        motors.get(0).setPower(clipPower(eastPwr));
        motors.get(1).setPower(clipPower(northPwr));
        motors.get(2).setPower(clipPower(southPwr));
        motors.get(3).setPower(clipPower(westPwr));
    }

    @Override
    public void telemetry() {

    }
}
