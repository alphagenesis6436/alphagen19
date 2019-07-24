package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class SlideDrive extends DriveTrain {
    @Override
    public void update() {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        if (!(gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05)) {
            brPower = -gamepad1.left_stick_y * drivePwrMax;
            flPower = -gamepad1.left_stick_y * drivePwrMax;
        }
        if (!(gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05)) {
            frPower = gamepad1.left_stick_x * drivePwrMax;
            blPower = gamepad1.left_stick_x * drivePwrMax;
        }
        if (gamepad1.left_stick_y == 0) {
            brPower = 0;
            flPower = 0;
        }
        if (gamepad1.left_stick_x == 0) {
            frPower = 0;
            blPower = 0;
        }
        if (!(gamepad1.right_stick_x < 0.05 && gamepad1.right_stick_x > -0.05)) {
            frPower = gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            blPower = -gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            flPower = -gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
            brPower = gamepad1.right_stick_x * turnPwrMax * drivePwrMax;
        }
        for (int i = 0; i < numOfMotors; i++) {
            if (i % 4 == 0) {
                motors.get(i).setPower(clipPower(flPower));
            }
            if (i % 4 == 1) {
                motors.get(i).setPower(clipPower(frPower));
            }
            if (i % 4 == 2) {
                motors.get(i).setPower(clipPower(blPower));
            }
            if (i % 4 == 3) {
                motors.get(i).setPower(clipPower(brPower));
            }
        }
    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void telemetry() {
        telemetry.addData("DriveTrain","SlideDrive");
        telemetry.addData("Left Front Pwr", motors.get(0).getPower());
        telemetry.addData("Right Front Pwr", motors.get(1).getPower());
        telemetry.addData("Left Back Pwr", motors.get(2).getPower());
        telemetry.addData("Right Back Pwr", motors.get(3).getPower());
    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }
}
