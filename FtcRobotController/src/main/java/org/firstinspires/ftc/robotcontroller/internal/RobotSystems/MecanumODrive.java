package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class MecanumODrive extends DriveTrain{
    @Override
    public void update() {
        double flPower, frPower, blPower, brPower;
        flPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax;
        frPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax;
        brPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax;
        blPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax;
        for (int i = 0; i < numOfMotors; i++) {
            if (i % 4 == 0) {
                motors.get(i).setPower(clipPower(flPower));
            } else if (i % 4 == 1) {
                motors.get(i).setPower(clipPower(frPower));
            } else if (i % 4 == 2) {
                motors.get(i).setPower(clipPower(blPower));
            } else if (i % 4 == 3) {
                motors.get(i).setPower(clipPower(brPower));
            }
        }
    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void telemetry() {
        telemetry.addData("DriveTrain","MecanumODrive");
        telemetry.addData("Left Front Pwr", motors.get(0).getPower());
        telemetry.addData("Right Front Pwr", motors.get(1).getPower());
        telemetry.addData("Left Back Pwr", motors.get(2).getPower());
        telemetry.addData("Rigth Back Pwr", motors.get(3).getPower());
    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void moveForward(double velocity, double revolutions) {
        super.moveForward(velocity, revolutions);
    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }
}
