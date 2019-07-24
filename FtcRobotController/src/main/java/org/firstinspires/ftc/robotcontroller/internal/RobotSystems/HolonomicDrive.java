package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class HolonomicDrive extends OmnidirectionalDrive {

    private double flPower, frPower, blPower, brPower;

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
        if (!(gamepad1.right_stick_x < 0.03 && gamepad1.right_stick_x > -0.03)) { //Move Clockwise/Anticlockwise
            flPower = gamepad1.right_stick_x * drivePwrMax * turnPwrMax;
            frPower = -gamepad1.right_stick_x * drivePwrMax * turnPwrMax;
            blPower = gamepad1.right_stick_x * drivePwrMax * turnPwrMax;
            brPower = -gamepad1.right_stick_x * drivePwrMax * turnPwrMax;
        }
        else if (!(gamepad1.left_stick_y < 0.10 && gamepad1.left_stick_y > -0.10) &&
                !(gamepad1.left_stick_x < 0.10 && gamepad1.left_stick_x > -0.10)) { //Move at diagonal
            if ((gamepad1.left_stick_x > 0 && gamepad1.left_stick_y < 0) ||
                    (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y > 0)) { //ForwardRight/BackwardLeft
                flPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                frPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                blPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                brPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                if (gamepad1.left_stick_x < 0) {
                    flPower *= -1;
                    brPower *= -1;
                }
            }
            else { //ForwardLeft/BackwardRight
                flPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                frPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                blPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                brPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                if (gamepad1.left_stick_x > 0) {
                    frPower *= -1;
                    blPower *= -1;
                }
            }
        }
        else if (!(gamepad1.left_stick_y < 0.10 && gamepad1.left_stick_y > -0.10)) { //Move forward/backward
            flPower = -gamepad1.left_stick_y * drivePwrMax;
            frPower = -gamepad1.left_stick_y * drivePwrMax;
            blPower = -gamepad1.left_stick_y * drivePwrMax;
            brPower = -gamepad1.left_stick_y * drivePwrMax;
        }
        else if (!(gamepad1.left_stick_x < 0.10 && gamepad1.left_stick_x > -0.10)) { //Move Right/left
            flPower = gamepad1.left_stick_x * drivePwrMax;
            frPower = -gamepad1.left_stick_x * drivePwrMax;
            blPower = -gamepad1.left_stick_x * drivePwrMax;
            brPower = gamepad1.left_stick_x * drivePwrMax;
        }
        else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) { //Stop Motion
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }
        motors.get(0).setPower(clipPower(flPower));
        motors.get(1).setPower(clipPower(frPower));
        motors.get(2).setPower(clipPower(blPower));
        motors.get(3).setPower(clipPower(brPower));
    }

    @Override
    public void telemetry() {

    }
}
