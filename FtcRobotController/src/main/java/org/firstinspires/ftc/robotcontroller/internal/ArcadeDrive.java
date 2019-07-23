package org.firstinspires.ftc.robotcontroller.internal;

public class ArcadeDrive extends ClassicDrive {
    @Override
    public void update() {
        for (int i = 0; i < numOfMotors; i++) {
            double power = drivePwrMax;
            if (i % 2 == 0)
                power *= -gamepad1.left_stick_y + gamepad1.right_stick_x;
            else
                power *= -gamepad1.left_stick_y - gamepad1.right_stick_x;
            motors.get(i).setPower(clipPower(power));
        }
    }
}
