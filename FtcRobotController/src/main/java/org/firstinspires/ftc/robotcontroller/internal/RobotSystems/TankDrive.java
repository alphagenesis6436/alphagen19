package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class TankDrive extends ClassicDrive {
    @Override
    public void update() {
        for (int i = 0; i < numOfMotors; i++) {
            double power = drivePwrMax;
            if (i % 2 == 0)
                power *= -gamepad1.left_stick_y;
            else
                power *= -gamepad1.right_stick_y;
            motors.get(i).setPower(clipPower(power));
        }
    }
}
