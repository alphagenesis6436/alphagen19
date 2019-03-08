package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GoldPosition.CENTER;
import static org.firstinspires.ftc.teamcode.GoldPosition.LEFT;
import static org.firstinspires.ftc.teamcode.GoldPosition.NOT_FOUND;
import static org.firstinspires.ftc.teamcode.GoldPosition.RIGHT;

/**
 * Updated by Alex on 2/11/2019.
 */
@Autonomous(name = "SquirtleTurningTestingAuto", group = "default")
@Disabled
public class SquirtleTurningTestingAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    double turningTime = 0;

    public SquirtleTurningTestingAuto() {}

    @Override
    public void loop(){
        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateGoal = "Initial Calibration";
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                setAnglePIDConstants();
                calibrateAutoVariables();
                resetEncoders();
                advanceState();
                break;

            case 2:
                stateGoal = "Turn 15 degree clockwise";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnRelativePID(15);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    turningTime = this.time - setTime;
                    advanceState();
                }
                break;

            case 4:
                stateGoal = "Wait for the A button to be pressed";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                telemetry.addData("Turning Time",String.format("%.1f", turningTime));
                if (gamepad1.a) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;
            case 6:
                stateGoal = "Turn 90 degree counter clockwise";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnRelativePID(-90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    turningTime = this.time - setTime;
                    advanceState();
                }
                break;


            case 8:
                stateGoal = "Wait for the A button to be pressed";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                telemetry.addData("Turning Time",String.format("%.1f", turningTime));
                if (gamepad1.a) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;


            case 1000: //Run When Autonomous is Complete
                stateGoal = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateGoal = "Calibrating";
                calibrateAutoVariables();
                resetEncoders();
                if (waitSec(0.001)) {
                    advanceState();
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}