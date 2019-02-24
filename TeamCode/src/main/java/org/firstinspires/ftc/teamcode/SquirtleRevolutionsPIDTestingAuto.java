package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Updated by Alex on 2/11/2019.
 */
@Autonomous(name = "SquirtleRevolutionsPIDTestingAuto", group = "default")
//@Disabled
public class SquirtleRevolutionsPIDTestingAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    double drivingTime = 0;
    double targetDistance = 0;

    public SquirtleRevolutionsPIDTestingAuto() {}

    @Override
    public void loop(){
        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateGoal = "Initial Calibration";
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                setRevolutionPIDConstants();
                calibrateAutoVariables();
                resetEncoders();
                advanceState();
                break;

            case 2:
                stateGoal = "Drive Forward 2 inches (0.16 revolutions)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                targetDistance = 2.0;
                driveTrain.moveForwardPID(driveTrain.convertDistance2Rev(targetDistance));

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    drivingTime = this.time - setTime;
                    advanceState();
                }
                break;

            case 4:
                stateGoal = "Wait for the A button to be pressed";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                telemetry.addData("Driving Time",String.format("%.2f", drivingTime));
                if (gamepad1.a) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 6:
                stateGoal = "Drive Forward 12 inches (0.95 revolutions)";
                targetDistance = 12.0;
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForwardPID(driveTrain.convertDistance2Rev(targetDistance));

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    drivingTime = this.time - setTime;
                    advanceState();
                }
                break;


            case 8:
                stateGoal = "Wait for the A button to be pressed";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                telemetry.addData("Driving Time",String.format("%.2f", drivingTime));
                if (gamepad1.a) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 10:
                stateGoal = "Drive Forward 20 inches (1.59 revolutions)";
                targetDistance = 20.0;
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForwardPID(driveTrain.convertDistance2Rev(targetDistance));

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    drivingTime = this.time - setTime;
                    advanceState();
                }
                break;


            case 12:
                stateGoal = "Wait for the A button to be pressed";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                telemetry.addData("Driving Time",String.format("%.2f", drivingTime));
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