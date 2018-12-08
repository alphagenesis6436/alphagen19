package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Updated by Alex on 11/5/2017.
 */
@Autonomous(name = "DeathBoxLatchAuto", group = "default")
@Disabled
public class DeathBoxLatchAuto extends DeathBoxOp {
    //Declare and Initialize any variables needed for this specific autonomous program


    public DeathBoxLatchAuto() {}

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                Marker.setPosition(MAX_MARKER_POS);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Lower Robot to Ground";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                LA.setPower(LATCH_PWR);

                if (waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    LA.setPower(0);
                    state = 1000;
                }
                break;

            case 4:
                stateName = "Unlatch Hook From Lander";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                LatchServo.setPosition(0.5 + MAX_LATCH_SPEED);

                if (waitSec(3)) { //Use a boolean value that reads true when state goal is completed
                    LatchServo.setPosition(0.5);
                    state = 1000;
                }
                break;

            case 6:
                stateName = "Drive Forward to Depot";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                moveForward(0.8, 4.8);

                if (encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 8:
                stateName = "Rotate 45 degrees counter-clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                turnClockwise(-45);

                if (angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 10:
                stateName = "Drive Forward to Wall";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                moveForward(0.8, 0.75);

                if (encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 12:
                stateName = "Drop Marker";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                Marker.setPosition(MIN_MARKER_POS);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    Marker.setPosition(MAX_MARKER_POS);
                    state++;
                }
                break;

            case 14:
                stateName = "Drive Backward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                encoderMode = EncoderMode.CONSTANT_POWER;
                moveForward(-0.8, -8.0);

                if (encoderTargetReached || waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    encoderMode = EncoderMode.CONSTANT_SPEED;
                    state++;
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                stopDriveMotors();
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                calibrateAutoVariables();
                resetEncoders();
                if (waitSec(0.01)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}