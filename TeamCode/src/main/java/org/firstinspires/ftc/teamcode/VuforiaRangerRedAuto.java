package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Updated by Alex on 12/7/2018.
 */
@Autonomous(name = "VuforiaRangerRedAuto", group = "default")
//@Disabled
public class VuforiaRangerRedAuto extends VuforiaRangerOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = GoldPosition.NOT_FOUND;
    String goldPos = "NOT FOUND";

    public VuforiaRangerRedAuto() {}

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));
        telemetry.addData("Gold Position", goldPos);
        switch (goldPosition) {
            case NOT_FOUND: goldPos = "NOT FOUND";
                break;
            case LEFT: goldPos = "LEFT";
                break;
            case CENTER: goldPos = "CENTER";
                break;
            case RIGHT: goldPos = "RIGHT";
                break;
        }

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2: //Search for Gold Mineral by Turning CCW and using GoldMineralDetector
                stateName = "Scan for Gold Mineral - Turn CCW";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                encoderMode = EncoderMode.CONSTANT_SPEED;
                turnClockwise(-0.10);


                if (goldAligned()) { //if gold found, then it's either center or left position
                    stopDriveMotors();
                    if (Math.abs(getHeading()) <= 15) {
                        goldPosition = GoldPosition.CENTER;
                    }
                    else {
                        goldPosition = GoldPosition.LEFT;
                    }
                    state = 5; //skip alignment step for gold in right position
                }
                else if (getHeading() < -25) { //if gold not found within 75 degree ccw turn, then cube is right position
                    stopDriveMotors();
                    goldPosition = GoldPosition.RIGHT;
                    state++;
                }
                break;


            case 4: //Turn Clockwise to face gold cube
                stateName = "Align to Cube in Right Position - Turn CW";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                encoderMode = EncoderMode.CONSTANT_SPEED;
               if (getHeading() < 0) {
                   turnClockwise(0.80); //turn quickly until heading is 25 degrees cw of initial position
               }
               else {
                   turnClockwise(0.10);
               }

                if (goldAligned()) {
                    stopDriveMotors();
                    state++;
                }
                else if (getHeading() > 40) { //if gold not found, STOP at 75 degrees cw of initial position
                    stopDriveMotors();
                    state++;
                }
                break;

            case 6: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                encoderMode = EncoderMode.CONSTANT_SPEED;
                double targetRevolutions = 0;
                switch (goldPosition) {
                    case LEFT:
                    case RIGHT: targetRevolutions = 3.75;
                        break;
                    case CENTER: targetRevolutions = 3.25;
                        break;
                }
                moveForward(0.80, targetRevolutions);
                if (encoderTargetReached) {
                    stopDriveMotors();
                    state++;
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                calibrateAutoVariables();
                resetEncoders();
                if (waitSec(1)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}
enum GoldPosition {
    NOT_FOUND, LEFT, CENTER, RIGHT;
}