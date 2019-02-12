package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GoldPosition.*;

/**
 * Updated by Alex on 11/5/2017.
 */
@Autonomous(name = "DepotAuto", group = "default")
@Disabled
public class DepotAuto extends WoBuZhiDaoOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    double revsTraveledFromLastState = 0.0;

    public DepotAuto() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveTrain.syncOpMode(gamepad1, telemetry, hardwareMap);
        driveTrain.setDrivePwrMax(DRIVE_PWR_MAX);
        driveTrain.setMotors();
        telemetry.addData(">", "Drive Train Initialization Successful");
        //armMotor = hardwareMap.dcMotor.get("arm");
        //armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //bodyMotor = hardwareMap.dcMotor.get("body");
        //bodyMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData(">", "Mineral Intake Initialization Successful");
        //Initialize servos
        //clawArm = hardwareMap.servo.get("ca");
        //Initialize Sensors
        latchMotor = hardwareMap.dcMotor.get("lm");
        markerServo = hardwareMap.servo.get("ms");
        sleighMotor = hardwareMap.dcMotor.get("sm");
        sleighMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Sleigh Intake Initialization Successful");
        driveTrain.initializeIMU();
        initializeDogeforia();
        telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }

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
                markerServo.setPosition(START_MARK_POS);
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            /*case 2:
                stateGoal = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                latchMotor.setPower(LATCH_PWR);

                if (waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    state = 1000;
                }
                break;*/

            case 2:
                stateName = "Move Toward Depot - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(0.4, 0.5);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 4:
                stateName = "Turn to have phone face sampling field - Turn 90 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolute(90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

                case 6: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateName = "Scan for Gold Mineral - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(-0.10, 6);


                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    if (Math.abs(driveTrain.getRevolutionsDriven()) <= 0.5) {
                        goldPosition = CENTER;
                    }
                    else {
                        goldPosition = LEFT;
                    }
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    state = 1000; //skip alignment step for gold in right position
                }
                else if (driveTrain.encoderTargetReached) { //if gold not found within 75 degree ccw turn, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = RIGHT;
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    state = 1000;
                }
                break;


            case 8: //Drive Forward to face gold cube
                stateName = "Align to Cube in Right Position - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (driveTrain.getRevolutionsDriven() > revsTraveledFromLastState) {
                    driveTrain.moveForward(0.80); //turn quickly until heading is 25 degrees cw of initial position
                }
                else {
                    driveTrain.moveForward(0.10, 4);
                }

                if (goldAligned()) {
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                else if (driveTrain.encoderTargetReached) { //if gold not found, STOP at 75 degrees cw of initial position
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;


            case 10:
                stateName = "Turn to have front of robot face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolute(0);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 12: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                double targetRevolutions = 0;
                switch (goldPosition) {
                    case LEFT:
                    case RIGHT: targetRevolutions = 3.75;
                        break;
                    case CENTER: targetRevolutions = 3.25;
                        break;
                }
                driveTrain.moveForward(0.80, targetRevolutions);
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 14:
                stateName = "Rotate 45 degrees counter-clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)

                int targetAngle = 0;
                switch (goldPosition) {
                    case CENTER:
                    case RIGHT: targetAngle = 135;
                        break;
                    case LEFT: targetAngle = 45;
                        break;
                }
                driveTrain.turnAbsolute(targetAngle);
                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 16:
                stateName = "Drive Backward to Wall";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case CENTER:
                    case RIGHT: driveTrain.moveForward(-0.8, -0.75);
                        break;
                    case LEFT: driveTrain.moveForward(0.8, 0.75);
                        break;
                }

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    if (goldPosition == LEFT) {
                        state += 3;
                    }
                    else {
                        state++;
                    }
                }
                break;



            case 18:
                stateName = "Rotate 135 degrees clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnAbsolute(135);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 20:
                stateName = "Drop Marker";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                markerServo.setPosition(MAX_MARKER_POS);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    markerServo.setPosition(MIN_MARKER_POS);
                    state++;
                }
                break;

            case 22:
                stateName = "Drive Forward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.8, 8.0);

                if (driveTrain.encoderTargetReached || waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    driveTrain.runConstantSpeed();
                    state = 1000;
                }

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