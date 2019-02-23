package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "SquirtleDepotAuto", group = "default")
//@Disabled
public class SquirtleDepotAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    String goldPos = "NOT FOUND";
    boolean latchReset = false;

    public SquirtleDepotAuto() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveTrain.syncOpMode(gamepad1, telemetry, hardwareMap);
        driveTrain.setDrivePwrMax(DRIVE_PWR_MAX);
        driveTrain.setMotors();
        telemetry.addData(">", "Drive Train Initialization Successful");

        latchMotor = hardwareMap.dcMotor.get("lm");
        telemetry.addData(">", "Latch Initialization Successful");

        scoringMotor = hardwareMap.dcMotor.get("sm");
        scoringMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Scoring Mechanism Initialization Successful");

        intakeMotor = hardwareMap.dcMotor.get("im");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Intake Initialization Successful");

        extenderMotor1 = hardwareMap.dcMotor.get("em1");
        extenderMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotor2 = hardwareMap.dcMotor.get("em2");
        extenderMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Extender Initialization Successful");

        driveTrain.initializeIMU();
        initializeDogeforia();
        telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData("" + state, stateGoal);
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
        telemetry.addLine();

        //retract latch
        if (state > 4) {
            if (!latchReset) {
                extendLatch(-LATCH_PWR, 0);
            }
            else {
                latchMotor.setPower(0);
            }
            if (latchValueReached) {
                latchReset = true;
            }
        }

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateGoal = "Initial Calibration";
                calibrateHardwareDevices();
                calibrateAutoVariables();
                resetEncoders();
                advanceState();
                break;

            case 2:
                stateGoal = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendLatch(LATCH_PWR, LATCH_HEIGHT);

                if (latchValueReached) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    advanceState();
                }
                break;

            case 4:
                stateGoal = "Turn to have hook unlatch - Turn 15 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnRelativePID(15);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 6:
                stateGoal = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(-0.90, -1.1);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 8:
                stateGoal = "Turn to original angle";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnAbsolutePID(0);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 10:
                stateGoal = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(-0.90, -0.4);
                extendIntake(0.9);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 12:
                stateGoal = "Extend Intake";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendIntake(0.9);

                if (waitSec(0.8)) { //Use a boolean value that reads true when state goal is completed
                    extendIntake(0);
                    advanceState();
                }
                break;

            case 14:
                stateGoal = "Drop Marker - Rotate intake down";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                setTiltServos(TILT_MIN);
                if (waitSec(0.55)) {
                    intakeMotor.setPower(0.90);
                }

                if (waitSec(1.25)) { //Use a boolean value that reads true when state goal is completed
                   advanceState();
                }
                break;

            case 16:
                stateGoal = "Rotate intake up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                setTiltServos(TILT_MAX);
                extendIntake(-0.4);

                if (waitSec(0.05)) { //Use a boolean value that reads true when state goal is completed
                    intakeMotor.setPower(0);
                    advanceState();
                }
                break;

            case 18:
                stateGoal = "Retract Intake";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendIntake(-0.8);
                driveTrain.moveForward(0.90, 0.30); //AND back up

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    extendIntake(0);
                    advanceState();
                }
                break;

            case 20:
                stateGoal = "Turn to have phone ALMOST face RIGHT Sampling Mineral - Turn 30 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnRelativePID(-25);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 22: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateGoal = "Scan for Gold Mineral - Turn to Absolute -90 degrees";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                //turn at 20% power when robot is in range of RIGHT or CENTER mineral
                if ((driveTrain.getHeading() <= -25 && driveTrain.getHeading() >= -50) || (driveTrain.getHeading() <= -65 && driveTrain.getHeading() >= -105)) {
                    driveTrain.turnClockwise(-0.15);
                }
                //turn at 90% power when robot is NOT in range of a mineral
                else {
                    driveTrain.turnClockwise(-0.90);
                }


                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    goldPosition = (driveTrain.getHeading() >= -65) ? RIGHT : CENTER;

                    advanceState(); //skip alignment step for gold in right position
                }
                else if (driveTrain.getHeading() <= -105) { //if gold not found within 1.5 revolutions, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = LEFT;
                    advanceState();
                }
                break;


            case 24: //Drive Forward to face gold cube
                stateGoal = "Align to Cube - Turn Appropriately";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT: driveTrain.turnAbsolutePID(-37);
                        break;
                    case CENTER: driveTrain.turnAbsolutePID(0);
                        break;
                    case RIGHT: driveTrain.turnAbsolutePID(37);
                        break;
                }

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;


            case 26:
                stateGoal = "Move Toward Lander and Bring Intake Down - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.9, 0.45);
                setTiltServos(TILT_MIN);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 28: //Knock off cube - Drive Forward
                stateGoal = "Collect cube - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(-0.90, -1.3);
                intakeMotor.setPower(-0.60);

                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    intakeMotor.setPower(0);
                    state++;
                }
                break;

            case 30: //Knock off cube - Drive Forward
                stateGoal = "Retreat With Mineral - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.90, 1.3);
                setTiltServos(TILT_MAX);
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 32:
                stateGoal = "Robot Face Crater - Turn to -105 degrees";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnAbsolutePID(-105);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;


            case 34:
                /*stateGoal = "Drive Backward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantPower();
                double targetTime = (goldPosition == LEFT)? 3 : 3.5;
                driveTrain.moveForward(-0.9);
                extendIntake(waitSec(1) ? 0 : 0.6);
                setTiltServos(TILT_SCORE);


                if (waitSec(targetTime)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }*/
                stateGoal = "Park in Crater - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                setTiltServos(TILT_SCORE - 0.20);
                driveTrain.moveForward(-0.90, -2.2);
                extendIntake(0.4);
                if (driveTrain.encoderTargetReached) {
                    extendIntake(0);
                    setTiltServos(TILT_MIN + 0.03);
                    driveTrain.stopDriveMotors();
                    state++;
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