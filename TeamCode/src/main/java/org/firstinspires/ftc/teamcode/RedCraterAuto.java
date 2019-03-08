package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GoldPosition.CENTER;
import static org.firstinspires.ftc.teamcode.GoldPosition.LEFT;
import static org.firstinspires.ftc.teamcode.GoldPosition.NOT_FOUND;
import static org.firstinspires.ftc.teamcode.GoldPosition.RIGHT;

/**
 * Updated by Alex on 2/11/2019.
 */
@Autonomous(name = "RedCraterAuto", group = "default")
//@Disabled
public class RedCraterAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    String goldPos = "NOT FOUND";
    boolean latchReset = false;

    public RedCraterAuto() {}

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
                driveTrain.moveForward(-0.90, -1.15);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 8:
                stateGoal = "Scan for Gold Mineral in RIGHT Position - Turn to Absolute -90 degrees";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                if (goldPosition == NOT_FOUND) {
                    //turn at 15% power when robot is in range of RIGHT mineral
                    if (driveTrain.inHeadingRange(-65, -10)) {
                        driveTrain.turnClockwise(-0.11);
                        //if gold found, then it's in right or center position
                        if (goldAligned() && driveTrain.inHeadingRange(-60, -15)) {
                            goldPosition = RIGHT;
                        }
                    }
                    //turn at 90% power when robot is NOT in range of a mineral
                    else {
                        driveTrain.turnAbsolutePID(-80);
                    }

                }
                else {
                    driveTrain.turnAbsolutePID(-80);
                }

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    if (goldPosition == RIGHT) {
                        advanceState(1);
                    }
                    else {
                        advanceState();
                    }
                }
                break;

            case 10: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateGoal = "Scan for Gold Mineral in CENTER Position & Drive to Wall - Drive Backwards";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                if (goldPosition == NOT_FOUND) {
                    //drive at 15% power when robot is in range of CENTER mineral
                    driveTrain.moveForward(-0.15);

                    //if gold found, then it's CENTER position
                    if (goldAligned()) {
                        goldPosition = CENTER;
                    }
                    //if gold not found within 4.5 inches, then cube is left position
                    else if (driveTrain.getDistance() <= -5) {
                        goldPosition = LEFT;
                        driveTrain.encoderTargetReached = true;
                    }
                }
                else { //stop in front of center mineral
                    driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(5)); //used to be 49
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 12: //Drive Forward to face gold cube
                stateGoal = "Align to Cube - Turn Appropriately";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT: driveTrain.turnAbsolutePID(-51);
                        break;
                    case CENTER: driveTrain.turnAbsolutePID(0);
                        break;
                    case RIGHT: driveTrain.turnAbsolutePID(-135);
                        break;
                }

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 14: //Knock off cube - Drive Forward
                stateGoal = "Knock off cube - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT: driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(17));
                        break;
                    case CENTER: driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(10));
                        break;
                    case RIGHT: driveTrain.moveForward(0.90, driveTrain.convertDistance2Rev(14));
                        break;
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    if (goldPosition == LEFT) {
                        advanceState(1);
                    }
                    else {
                        advanceState();
                    }
                }
                break;

            case 16: //Retreat From Mineral - Drive Forward"
                stateGoal = "Retreat From Mineral - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case CENTER: driveTrain.moveForward(0.90, driveTrain.convertDistance2Rev(9));
                        break;
                    case RIGHT: driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(13));
                        break;
                }

                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;


            case 18: //Turn to face Depot Wall
                stateGoal = "Turn to Face Depot Wall - Turn 45 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnAbsolute(-90);

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 20:
                stateGoal = "Drive to Depot Wall - Drive Backwards";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(25));
                        break;
                    case CENTER:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(47.5));
                        break;
                    case RIGHT:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(48));
                        break;
                }

                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 22: //Turn to face Depot Wall
                stateGoal = "Turn to Face Depot - Turn 45 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnRelativePID(-39);

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;


            case 24:
                stateGoal = "Drive to Depot & Lower Intake - Drive Backwards";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(32));
                        break;
                    case CENTER:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(19));
                        break;
                    case RIGHT:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(22));
                        break;
                }
                setTiltServos(TILT_MIN + 0.10);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 26:
                stateGoal = "Drop Marker - Spin Intake Out";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                setTiltServos(TILT_MIN);
                intakeMotor.setPower(0.50);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    advanceState();
                }
                break;

            case 28:
                stateGoal = "Drive Away from Depot - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.90, driveTrain.convertDistance2Rev(24));
                setTiltServos(TILT_MAX);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    intakeMotor.setPower(0);
                    advanceState();
                }
                break;

            case 30: //Turn to face Depot
                stateGoal = "Turn to Face Crater - Turn 180 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnAbsolutePID(30);

                if (driveTrain.angleTargetReached) {
                    driveTrain.stopDriveMotors();
                    advanceState();
                }
                break;

            case 32:
                stateGoal = "Park in Crater - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                switch (goldPosition) {
                    case LEFT: driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(24.5));
                        break;
                    case CENTER:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(24.5));
                        break;
                    case RIGHT:  driveTrain.moveForward(-0.90, -driveTrain.convertDistance2Rev(24));
                        break;
                }
                if (driveTrain.encoderTargetReached) {
                    setTiltServos(TILT_MIN + 0.03);
                    driveTrain.stopDriveMotors();
                    state = 1000;
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