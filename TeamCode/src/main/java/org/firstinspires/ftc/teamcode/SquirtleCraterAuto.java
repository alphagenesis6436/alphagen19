package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GoldPosition.CENTER;
import static org.firstinspires.ftc.teamcode.GoldPosition.LEFT;
import static org.firstinspires.ftc.teamcode.GoldPosition.NOT_FOUND;
import static org.firstinspires.ftc.teamcode.GoldPosition.RIGHT;

/**
 * Updated by Alex on 1/7/2019.
 */
@Autonomous(name = "SquirtleCraterAuto", group = "default")
//@Disabled
public class SquirtleCraterAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    double revsTraveledFromLastState = 0.0;

    public SquirtleCraterAuto() {}

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

        intakeServo = hardwareMap.servo.get("is");
        intakeServo.setDirection(Servo.Direction.REVERSE);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Intake Initialization Successful");

        extenderMotor1 = hardwareMap.dcMotor.get("em1");
        extenderMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
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
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                scoringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                scoringMotor.setPower(0);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            /*case 2:
                stateName = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                latchMotor.setPower(LATCH_PWR);

                if (waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    state = 1000;
                }
                break;*/

            case 2:
                stateName = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(-0.9, -1.0);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 4:
                stateName = "Turn to have phone face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(-90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 6: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateName = "Scan for Gold Mineral - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(-0.35, -1.5);

                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    goldPosition = (Math.abs(driveTrain.getRevolutionsDriven()) <= 1) ? CENTER : LEFT;
                    revsTraveledFromLastState = driveTrain.getRevolutionsDriven();
                    state += 3; //skip alignment step for gold in right position
                }
                else if (driveTrain.encoderTargetReached) { //if gold not found within 1.5 revolutions, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = RIGHT;
                    state++;
                }
                break;


            case 8: //Drive Forward to face gold cube
                stateName = "Align to Cube in Right Position - Drive Forward (SKIP UNLESS RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(0.90, 1.75);
                }
                else {
                    driveTrain.moveForward(0, 0);
                }


                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;


            case 10:
                stateName = "Turn to have front of robot face sampling field - Turn 90 cw (30 ccw if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                double targetAngle = (goldPosition == RIGHT)? -120 : 0;
                driveTrain.turnClockwise(targetAngle);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 12: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Backward (Forward if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(0.90, 1.5);
                }
                else {
                    driveTrain.moveForward(-0.90, -0.9);
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 14: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Forward (Backward if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(-0.90, -1.5);
                }
                else {
                    driveTrain.moveForward(0.90, 0.9);
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 16:
                stateName = "Turn to have phone face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(-90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;


            case 18:
                stateName = "Drive Backward To Align with Depot Wall";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                double targetRevolutions = (goldPosition == RIGHT)? -5.75 : -4.0;
                driveTrain.moveForward(-0.9, targetRevolutions);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }

            case 20:
                stateName = "Turn to Parallel with Wall - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(-135);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
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
                if (waitSec(0.01)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}