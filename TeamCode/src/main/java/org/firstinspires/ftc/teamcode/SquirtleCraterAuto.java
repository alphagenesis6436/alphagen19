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
 * Updated by Alex on 1/7/2019.
 */
@Autonomous(name = "SquirtleCraterAuto", group = "default")
//@Disabled
public class SquirtleCraterAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    String goldPos = "NOT FOUND";
    boolean latchReset = false;

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

        // Make sure that the sound files exist on the phone
        bandSoundID = hardwareMap.appContext.getResources().getIdentifier("band", "raw", hardwareMap.appContext.getPackageName());
        marchSoundID   = hardwareMap.appContext.getResources().getIdentifier("march",   "raw", hardwareMap.appContext.getPackageName());
        weowSoundID   = hardwareMap.appContext.getResources().getIdentifier("weow",   "raw", hardwareMap.appContext.getPackageName());
        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (bandSoundID != 0)
            bandFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, bandSoundID);

        if (marchSoundID != 0)
            marchFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, marchSoundID);
        if (weowSoundID != 0)
            weowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, weowSoundID);

        // Display sound status
        telemetry.addData("band sound",   bandFound ?   "Found" : "NOT found\n Add band.mp3 to /src/main/res/raw" );
        telemetry.addData("march sound", marchFound ? "Found" : "NOT found\n Add march.mp3 to /src/main/res/raw"  );
        telemetry.addData("WEOW sound", weowFound ? "Found" : "NOT found\n Add weow.mp3 to /src/main/res/raw"  );

        telemetry.addData(">", "Press Start to continue");
    }

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
                stateName = "Initial Calibration";
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                scoringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                scoringMotor.setPower(0);
                latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendLatch(LATCH_PWR, 18.7);

                if (latchValueReached) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    state++;
                }
                break;

            case 4:
                stateName = "Turn to have hook unlatch - Turn 15 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(15);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, weowSoundID);
                    telemetry.addData("Playing", "WEEOOOOWWW");
                    state++;
                }
                break;

            case 6:
                stateName = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(-0.9, -1);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 8:
                stateName = "Turn to original angle";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(0);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 10:
                stateName = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(-0.9, -0.25);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 12:
                stateName = "Turn to have phone face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(-90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 14: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateName = "Scan for Gold Mineral - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(-0.25, -1.75);

                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    goldPosition = (Math.abs(driveTrain.getRevolutionsDriven()) <= 0.75) ? CENTER : LEFT;
                    state += 3; //skip alignment step for gold in right position
                }
                else if (driveTrain.encoderTargetReached) { //if gold not found within 1.5 revolutions, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = RIGHT;
                    state++;
                }
                break;


            case 16: //Drive Forward to face gold cube
                stateName = "Align to Cube in Right Position - Drive Forward (SKIP UNLESS RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(0.90, 2.55);
                }
                else {
                    driveTrain.moveForward(0, 0);
                }


                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;


            case 18:
                stateName = "Turn to have front of robot face sampling field - Turn 90 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                if (goldPosition == CENTER) {
                    driveTrain.turnClockwise(-5);
                }
                else {
                    driveTrain.turnClockwise(0);
                }

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 20:
                stateName = "Move Toward Lander and Bring Intake Down - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(0.9, 0.45);
                setTiltServos(TILT_MIN);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 22: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(-0.90, -1.3);
                intakeMotor.setPower(-0.50);

                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    intakeMotor.setPower(0);
                    state++;
                }
                break;

            case 24: //Park in Crater
                stateName = "Park in Crater - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                setTiltServos(TILT_SCORE - 0.20);
                driveTrain.moveForward(-0.90, -0.75);
                extendIntake(0.2);
                if (driveTrain.encoderTargetReached) {
                    extendIntake(0);
                    setTiltServos(TILT_MIN + 0.03);
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