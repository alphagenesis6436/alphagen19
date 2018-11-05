package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

    /**
     * Updated by Alex on 6/1/2017.
     */

    @TeleOp(name = "MichaelOp", group = "Default")
    @Disabled
    public class MichaelOp extends OpMode {
        //Declare any motors, servos, and sensors
        DcMotor leftMotor;
        DcMotor rightMotor;
        Servo clawArm; //180

        //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
        final double DRIVE_MAX_PWR = 0.80;
        double currentLeftPwr = 0.0;
        double currentRightPwr = 0.0;
        final double CLAW_ARM_START_POS = 0.5;
        double clawArmPosition = CLAW_ARM_START_POS;
        double clawDelta = 0.001;
        final double CLAW_MAX = 1.0;
        final double CLAW_MIN = 0.0;


        public MichaelOp() {}

        @Override public void init() {
            //Initialize motors & set direction
            rightMotor = hardwareMap.dcMotor.get("rm");
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor = hardwareMap.dcMotor.get("lm");
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            //Initialize servos
            clawArm = hardwareMap.servo.get("ca");
            //Initialize sensors

            telemetry();
        }
        @Override public void loop() {
            //Update all the data based on driver input
            updateData();

            /* Clip Variables to make sure they don't exceed their
             * ranged values and Set them to the Motors/Servos */
            initialization();

            //Show the Real Values of the Data Using Telemetry
            telemetry();
        }

        void updateData() {
            //Add in update methods for specific robot mechanisms
            updateClaw();
            updateDriveTrain();
        }

        void initialization() {
            //Clip and Initialize Specific Robot Mechanisms
            currentLeftPwr = Range.clip(currentLeftPwr,-DRIVE_MAX_PWR,DRIVE_MAX_PWR);
            leftMotor.setPower(currentLeftPwr);
            currentRightPwr = Range.clip(currentLeftPwr,-DRIVE_MAX_PWR,DRIVE_MAX_PWR);
            rightMotor.setPower(currentRightPwr);
            clawArmPosition = Range.clip(clawArmPosition,CLAW_MIN,CLAW_MAX);
            clawArm.setPosition(clawArmPosition);
        }
        void telemetry() {
            //Show Data for Specific Robot Mechanisms
            telemetry.addData("RM Pwr",rightMotor.getPower());
            telemetry.addData("LM Pwr",leftMotor.getPower());
            telemetry.addData("Claw Pos",clawArm.getPosition());

        }

        //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1:  Press the up/down button to move the clawservo towards 180/0
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */
        void updateClaw(){
            if (gamepad1.dpad_up) {
                clawArmPosition += clawDelta;
            }
            else if(gamepad1.dpad_down){
                clawArmPosition -= clawDelta;
            }
        }

        //Create variables/methods that will be used in ALL autonomous programs for this specific robot

        double setTime; //used to measure the time period of each step in autonomous
        int state = 0; //used to control the steps taken during autonomous
        String stateName = ""; //Overwrite this as the specific step used in Autonomous

        void resetEncoders() {

        }
        void runConstantSpeed() {

        }
        void runConstantPower() {

        }
        void updateDriveTrain() {
            currentLeftPwr = -gamepad1.left_stick_y * DRIVE_MAX_PWR;
            currentRightPwr = -gamepad1.right_stick_y * DRIVE_MAX_PWR;
        }
        void resetSensors() {

        }
        //used to measure the amount of time passed since a new step in autonomous has started
        boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    }


