package  org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TankOp", group = "Default")
//@Disabled
public class TankOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor FR;
    DcMotor FL;

    DcMotor Arm;

    Servo ClawR;
    Servo ClawL;
    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double DRIVE_PWR_MAX = 0.8;
    double currentLeftPwr = 0;
    double currentRightPwr = 0;

    final double ARM_PWR_MAX = 0.6;
    double currentArmPwr = 0;

    final double clawDelta = 0.001;
    final double CLAW_ARM_START_POS_R = 0.5;
    final double CLAW_ARM_START_POS_L = 0.5;
    double clawArmPosR = CLAW_ARM_START_POS_R;
    double clawArmPosL = CLAW_ARM_START_POS_L;


    public TankOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        telemetry.addData(">", "Drive Train Initialization Successful");
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL = hardwareMap.dcMotor.get("fl");
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        Arm = hardwareMap.dcMotor.get("arm");
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize servos
        ClawR = hardwareMap.servo.get("CR");
        ClawL = hardwareMap.servo.get("CL");

        //Initialize sensors

        telemetry.addData(">", "Press Start to continue");
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

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        currentLeftPwr = Range.clip(currentLeftPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        FL.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        FR.setPower(currentRightPwr);

        currentArmPwr = Range.clip(currentArmPwr,-ARM_PWR_MAX,ARM_PWR_MAX);
        Arm.setPower(currentArmPwr);

        clawArmPosR = Range.clip(clawArmPosR,0.5,1);
        ClawR.setPosition(clawArmPosR);
        clawArmPosL = Range.clip(clawArmPosL,0,0.5);
        ClawL.setPosition(clawArmPosL);



    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR_PWR",FR.getPower());
        telemetry.addData("FL_PWR",FL.getPower());

        telemetry.addData("ARM_PWR",Arm.getPower());

        telemetry.addData("CR_POS",ClawR.getPosition());
        telemetry.addData("CL_POS",ClawL.getPosition());
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateTankDrive();
        updateArm();
        updateClawArm();
    }
    void updateTankDrive() {
        currentLeftPwr = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        currentRightPwr = -gamepad1.right_stick_y * DRIVE_PWR_MAX;
    }
    void updateArm(){
        currentArmPwr = - gamepad1.right_trigger * ARM_PWR_MAX + gamepad1.left_trigger * ARM_PWR_MAX;
    }
    void updateClawArm(){
        if(gamepad1.right_bumper) {
            clawArmPosR -= clawDelta;
            clawArmPosL += clawDelta;
        }
        if(gamepad1.left_bumper) {
            clawArmPosR += clawDelta;
            clawArmPosL -= clawDelta;
        }
    }

    //Create Methods that will update the driver data

    /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
    */


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateGoal = ""; //Overwrite this as the specific step used in Autonomous

    void advanceState() {
        state++;
        setTime = this.time;
    }

    void advanceState(int skipState) {
        state += (skipState * 2) + 1;
        setTime = this.time;
    }

    void resetEncoders() {

    }
    void runConstantSpeed() {

    }
    void runConstantPower() {

    }
    void calibrateAutoVariables() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}



