package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Alex on 1/4/2018.
 */


@TeleOp(name = "SquirtleOp", group = "Default")
//@Disabled
public class SquirtleOp extends OpMode {
    //Declare any motors, servos, and sensors
    DriveTrain driveTrain = new DriveTrain(DriveMode.ARCADE, 2);
    DcMotor latchMotor; //40:1 AndyMark //encoder
    DcMotor extenderMotor1; // Rev Hex Core Motor //encoder
    DcMotor extenderMotor2; // Rev Hex Core Motor //encoder
    DcMotor scoringMotor; //40:1 AndyMark Neverest Motor //encoder
    DcMotor intakeMotor; //Rev Hex Core Motor
    Servo tiltServo1; //180 Rev Servo, left side
    Servo tiltServo2; //180 Rev Servo, right side

    //Declare any variables & constants pertaining to Scoring
    final double CUBE_PWR_MAX = 0.5;
    final double BALL_PWR_MAX = 0.5;
    double currentScoringPwr = 0.0;
    int scoringState = 0;
    final double COUNTS_PER_REV = 1120;


    //Declare any variables & constants pertaining to Intake
    final double INTAKE_PWR_MAX = 0.5;
    double currentIntakePwr = 0;
    final double TILT_MIN = 0.0; //Intake is Down
    final double TILT_SCORE = 0.45; //Intake ready to score
    final double TILT_MAX = 0.54; //Intake is Up
    final double TILT_START_POS = TILT_MAX;
    double currentTiltPos = TILT_START_POS;
    double tiltDelta = 0.01;
    boolean tiltDown = false;

    //Declare any variables & constants pertaining to Drive Train
    final double DRIVE_PWR_MAX = 0.90;

    //Declare any variables & constants pertaining to Latch System
    final double LATCH_PWR = 0.95;
    double currentLatchPwr = 0.0;

    //Declare any variables & constants pertaining to Extender System
    final double EXTENDER_PWR_MAX = 0.8;
    double currentExtendPwr = 0.0;

    //Vuforia Stuff
    //Elapsed time and measurement constants
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Detector object
    GoldAlignDetector detector;


    public SquirtleOp() {}

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

        //driveTrain.initializeIMU();
        //initializeDogeforia();
        //telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }
    @Override public void start() {
        runtime.reset();
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
        initializeDriveTrain();
        initializeIntake();
        initializeExtender();
        initializeScoring();
        initializeLatch();
    }

    void initializeScoring() {
        currentScoringPwr = Range.clip(currentScoringPwr, -CUBE_PWR_MAX, CUBE_PWR_MAX);
        scoringMotor.setPower(currentScoringPwr);
    }

    void initializeExtender() {
        currentExtendPwr = Range.clip(currentExtendPwr, -EXTENDER_PWR_MAX, EXTENDER_PWR_MAX);
        extenderMotor1.setPower(currentExtendPwr);
        extenderMotor2.setPower(currentExtendPwr);
    }

    void initializeIntake() {
        currentIntakePwr = Range.clip(currentIntakePwr, -INTAKE_PWR_MAX, INTAKE_PWR_MAX);
        intakeMotor.setPower(currentIntakePwr);
        currentTiltPos = Range.clip(currentTiltPos, TILT_MIN, TILT_MAX);
        tiltServo1.setPosition(currentTiltPos);
        tiltServo2.setPosition(currentTiltPos);
    }

    void initializeLatch() {
        currentLatchPwr = Range.clip(currentLatchPwr, -LATCH_PWR, LATCH_PWR);
        latchMotor.setPower(currentLatchPwr);
    }

    void initializeDriveTrain() {
        driveTrain.initialize();
    }

    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        //telemetryDriveTrain();
        //telemetryIntake();
        //telemetryExtender();
        //telemetryScoring();
        telemetryLatch();
    }

    void telemetryScoring() {
        telemetry.addData("SCORING", "TELEMETRY");
        telemetry.addData("Scoring Pwr", scoringMotor.getPower());
        telemetry.addLine();
    }

    void telemetryExtender() {
        telemetry.addData("EXTENDER", "TELEMETRY");
        telemetry.addData("Extender Pwr", extenderMotor1.getPower());
        telemetry.addLine();
    }

    void telemetryIntake() {
        telemetry.addData("INTAKE", "TELEMETRY");
        telemetry.addData(">>>Intake Pwr", intakeMotor.getPower());
        telemetry.addData(">>>Tilt1 Pos", tiltServo1.getPosition());
        telemetry.addData(">>>Tilt2 Pos", tiltServo2.getPosition());
        telemetry.addLine();
    }

    void telemetryLatch() {
        telemetry.addData("LATCH", "TELEMETRY");
        telemetry.addData("Latch Pwr", latchMotor.getPower());
        telemetry.addData("Latch Enc", String.format("%.3f", latchMotor.getCurrentPosition() / COUNTS_PER_REV));
        telemetry.addLine();
    }


    void telemetryDriveTrain() {
        driveTrain.telemetry();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateExtender();
        updateDriveTrain();
        updateLatch();
        updateIntake();
        updateScoring();
    }

    void updateScoring() {
        if (gamepad2.left_trigger > 0.05) { //bring scoring arm down
            currentScoringPwr = -gamepad2.left_trigger * (BALL_PWR_MAX);
        }
        else if (gamepad2.b) { //score balls
            currentScoringPwr = BALL_PWR_MAX;
        }
        else if (gamepad2.x) { //score cubes
            currentScoringPwr = CUBE_PWR_MAX;
        }
        else { currentScoringPwr = 0; }
    }

    void updateExtender() {
        currentExtendPwr = -gamepad2.left_stick_y * EXTENDER_PWR_MAX;
    }

    void  updateIntake() {
        currentIntakePwr = -gamepad2.right_stick_y * INTAKE_PWR_MAX;

        if (gamepad2.y) {
            currentTiltPos = TILT_MAX;
        }
        if (gamepad2.a) {
            currentTiltPos = TILT_SCORE;
        }
        else if (gamepad2.dpad_up) {
            currentTiltPos += tiltDelta;
        }
        else if (gamepad2.dpad_down) {
            currentTiltPos -= tiltDelta;
        }
    }

    void updateLatch() {
        if (gamepad1.left_trigger > 0.05) { //extends latch
            currentLatchPwr = gamepad1.left_trigger * LATCH_PWR;
        }
        else if (gamepad1.right_trigger > 0.05) { //retracts latch
            currentLatchPwr = -gamepad1.right_trigger * LATCH_PWR;
        }
        else { currentLatchPwr = 0; }
    }

    void updateDriveTrain() {
        driveTrain.update();
    }

    void extendIntake(double power) {
        extenderMotor1.setPower(power);
        extenderMotor2.setPower(power);
    }
    void setTiltServos(double pos) {
        pos = Range.clip(pos, TILT_MIN, TILT_MAX);
        tiltServo1.setPosition(pos);
        tiltServo2.setPosition(pos);
    }

    void extendIntake(double power, double revolutions) { //only use if attach encoders to extender
        double target = revolutions * COUNTS_PER_REV;
        double kp = 2 * (Math.abs(power) - 0.10) / COUNTS_PER_REV;
        double error = target - extenderMotor1.getCurrentPosition();
        if (!extenderValueReached) {
            if (Math.abs(error) <= COUNTS_PER_REV / 2) {
                power = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            extendIntake(power);
        }
        if (Math.abs(error) <= 4) {
            extendIntake(0);
            extenderValueReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REV));
        }
    }
    boolean extenderValueReached = false;

    void extendLatch(double power, double revolutions) { //only use if attach encoders to latch
        double target = revolutions * COUNTS_PER_REV;
        double kp = 2 * (Math.abs(power) - 0.10) / COUNTS_PER_REV;
        double error = target - latchMotor.getCurrentPosition();
        if (!latchValueReached) {
            if (Math.abs(error) <= COUNTS_PER_REV / 8) {
                power = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            latchMotor.setPower(power);
        }
        if (Math.abs(error) <= 4) {
            latchMotor.setPower(0);
            latchValueReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REV));
        }
    }
    boolean latchValueReached = false;

    //Create Methods that will update the driver data
    void initializeDogeforia() {
        // Setup camera and Vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Set Vuforia parameters
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.fillCameraMonitorViewParent = true;

        // Init Dogeforia
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Set target names
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        // Set trackables' location on field
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        //Set camera displacement
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        // Set phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        //Set info for the trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Activate targets
        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector(); // Create a gold aligndetector
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //Setup Vuforia
        vuforia.setDogeCVDetector(detector); // Set the Vuforia detector
        vuforia.enableDogeCV(); //Enable the DogeCV-Vuforia combo
        vuforia.showDebug(); // Show debug info
        vuforia.start(); // Start the detector
    }

    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {
        driveTrain.resetEncoders();
    }
    void calibrateAutoVariables() {
        driveTrain.encoderTargetReached = false;
        driveTrain.angleTargetReached = false;
        latchValueReached = false;
        extenderValueReached = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    boolean goldAligned() {
        boolean isAligned = false;
        int centerValue = 275;
        int uncertainty = 25;
        if (Math.abs(detector.getXPosition() - centerValue) <= uncertainty)
            isAligned = true;
        return isAligned;
    }

}



