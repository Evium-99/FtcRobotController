package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;

class tfClientsideRecognition {
    // found, sideRecognised, x, y, confidance
    // If sideRecognised = Center and x>250 then on right spike line else Center Spike Line
    // If sideRecognised = Right
    boolean found = false;
    String sideRecognised;
    int x;
    int y;
    float confidance;
}

@Disabled
@Autonomous(name="FullAutoProgram 2.0.17")
public class FullAutoProgram extends LinearOpMode {
    public int SEARCH_TIMEOUT_CYCLES = 2;
    public int VISION_SEARCH_CYCLES = 100;
    public int SEARCH_ANGLE = 30;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red & Blue Prop.tflite";
    private static final String[] LABELS = {
            "Blue Prop",
            "Red Prop"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    static final HashMap<Integer, Pose2d> aprils = new HashMap<Integer, Pose2d>();
    static {
        aprils.put(1, new Pose2d(-42.83, 63.5, Math.toRadians(90)));
        aprils.put(2, new Pose2d(-36.83, 63.5, Math.toRadians(90)));
        aprils.put(3, new Pose2d(-30.83, 63.5, Math.toRadians(90)));
        aprils.put(4, new Pose2d(30.83, 63.5, Math.toRadians(90)));
        aprils.put(5, new Pose2d(36.83, 63.5, Math.toRadians(90)));
        aprils.put(6, new Pose2d(42.83, 63.5, Math.toRadians(90)));
        aprils.put(7, new Pose2d(43, -72, Math.toRadians(270)));
        aprils.put(8, new Pose2d(37.5, -72, Math.toRadians(270)));
        aprils.put(9, new Pose2d(-37.5, -72, Math.toRadians(270)));
        aprils.put(10, new Pose2d(-43, -72, Math.toRadians(270)));
    }

    static final HashMap<String, Vector2d> positions = new HashMap<String, Vector2d>();

    static {
        positions.put("p1-1", new Vector2d(30.36, 43.72));
        positions.put("p1-2", new Vector2d(36, 43.72));
        positions.put("p1-3", new Vector2d(42.69, 43.72));
        positions.put("p1-4", new Vector2d(36, 12));
        positions.put("p1-5", new Vector2d(63.62, 44.91));

        positions.put("p2-1", new Vector2d(-41.83, 43.72));
        positions.put("p2-2", new Vector2d(-36, 43.72));
        positions.put("p2-3", new Vector2d(-29.42, 43.72));
        positions.put("p2-4", new Vector2d(-36, 12));
        positions.put("p2-5", new Vector2d(-61.61, 44.91));

        positions.put("p3-1", new Vector2d(-36, -60));
        positions.put("p3-2", new Vector2d(-24, -60));
        positions.put("p3-3", new Vector2d(-12, -60));
        positions.put("p3-4", new Vector2d(-36, -36));
        positions.put("p3-5", new Vector2d(-61.61, -44.16));

        positions.put("p4-1", new Vector2d(12, -60));
        positions.put("p4-2", new Vector2d(24, -60));
        positions.put("p4-3", new Vector2d(36, -60));
        positions.put("p4-4", new Vector2d(36, -36));
        positions.put("p4-5", new Vector2d(63.62, -45.06));

    }

    private WebcamName webcam1, webcam2;
    private AprilTagProcessor aprilTag;

    // Declare OpMode members.
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private Servo armServo = null; // Arm Servo
    private DistanceSensor distanceRight = null; // Distance Sensor
    private DistanceSensor distanceLeft = null; // Distance Sensor
    private ElapsedTime runtime = new ElapsedTime();

    private Pose2d startPose;

    @Override
    public void runOpMode() {

        initVision();

        // Control Hub Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");


        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");

        // Control Hub Servos
        armServo = hardwareMap.get(Servo.class, "arm");
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");

        // Control Hub I2C

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotor.Direction.FORWARD);
        rightHex.setDirection(DcMotor.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startPose = localize();
        if (startPose == null) {
            startPose = new Pose2d(-60, -40, Math.toRadians(90));
        }

        drive.setPoseEstimate(startPose);

        TrajectorySequence moveUpABit = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Move From Wall");
                    telemetry.update();
                    // Send Power to Grip Pixel
                    gripServo1.setPosition(1);
                    gripServo2.setPosition(-1);
                    armServo.setPosition(1);
                })
                .forward(12)
                .build();

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Go To Left Spike Mark");
                    telemetry.update();
                })
                .forward(18)
                .turn(1.7)
                .forward(8)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Go To Right Spike Mark");
                    telemetry.update();
                })
                .forward(18)
                .turn(-1.6)
                .back(7)
                .build();

        TrajectorySequence backSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Go To Back Spike Mark");
                    telemetry.update();
                })
                .forward(17)
                .turn(0.45)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move Away from Wall
        drive.followTrajectorySequence(moveUpABit);

        // Look For Team Prop by looking left and right
        findTeamProp(SEARCH_TIMEOUT_CYCLES, VISION_SEARCH_CYCLES, SEARCH_ANGLE, drive);
        drive.followTrajectorySequence(backSpikeMark); // Assume Left Spike Mark
        while (opModeIsActive()) {
            gripServo2.setPosition(1); // Open Purple Side
            telemetry.addData("Status", "Done");
        }

        visionPortal.close();
    }

    private float[] lookForProp(int cycles) {

        for (int i = 0; i < cycles; i++) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                return new float[] {recognition.getConfidence(), (float) x, (float) y};
            }
        }
        return new float[0];
    }

    private tfClientsideRecognition findTeamProp(int cyclesTimeout, int visionCycles, int angle, SampleMecanumDrive drive) {
        boolean found = false;
        float[] tempPropRecognitionStore = lookForProp(visionCycles);
        tfClientsideRecognition propDataStore = null;
        TrajectorySequence lookLeft = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Looking For Prop: CCW");
                    telemetry.update();
                })
                .turn(Math.toRadians(angle)) // Turns 15 degrees counter-clockwise
                .build();

        TrajectorySequence lookRight = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Looking For Prop: CW");
                    telemetry.update();
                })
                .turn(Math.toRadians(-angle)) // Turns 15 degrees clockwise
                .build();
        for (int i = 0; i < cyclesTimeout; i++) {
            drive.followTrajectorySequence(lookLeft); // Looking Left
            tempPropRecognitionStore = lookForProp(visionCycles);
            if (tempPropRecognitionStore.length > 0) {
                propDataStore = new tfClientsideRecognition();
                propDataStore.found = true;
                propDataStore.confidance = tempPropRecognitionStore[0];
                propDataStore.x = (int) tempPropRecognitionStore[1];
                propDataStore.y = (int) tempPropRecognitionStore[2];
                found = true;
            }
            drive.followTrajectorySequence(lookLeft); // Looking Left Again
            tempPropRecognitionStore = lookForProp(visionCycles);
            if (tempPropRecognitionStore.length > 0) {
                propDataStore = new tfClientsideRecognition();
                propDataStore.found = true;
                propDataStore.confidance = tempPropRecognitionStore[0];
                propDataStore.x = (int) tempPropRecognitionStore[1];
                propDataStore.y = (int) tempPropRecognitionStore[2];
                found = true;
            }
            drive.followTrajectorySequence(lookRight); // Back to Slightly Left
            tempPropRecognitionStore = lookForProp(visionCycles);
            if (tempPropRecognitionStore.length > 0) {
                propDataStore = new tfClientsideRecognition();
                propDataStore.found = true;
                propDataStore.confidance = tempPropRecognitionStore[0];
                propDataStore.x = (int) tempPropRecognitionStore[1];
                propDataStore.y = (int) tempPropRecognitionStore[2];
                found = true;
            }
            drive.followTrajectorySequence(lookRight); // Back to Center
            tempPropRecognitionStore = lookForProp(visionCycles);
            if (tempPropRecognitionStore.length > 0) {
                propDataStore = new tfClientsideRecognition();
                propDataStore.found = true;
                propDataStore.confidance = tempPropRecognitionStore[0];
                propDataStore.x = (int) tempPropRecognitionStore[1];
                propDataStore.y = (int) tempPropRecognitionStore[2];
                found = true;
            }
            if (found) {
                return propDataStore;
            }
        }
        return new tfClientsideRecognition();
    }

    private void initVision() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(1280, 720));

        builder.enableLiveView(true);

        // Create Processor for April Tags
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Set and enable the processor for tfod and apriltags.
        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.72f);
    }

    private Pose2d aprilToBotLocation(int tagID, double botRelationX, double botRelationY, double yaw) {
        Pose2d tag = aprils.get(tagID);
        if(tag == null){
            return startPose;
        }
        return new Pose2d(tag.getX() - botRelationX, tag.getY() - botRelationY, tag.getHeading() - yaw);
    }

    private Pose2d localize() {
        Pose2d latestPose = null;
        ElapsedTime localizationRuntime = new ElapsedTime();
        localizationRuntime.reset();
        localizationRuntime.startTime();
        while (localizationRuntime.milliseconds()/1000 < 3) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) { // detection.ftcPose.x, detection.ftcPose.y,
                    latestPose = aprilToBotLocation(detection.id, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
                }
            }
        }
        localizationRuntime.reset();
        return latestPose;
    }

}
