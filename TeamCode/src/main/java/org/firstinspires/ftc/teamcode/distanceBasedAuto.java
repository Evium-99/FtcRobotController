package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Distance Based Auto")
public class distanceBasedAuto extends LinearOpMode {
    public static double DISTANCE_FROM_BACKBOARD = 6;
    public static double armP = 0.004;
    public static double armI = 0;
    public static double armD = 0.4;
    public double integralSummation = 0;
    public double lastError = 0;
    public static double position = -220;
    public static double positionb = -10;
    public static double W = -1;
    public static double Z = 1;
    public static double Public_Angle = 90;
    public static double centerOffset = 7 * Z;
    public static double CenterOffsetDeg = 23 * Z;
    public static double rightOffset = 3 * Z;
    public static double rightOffsetDeg = 5 * Z;
    public static double leftOffset = 7 * Z;
    public static double leftOffsetDeg = 23 * W;
    private double angleW = Public_Angle * W;
    private double angleZ = Public_Angle * Z;
    public static double distanceForward = 28;
    public static double backFromPixel = 5;
    public static double strafeRight = 10;
    public static double strafeRightB = 8 * Z;
    public static double strafeRightC = 3 * Z;
    public static double toBoardFromCenter = 30;
    public static double toBoardFromLeft = 30;
    public static double dontHitTheRigging = 20;
    public static double toBoardFromRight = 20;
    public static double Angle180 = 180;
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private DcMotorEx winchHex = null; // Winch Hex
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servo
    private Servo shoota = null; // Shooter Servo
    private DistanceSensor distanceBack = null; // Distance sensor on the back
    private DistanceSensor distanceFront = null; // Distance Sensor Front
    private DistanceSensor distanceFrontLeft = null; // Distance Sensor Front Left
    private TouchSensor armHit = null; // Arm Touch Sensor
    public String side = "Left";
    ElapsedTime timer = new ElapsedTime();
    boolean stopPower = false;
    boolean stopPowerb = false;
    boolean toBoard = false;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        // Control Hub Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");
        winchHex = hardwareMap.get(DcMotorEx.class, "rightEncoder");

        // Control Hub Servos
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        shoota = hardwareMap.get(Servo.class, "shooter");

        // Control Hub I2C
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceFrontLeft = hardwareMap.get(DistanceSensor.class, "frontLeft");
        armHit = hardwareMap.get(TouchSensor.class, "armHit");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.REVERSE);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set Drive Constants
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Hex Mode
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Pose estimate to 0, 0, 0 for initialization purposes
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Set Trajectory 1 to go forward
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(distanceForward)
                .build();

        // Set Trajectory 1 to go forward
        TrajectorySequence trajectory66 = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .build();

        // Set turnAngleW to turn W
        TrajectorySequence turnAngleW = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angleW))
                .build();

        // Set turnAngleZ to turn Z
        TrajectorySequence turnAngleZ = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angleZ))
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRight)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3b = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRightB)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3c = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRightC)
                .build();

        // Set toBoardFromCenterTraj to go back to board
        TrajectorySequence toBoardFromCenterTraj = drive.trajectorySequenceBuilder(startPose)
                .back(toBoardFromCenter)
                .build();

        // Set toBoardFromLeftTraj to go back to board
        TrajectorySequence toBoardFromLeftTraj = drive.trajectorySequenceBuilder(startPose)
                .back(toBoardFromLeft)
                .build();

        // Set dontHitTheRiggingTraj to got forward dontHitTheRigging
        TrajectorySequence dontHitTheRiggingTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(dontHitTheRigging)
                .build();

        // Set toBoardFromRightTraj to go back to board
        TrajectorySequence toBoardFromRightTraj = drive.trajectorySequenceBuilder(startPose)
                .back(toBoardFromRight)
                .build();

        // Set 180Traj to go forward to board
        TrajectorySequence Traj180 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(Angle180))
                .build();

        // Set toPixelforLeft to back backFromPixel
        TrajectorySequence toPixelforLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(backFromPixel)
                .build();

        // Set BackFromPixel to back backFromPixel
        TrajectorySequence BackFromPixel = drive.trajectorySequenceBuilder(startPose)
                .back(backFromPixel)
                .build();

        // Set CenterBoardTrajA to Center
        TrajectorySequence CenterBoardTrajA = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(CenterOffsetDeg))
                .build();

        // Set CenterBoardTrajB to Center
        TrajectorySequence CenterBoardTrajB = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(centerOffset)
                .build();
        // Sat RightBoardTrajA to Right Side of Board
        TrajectorySequence RightBoardTrajA = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(rightOffsetDeg))
                .build();

        // Sat RightBoardTrajB to Right Side of Board
        TrajectorySequence RightBoardTrajB = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(rightOffset)
                .build();

        // Set LeftBoardTrajA to Right Side of Board
        TrajectorySequence LeftBoardTrajA = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(leftOffsetDeg))
                .build();

        // Set LeftBoardTrajB to Right Side of Board
        TrajectorySequence LeftBoardTrajB = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(leftOffset)
                .build();

        // State initialization sequence is completed
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pos", leftHex.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        side = "Left";
        waitForStart();
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Squeeze Pixels
        gripServo1.setPosition(1);
        gripServo2.setPosition(0);

        // Update Telemetry
        telemetry.addData("Side", side);
        telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Pos", leftHex.getCurrentPosition());
        telemetry.update();

        // Move Away from Wall
        drive.followTrajectorySequence(trajectory1);

        // Wait for Robot to settle
        sleep(1000);

        // If senses center, update telemetry and set side
        if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
            side = "Center";
            // Update Telemetry
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Pos", leftHex.getCurrentPosition());
            telemetry.update();
            // Place Pixel then turn back and go to board
            drive.followTrajectorySequence(trajectory3b);
            sleep(700);
            armServo.setPosition(0);
            sleep(1100);
            gripServo2.setPosition(1);
            sleep(700);
            drive.followTrajectorySequence(BackFromPixel);
            sleep(700);
            armServo.setPosition(0.7);
            sleep(700);
            drive.followTrajectorySequence(turnAngleZ);
            drive.followTrajectorySequence(toBoardFromCenterTraj);
            drive.followTrajectorySequence(CenterBoardTrajA);
            drive.followTrajectorySequence(CenterBoardTrajB);
        } else {
            // Turn Right
            drive.followTrajectorySequence(turnAngleW);

            // Wait for Robot to settle
            sleep(1000);

            // If senses right, update telemetry and set side
            if (distanceFrontLeft.getDistance(DistanceUnit.INCH) < 10) {
                side = "Right";
                telemetry.addData("Side", side);
                telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("Pos", leftHex.getCurrentPosition());
                telemetry.update();

                // If Right drop Pixel
                armServo.setPosition(0);
                sleep(1100);
                gripServo2.setPosition(1);
                sleep(700);
                drive.followTrajectorySequence(BackFromPixel);
                sleep(700);
                armServo.setPosition(0.7);
                sleep(700);
                drive.followTrajectorySequence(trajectory66);

                // Strafe Right to Avoid Pixel
                drive.followTrajectorySequence(trajectory3);
                // #dontHitTheRiggingTraj
                drive.followTrajectorySequence(dontHitTheRiggingTraj);
                // Turn 180
                drive.followTrajectorySequence(Traj180);
                drive.followTrajectorySequence(RightBoardTrajA);
                // Go back to Board
                drive.followTrajectorySequence(toBoardFromRightTraj);
                // Center on Right side of Board RightBoardTrajB
                drive.followTrajectorySequence(RightBoardTrajB);
            } else {
                // If Left 180; Wait, Drop Pixel, wait; 180
                drive.followTrajectorySequence(Traj180);
                drive.followTrajectorySequence(trajectory3c);
                sleep(700);
                armServo.setPosition(0);
                sleep(1100);
                gripServo2.setPosition(1);
                sleep(700);
                drive.followTrajectorySequence(BackFromPixel);
                sleep(700);
                armServo.setPosition(0.7);
                sleep(700);
                drive.followTrajectorySequence(LeftBoardTrajA);
                drive.followTrajectorySequence(toBoardFromLeftTraj);
                drive.followTrajectorySequence(LeftBoardTrajB);
            }
        }
        // End Place Purple Pixel Sequence -->  Start Place Yellow Pixel Sequence

        // End Place Yellow Pixel Sequence

        // Read Data to Screen
        while (opModeIsActive()) {
            if (!toBoard) {
                double power = -0.75*((0.6*Math.log10(distanceBack.getDistance(DistanceUnit.INCH)-DISTANCE_FROM_BACKBOARD))+0.2);
                if ((distanceBack.getDistance(DistanceUnit.INCH) > (DISTANCE_FROM_BACKBOARD-0.4)) && (distanceBack.getDistance(DistanceUnit.INCH) < (DISTANCE_FROM_BACKBOARD+0.4))) {
                    toBoard = true;
                } else if (distanceBack.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_BACKBOARD) {
                    motor1.setPower(0.1);
                    motor2.setPower(0.1);
                    motor3.setPower(0.1);
                    motor4.setPower(0.1);
                } else if (power < 0) {
                    motor1.setPower(power);
                    motor2.setPower(power);
                    motor3.setPower(power);
                    motor4.setPower(power);
                }
                telemetry.addData("Distance (inch)", distanceBack.getDistance(DistanceUnit.INCH));
                telemetry.addData("Pos", leftHex.getCurrentPosition());
                telemetry.update();
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
                if (!stopPower) {
                    telemetry.addData("Status", "Putting Pixel on Board");
                    telemetry.addData("Pos", leftHex.getCurrentPosition());
                    telemetry.update();
                    double power = pidController(position, leftHex.getCurrentPosition());
                    if (((position + 30) > leftHex.getCurrentPosition()) && (leftHex.getCurrentPosition() > position - 30)) {
                        stopPower = true;
                    }
                    leftHex.setPower(power);
                    rightHex.setPower(power);
                } else {
                    leftHex.setPower(0);
                    rightHex.setPower(0);
                    gripServo1.setPosition(0);
                    if (!stopPowerb) {
                        telemetry.addData("Status", "Main Loop");
                        telemetry.update();
                        double power = pidController(positionb, leftHex.getCurrentPosition());
                        if (((positionb + 30) > leftHex.getCurrentPosition()) && (leftHex.getCurrentPosition() > positionb - 30)) {
                            stopPowerb = true;
                        }
                        leftHex.setPower(power);
                        rightHex.setPower(power);
                    } else {
                        leftHex.setPower(0);
                        rightHex.setPower(0);
                    }

                }
                telemetry.addData("Current Pos", leftHex.getCurrentPosition());
                telemetry.update();
            }
        }
    }
    public double pidController(double reference, double state) {
        double error = reference - state;
        integralSummation += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
