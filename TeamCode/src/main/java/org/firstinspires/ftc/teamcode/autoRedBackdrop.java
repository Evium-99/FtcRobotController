package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Red Backdrop v27")
public class autoRedBackdrop extends LinearOpMode {
    public static double DISTANCE_FROM_BACKBOARD = 6;
    public static double armP = 0.002;
    public static double armI = 0;
    public static double armD = 0.2;
    public double integralSummation = 0;
    public double lastError = 0;
    public static double position = -270;
    public static double positionb = -10;
    public static double W = -1;
    public static double Z = 1;
    public static double Public_Angle = 90;
    public static double centerOffset = 15 * Z;
    public static double CenterOffsetDeg = 40 * Z;
    public static double rightOffset = 3 * Z;
    public static double rightOffsetDeg = -10 * Z;
    public static double leftOffset = -7 * Z;
    public static double leftOffsetDeg = 23 * W;
    private double angleZ = Public_Angle * Z;
    public static double distanceForward = 29;
    public static double backFromPixel = 5;
    public static double strafeRight = 10 * Z;
    public static double strafeRightB = 8 * Z;
    public static double toBoardFromCenter = 30;
    public static double toBoardFromLeft = 30;
    public static double dontHitTheRigging = 20;
    public static double toBoardFromRight = 15;
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
    public String side = "Left";
    ElapsedTime timer = new ElapsedTime();
    boolean stopPower = false;
    boolean stopPowerb = false;
    boolean toBoard = false;

    @Override
    public void runOpMode() throws InterruptedException {
//        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

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
        gripServo2 = hardwareMap.get(Servo.class, "grip1");
        gripServo1 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        shoota = hardwareMap.get(Servo.class, "shooter");

        // Control Hub I2C
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.FORWARD);
        rightHex.setDirection(DcMotorEx.Direction.REVERSE);

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
        TrajectorySequence strafeOutOfWay = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(25)
                .build();

        // Set Trajectory 1 to go forward
        TrajectorySequence trajectory66 = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .build();

        // Set turnAngleW to turn W
        TrajectorySequence turnAngleW = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-75))
                .build();

        // Set turnAngleZ to turn Z
        TrajectorySequence turnAngleZ = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angleZ))
                .build();

        // Set turnAngleZ to turn Z
        TrajectorySequence turnAngleCenter = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRight)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3b = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRightB)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3dba = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3dbb = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3bb = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(strafeRightB)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3c = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(5)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3cb = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
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
                .turn(Math.toRadians(160))
                .build();

        // Set 180Traj to go forward to board
        TrajectorySequence Traj170 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-185))
                .build();

        // Crazy Spline
        TrajectorySequence CrazyStrafea = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-25, 20)) // Forward Right
                .build();

        // Crazy Spline
        TrajectorySequence CrazyStrafeb = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-18, -15))
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
        sleep(500);

        // If senses center, update telemetry and set side
        if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
            side = "Center";
            // Update Telemetry
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Pos", leftHex.getCurrentPosition());
            telemetry.update();
            // Place Pixel then turn back and go to board
            //strafe right
            drive.followTrajectorySequence(trajectory3b);
            sleep(700);

            //move arm down
            armServo.setPosition(0);
            sleep(1100);

            //open purple side
            gripServo2.setPosition(1);
            sleep(700);

            //back up
            drive.followTrajectorySequence(BackFromPixel);
            sleep(700);

            //bring arm up
            armServo.setPosition(0.6);

            //turn towards board
            drive.followTrajectorySequence(turnAngleCenter); // Turn Left
            drive.followTrajectorySequence(toBoardFromCenterTraj);
//            drive.followTrajectorySequence(CenterBoardTrajB);
        } else {
            // Turn Right
            drive.followTrajectorySequence(turnAngleW);

            // Wait for Robot to settle
            sleep(1000);

            // If senses right, update telemetry and set side
            if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
                side = "Right";
                telemetry.addData("Side", side);
                telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("Pos", leftHex.getCurrentPosition());
                telemetry.update();

                // Move to Right
                drive.followTrajectorySequence(trajectory3dba);
                sleep(700);

                // If Right drop Pixel
                armServo.setPosition(0);
                sleep(1100);
                drive.followTrajectorySequence(trajectory3dbb);
                sleep(700);
                gripServo2.setPosition(1);
                sleep(700);
                drive.followTrajectorySequence(BackFromPixel);
                sleep(500);
                armServo.setPosition(0.6);
                // Turn 180
                drive.followTrajectorySequence(Traj170);
                drive.followTrajectorySequence(CrazyStrafea);
                drive.followTrajectorySequence(CrazyStrafeb);
                // Center on Right side of Board RightBoardTrajB
//                drive.followTrajectorySequence(RightBoardTrajB);
            } else {
                // If Left 180; Wait, Drop Pixel, wait; 180
                drive.followTrajectorySequence(Traj180);
                drive.followTrajectorySequence(trajectory3c);
                sleep(700);
                armServo.setPosition(0);
                sleep(1100);
                drive.followTrajectorySequence(trajectory3cb);
                gripServo2.setPosition(1);
                sleep(700);
                drive.followTrajectorySequence(BackFromPixel);
                sleep(700);
                armServo.setPosition(0.6);
                sleep(700);
                drive.followTrajectorySequence(LeftBoardTrajA);
                drive.followTrajectorySequence(toBoardFromLeftTraj);
            }
        }

        // Move toward the Board
        while(!toBoard && opModeIsActive()){

            // Set Power to Motors Based on Log Func
            double power = -0.50 * ((0.6 * Math.log10(distanceBack.getDistance(DistanceUnit.INCH) - DISTANCE_FROM_BACKBOARD)) + 0.2);

            // Set Distance to Board Var for Easy Access
            double distance_to_board = distanceBack.getDistance(DistanceUnit.INCH);

            // Check if within tolerable distance to board
            if (distance_to_board < (DISTANCE_FROM_BACKBOARD + 0.4)) {
                // If tolerable, break out of loop
                toBoard = true;
                break;
            }

            if (power < 0) {
                motor1.setPower(power);
                motor2.setPower(power);
                motor3.setPower(power);
                motor4.setPower(power);
            }

            // Telemetry
            telemetry.addData("Distance (inch)", distance_to_board);
            telemetry.addData("Pos", leftHex.getCurrentPosition());
            telemetry.update();
        }

        // Stop Moving Base
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        // move arm towards board
        while(!stopPower && opModeIsActive()){
            // Set Variables for easy access
            double power = pidController(position, leftHex.getCurrentPosition());
            double arm_position = leftHex.getCurrentPosition();

            // Check if within tolerable arm position
            if ((arm_position < (position + 30)) && (arm_position > (position - 30))) {
                // Break out of loop if tolerable position
                stopPower = true;
                break;
            }

            // Set power to hex motors
            leftHex.setPower(power);
            rightHex.setPower(power);
        }

        // Stop arm
        leftHex.setPower(0);
        rightHex.setPower(0);
        sleep(1000);

        // drop pixels
        gripServo1.setPosition(0);

        sleep(1000);

        // Move Arm back down
        while (opModeIsActive() && !stopPowerb) {
            telemetry.addData("Status", "Moving Arm Back Down...");
            telemetry.addData("Current Pos", leftHex.getCurrentPosition());
            telemetry.update();
            double power = pidController(positionb, leftHex.getCurrentPosition());
            if (((positionb + 30) > leftHex.getCurrentPosition()) && (leftHex.getCurrentPosition() > positionb - 30)) {
                stopPowerb = true;
            }
            leftHex.setPower(power);
            rightHex.setPower(power);
        }

        // Stop Arm Again
        leftHex.setPower(0);
        rightHex.setPower(0);

        // Put wrist down
        armServo.setPosition(0);

        // Strafe Left 20
        drive.followTrajectorySequence(strafeOutOfWay);
    }
    public double pidController(double reference, double state) {
        // Calculate error
        double error = reference - state;
        // Based on error add to the Integral Summation
        integralSummation += error * timer.seconds();
        // Calculate the Derivative of the error
        double derivative = (error - lastError) / timer.seconds();
        // Set the last error to the current error
        lastError = error;
        // Return power output based off of the PID Constants
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
