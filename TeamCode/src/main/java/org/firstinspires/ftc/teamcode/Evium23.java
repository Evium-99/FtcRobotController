package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Evium 2023 - TeleOP 1", group="Linear Opmode")
public class Evium23 extends LinearOpMode {

    // Define Cords to Power Function

    static double cordToPower(float controllerX, float controllerY) {
        return (Math.sqrt((Math.pow(controllerX, 2) + Math.pow(controllerY, 2)))/1.42);
    }

    // Define Cords to Deg Func

    static double cordToDeg(float controllerX, float controllerY) {
        double val = Math.abs(Math.asin((controllerY / (Math.sqrt(Math.pow(controllerX, 2) + Math.pow(controllerY, 2))))) * (180 / Math.PI));
        if (controllerX > 0 && controllerY > 0) {
            return val;
        } else if (controllerX < 0 && controllerY > 0) {
            return (90 - val) + 90;
        } else if (controllerX < 0 && controllerY < 0) {
            return val + 180;
        } else if (controllerY < 0 && controllerX > 0) {
            return (90 - val) + 270;
        }
        return -1;
    }

    // Declare OpMode members.

    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Change The Left side to Backwards
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftStickDegrees = cordToDeg(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double power = cordToPower(gamepad1.left_stick_x, gamepad1.left_stick_y);

            // if (gamepad1.left_trigger)

            if ((0 < leftStickDegrees && leftStickDegrees < 45) || (leftStickDegrees == 45)) {
                // *B
                double percentageRight = (45 - leftStickDegrees)/45;
                motor1.setPower(power*percentageRight);
                motor2.setPower(power);
                motor3.setPower(power*percentageRight);
                motor4.setPower(power);
            } else if ((45 < leftStickDegrees && leftStickDegrees < 90) || (leftStickDegrees == 90)) {
                // *A
                double percentageForwardFrom45 = (leftStickDegrees - 45)/45;
                motor1.setPower(power);
                motor2.setPower(power*percentageForwardFrom45);
                motor3.setPower(power);
                motor4.setPower(power*percentageForwardFrom45);
            } else if ((90 < leftStickDegrees && leftStickDegrees < 135) || (leftStickDegrees == 135)) {
                // *H
                double inversePercentage135From90 = 1 - ((leftStickDegrees - 90)/45);
                motor1.setPower(power);
                motor2.setPower(power*inversePercentage135From90);
                motor3.setPower(power);
                motor4.setPower(power*inversePercentage135From90);
            } else if ((135 < leftStickDegrees && leftStickDegrees < 180) || (leftStickDegrees == 180)) {
                // *G
                double percentage180From135 = (leftStickDegrees - 135)/45;
                motor1.setPower(power);
                motor2.setPower(-power*percentage180From135);
                motor3.setPower(power);
                motor4.setPower(-power*percentage180From135);
            } else if ((180 < leftStickDegrees && leftStickDegrees < 225) || (leftStickDegrees == 225)) {
                // F
                double inversePercentage225From180 = 1 - ((leftStickDegrees - 180)/45);
                motor1.setPower(power*inversePercentage225From180);
                motor2.setPower(-power);
                motor3.setPower(power*inversePercentage225From180);
                motor4.setPower(-power);
            } else if ((225 < leftStickDegrees && leftStickDegrees < 270) || (leftStickDegrees == 270)) {
                // E
                double percentage270From225 = (leftStickDegrees - 225)/45;
                motor1.setPower(-power*percentage270From225);
                motor2.setPower(-power);
                motor3.setPower(-power*percentage270From225);
                motor4.setPower(-power);
            } else if ((270 < leftStickDegrees && leftStickDegrees < 315) || (leftStickDegrees == 315)) {
                // D
                double inversePercentage315From270 = 1 - ((leftStickDegrees - 270)/45);
                motor1.setPower(-power);
                motor2.setPower(-power*inversePercentage315From270);
                motor3.setPower(-power);
                motor4.setPower(-power*inversePercentage315From270);
            } else if ((315 < leftStickDegrees && leftStickDegrees < 360) || (leftStickDegrees == 360) || (leftStickDegrees == 0)) {
                // C
                double percentage360From315 = (leftStickDegrees - 315)/45;
                motor1.setPower(-power);
                motor2.setPower(power*percentage360From315);
                motor3.setPower(-power);
                motor4.setPower(power*percentage360From315);
            }

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
