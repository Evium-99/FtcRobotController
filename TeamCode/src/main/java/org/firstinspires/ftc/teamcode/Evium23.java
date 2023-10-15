package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Evium 2023 - TeleOP 2")
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
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motor1.setPower(frontRightPower);
            motor2.setPower(frontLeftPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
