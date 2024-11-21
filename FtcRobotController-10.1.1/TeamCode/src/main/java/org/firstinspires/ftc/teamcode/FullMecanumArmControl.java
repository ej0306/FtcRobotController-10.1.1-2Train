package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FullMecanumArmControl", group="FTC")
public class FullMecanumArmControl extends OpMode {

    // Mecanum drivetrain motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    // Arm, wrist, and intake components
    private DcMotor armMotor = null;  // Motor for arm movement
    private Servo wristServo = null; // Servo for wrist control
    private Servo intakeServo = null; // Continuous rotation servo for intake

    // Arm setpoints (in degrees of rotation)
    private final double ARM_HOME = 0;          // Home position
    private final double ARM_LOW = 30;         // Low position
    private final double ARM_MIDDLE = 60;      // Middle position
    private final double ARM_HIGH = 90;        // High position

    // Gear ratio and motor properties
    private final double TICKS_PER_REV = 145.1; // Encoder ticks per motor revolution
    private final double GEAR_RATIO = 254.47;   // Total gear reduction
    private final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    @Override
    public void init() {
        // Initialize drivetrain hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        // Initialize arm, wrist, and intake hardware
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");

        // Configure drivetrain motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Configure arm motor
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Safe default mode

        // Set initial servo positions
        wristServo.setPosition(0.5); // Neutral wrist position
        intakeServo.setPosition(0.5); // Stop intake

        telemetry.addData("Status", "Initialized");
    }

    private void moveArmToPosition(double degrees) {
        // Calculate the target position in encoder ticks
        int targetPosition = (int) (degrees * TICKS_PER_DEGREE);

        // Set the target position
        armMotor.setTargetPosition(targetPosition);

        // Set the motor to RUN_TO_POSITION mode
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to move the arm
        armMotor.setPower(1.0); // Use a lower value if needed for smoother control
    }

    @Override
    public void loop() {
        // ----- Mecanum Drivetrain Control -----
        double drive = -gamepad1.left_stick_y; // Forward/backward (Y is reversed)
        double strafe = gamepad1.left_stick_x * 1.1; // Strafing (slightly adjusted for imperfections)
        double turn = gamepad1.right_stick_x; // Rotation

        // Calculate motor powers for mecanum drive
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double leftFrontPower = (drive + strafe + turn) / denominator;
        double leftRearPower = (drive - strafe + turn) / denominator;
        double rightFrontPower = (drive - strafe - turn) / denominator;
        double rightRearPower = (drive + strafe - turn) / denominator;

        // Set power to the drivetrain motors
        leftFrontDrive.setPower(leftFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightRearDrive.setPower(rightRearPower);

        // ----- Arm Control -----
        if (gamepad2.dpad_up) {
            moveArmToPosition(ARM_HIGH); // High position
        } else if (gamepad2.dpad_right) {
            moveArmToPosition(ARM_MIDDLE); // Middle position
        } else if (gamepad2.dpad_down) {
            moveArmToPosition(ARM_LOW); // Low position
        } else if (gamepad2.dpad_left) {
            moveArmToPosition(ARM_HOME); // Home position
        }

        // Stop the motor if it's no longer busy
        if (!armMotor.isBusy()) {
            armMotor.setPower(0); // Stop the motor
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Return to normal mode
        }

        // ----- Telemetry -----
        telemetry.addData("Drive (Y)", drive);
        telemetry.addData("Strafe (X)", strafe);
        telemetry.addData("Turn (RX)", turn);
        telemetry.addData("Arm Target", armMotor.getTargetPosition());
        telemetry.addData("Arm Current", armMotor.getCurrentPosition());
        telemetry.addData("Arm Busy", armMotor.isBusy());
        telemetry.update();
    }
}
