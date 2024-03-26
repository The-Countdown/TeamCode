package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Positioning;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        while (opModeIsActive()) {

            // gamepad 1
            double turnR = 1500 * Math.pow(gamepad1.right_trigger, 3);
            double turnL = 1500 * Math.pow(gamepad1.left_trigger, 3);

            if (turnL < 200) {
                turnL = 0;
            }
            if (turnR < 200) {
                turnR = 0;
            }

//            if (gamepad1.options) {
//                telemetry.addData("robpos x: ", robot.robotPosition.position.x);
//                telemetry.addData("robpos y: ", robot.robotPosition.position.y);
//                telemetry.update();
//            }
            telemetry.addData("robpos x: ", robot.robotPosition.position.x);
            telemetry.addData("robpos y: ", robot.robotPosition.position.y);
            telemetry.update();

            if (gamepad1.right_bumper) {
                turnR = 800;
            }

            if (gamepad1.left_bumper) {
                turnL = 800;
            }

            double forwards = 2500 * Math.pow(-gamepad1.left_stick_y, 3);
            double sideways = 2500 * Math.pow(-gamepad1.left_stick_x, 3);

            if ((Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) && !gamepad1.right_stick_button) {
                forwards = 2500 * Math.pow(-gamepad1.right_stick_y, 3);
                sideways = 2500 * Math.pow(-gamepad1.right_stick_x, 3);

                if (gamepad1.left_stick_button) {
                    forwards *= 0.5;
                    sideways *= 0.5;
                }

                robot.drive.move(forwards, sideways, turnR, turnL);
            } else {
                if (gamepad1.right_stick_button) {
                    forwards *= 0.5;
                    sideways *= 0.5;
                }

                robot.drive.moveField(forwards, sideways, turnR, turnL);
            }

            // gamepad 2

            if (gamepad2.right_bumper) {
                robot.slide.move(2500);
            } else if (gamepad2.left_bumper) {
                robot.slide.move(-2500);
            } else {
                robot.slide.stop();
            }

            if (gamepad2.dpad_up) {
                robot.arm.setPosition(Arm.positions.hanging);
            }
            if (gamepad2.dpad_down) {
                robot.arm.setPosition(Arm.positions.pickup);
            }
            if (gamepad2.dpad_right) {
                robot.arm.setPosition(Arm.positions.scoring);
            }
            if (gamepad2.dpad_left) {
                robot.arm.setPosition(Arm.positions.outOfTheWay);
            }

            if (gamepad2.x) {
                robot.claw.openClaw();
            }
            if (gamepad2.a) {
                robot.claw.closeClaw();
            }

            if (-gamepad2.right_stick_y > 0.5) {
                robot.claw.openRightClaw();
            }
            if (-gamepad2.right_stick_y < -0.5) {
                robot.claw.closeRightClaw();
            }

            if (-gamepad2.left_stick_y > 0.5) {
                robot.claw.openLeftClaw();
            }
            if (-gamepad2.left_stick_y < -0.5) {
                robot.claw.closeLeftClaw();
            }
        }
    }
}
