package org.firstinspires.ftc.teamcode.subsystems;

public class Claw extends Robot.HardwareDevices {
    public static final class positions {
        public static final double open = 0.5;
        public static final double closed = 0.1;
    }

    public void closeClaw() {
        ClawL.setPosition(positions.closed);
        ClawR.setPosition(positions.closed);
    }

    public void openClaw() {
        ClawL.setPosition(positions.open);
        ClawR.setPosition(positions.open);
    }

    public void openLeftClaw() {
        ClawL.setPosition(positions.open);
    }

    public void openRightClaw() {
        ClawR.setPosition(positions.open);
    }

    public void closeLeftClaw() {
        ClawL.setPosition(positions.closed);
    }

    public void closeRightClaw() {
        ClawR.setPosition(positions.closed);
    }
}
