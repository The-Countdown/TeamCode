package org.firstinspires.ftc.teamcode.subsystems;
public class Drive extends Robot.HardwareDevices {
    public void stop() {
        MotorFL.setVelocity(0);
        MotorFR.setVelocity(0);
        MotorBL.setVelocity(0);
        MotorBR.setVelocity(0);
    }

    public void move(double forwardsVelocity, double sidewaysVelocity) {
        MotorFL.setVelocity(forwardsVelocity - sidewaysVelocity);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBR.setVelocity(forwardsVelocity - sidewaysVelocity);
    }
    public void move(double forwardsVelocity, double sidewaysVelocity, double amount) {
        double start = MotorFL.getCurrentPosition();

        move(forwardsVelocity, sidewaysVelocity);

        if (amount > 0) {
            while (Math.abs(MotorFL.getCurrentPosition() - start) < amount) {
                //wait
            }
        } else {
            while (MotorFL.getCurrentPosition() - start > amount) {
                //wait
            }
        }

        stop();
    }

    public void move(double forwardsVelocity, double sidewaysVelocity, double turnR, double turnL){
        MotorFL.setVelocity(forwardsVelocity - sidewaysVelocity + turnR - turnL);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
        MotorBR.setVelocity(forwardsVelocity - sidewaysVelocity - turnR + turnL);
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity){
        // get IMU and clamp to 0-360
        double angle = imu.getAngularOrientation().firstAngle;
        if (angle < 0) {
            angle += 360;
        }

        //adjust motor powers
        double angleInRadians  = Math.toRadians(angle);
        double newForward = forwardsVelocity * Math.cos(angleInRadians) +
                sidewaysVelocity * Math.sin(angleInRadians);
        sidewaysVelocity = -forwardsVelocity * Math.sin(angleInRadians) +
                sidewaysVelocity * Math.cos(angleInRadians);
        forwardsVelocity = newForward;

        MotorFL.setVelocity(forwardsVelocity - sidewaysVelocity);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity);
        MotorBR.setVelocity(forwardsVelocity - sidewaysVelocity);
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity, double amount) {
        double start = MotorFL.getCurrentPosition();

        moveField(forwardsVelocity, sidewaysVelocity);

        if (amount > 0) {
            while (Math.abs(MotorFL.getCurrentPosition() - start) < amount) {
                //wait
            }
        } else {
            while (MotorFL.getCurrentPosition() - start > amount) {
                //wait
            }
        }

        stop();
    }

    public void moveField(double forwardsVelocity, double sidewaysVelocity, double turnR, double turnL){
        // get IMU and clamp to 0-360
        double angle = imu.getAngularOrientation().firstAngle;
        if (angle < 0) {
            angle += 360;
        }

        //adjust motor powers
        double angleInRadians  = Math.toRadians(angle);
        double newForward = forwardsVelocity * Math.cos(angleInRadians) +
                sidewaysVelocity * Math.sin(angleInRadians);
        sidewaysVelocity = -forwardsVelocity * Math.sin(angleInRadians) +
                sidewaysVelocity * Math.cos(angleInRadians);
        forwardsVelocity = newForward;

        MotorFL.setVelocity(forwardsVelocity - sidewaysVelocity + turnR - turnL);
        MotorFR.setVelocity(forwardsVelocity + sidewaysVelocity - turnR + turnL);
        MotorBL.setVelocity(forwardsVelocity + sidewaysVelocity + turnR - turnL);
        MotorBR.setVelocity(forwardsVelocity - sidewaysVelocity - turnR + turnL);
    }

    public void turnToAngle(double velocity, double targetAngle) {
        double angle;
        do {
            angle = -imu.getAngularOrientation().firstAngle;

            while (angle > 360) {
                angle -= 360;
            }

            while (angle < 0) {
                angle += 360;
            }

            if (angle < targetAngle) {
                move(0, 0, velocity, -velocity);
            } else if (angle > targetAngle - 1) {
                move(0, 0, -velocity, velocity);
            } else {
                stop();
            }
        } while (MotorFL.getVelocity() != 0);
        stop();
    }
}
