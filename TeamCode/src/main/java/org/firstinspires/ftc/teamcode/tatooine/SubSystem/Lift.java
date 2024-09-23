package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift {
    DcMotorEx liftMotor = null;
    Telemetry telemetry;
    private boolean didntFinishedHanging = true;
    private double power = 0;
    private static boolean isDebug = false;
    private boolean initialized = false;

    public Lift(OpMode opMode, boolean isDebug) {
        telemetry = opMode.telemetry;
        this.isDebug = isDebug;
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        if (isDebug) {
            opMode.telemetry.addData("LiftConstructor", true);
        }
    }

    public void init() {
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();
        if (isDebug) {
            telemetry.addData("LiftInit", true);
        }
    }

    public Action hanging() {
        if (liftMotor.getCurrent(CurrentUnit.AMPS) <= 5 && power < 0) {
            resetEncoders();
            if (isDebug) {
                telemetry.addData("LiftEncoder", liftMotor.getCurrentPosition());
            }
        }
        return new setPowerAction();
    }

    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }

    public void setLiftMotor(DcMotorEx liftMotor) {
        this.liftMotor = liftMotor;
    }

    public boolean isDidntFinishedHanging() {
        return didntFinishedHanging;
    }

    public void setDidntFinishedHanging(boolean didntFinishedHanging) {
        this.didntFinishedHanging = didntFinishedHanging;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public class setPowerAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            while (liftMotor.getCurrentPosition() >= 0 && !initialized) {
                liftMotor.setPower(1);
                if (isDebug) {
                    telemetryPacket.put("liftUp", true);
                }
            }
            while (liftMotor.getCurrentPosition() <= 0 && !initialized) {
                liftMotor.setPower(-1);
                if (isDebug) {
                    telemetryPacket.put("firstAscend", true);
                }
            }
            while (liftMotor.getCurrentPosition() <= 0 && !initialized) {
                liftMotor.setPower(1);
                if (isDebug) {
                    telemetryPacket.put("liftUpSecondTime", true);
                }
            }
            if (isDebug) {
                telemetryPacket.put("secondAscend", true);
            }
            if (!initialized) {
                liftMotor.setPower(-1);
            }
            return true;

        }
    }
}
