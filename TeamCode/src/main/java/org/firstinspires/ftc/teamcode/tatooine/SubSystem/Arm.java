package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

@Config
public class Arm {

    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Arm";

    // PID Tolerances
    public static double ANGLE_TOLERANCE = 3;// degrees
    private static final double EXTEND_TOLERANCE = 3;   // centimeters or appropriate unit

    // Encoder Counts Per Revolution (CPR)
    private static final double ANGLE_CPR = 28.0 * 70.0 * (34.0/16.0); // Arm angle motor
    private static final double EXTEND_CPR = 537.7*(25.0/25.0);// Arm extension motor

    // Physical Dimensions
    private static final double SPOOL_DIM = 3.8;  // cm, spool diameter for extension
    private static double ANGLE_OFFSET = -5;  // degrees, initial offset

    //PID

    public static double KP = 0;
    public static double KI = 0;

    public static double KD = 0;
    // Feedforward
    public static double KF = 0.13;  // feedforward constant


    // Current Limit (if applicable; set as needed)
    private static final double AMP_LIMIT = 0;  // TODO: Set correct value if using current-limiting logic

    public static double MAX_EXTEND = 60;

    private final static double MIN_EXTEND = 37;


    // ---------------------------------------------------------------------------------------------
    // Hardware Components
    // ---------------------------------------------------------------------------------------------
    private final DcMotorEx angleLeft;
    private final DcMotorEx angleRight;
    private final DcMotorEx extendLeft;
    private final DcMotorEx extendRight;
    private final DigitalChannel touchSensor;   // TODO: Initialize if needed

    // ---------------------------------------------------------------------------------------------
    // PID Controllers
    // ---------------------------------------------------------------------------------------------
//    public static PIDFController anglePID = new PIDFController(0.021, 0, 0, 0);
    public static PIDFController anglePID = new PIDFController(0.09, 0, 0.003, 0);
    private final PIDFController extendPID = new PIDFController(0.1, 0, 0.0001, 0);

    // ---------------------------------------------------------------------------------------------
    // State Variables
    // ---------------------------------------------------------------------------------------------
    private final boolean isDebugMode;
    // ---------------------------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------------------------
    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------------------
    /**
     * Creates an Arm subsystem instance with optional debug logging.
     *
     * @param opMode the active OpMode
     * @param isDebugMode whether to enable debug telemetry logging
     */
    public Arm(OpMode opMode, boolean isDebugMode) {
        this.telemetry = opMode.telemetry;
        this.isDebugMode = isDebugMode;

        // Retrieve motors from harare map
        this.angleLeft = opMode.hardwareMap.get(DcMotorEx.class, "AL");
        this.angleRight = opMode.hardwareMap.get(DcMotorEx.class, "AR");
        this.extendLeft = opMode.hardwareMap.get(DcMotorEx.class, "EL");
        this.extendRight = opMode.hardwareMap.get(DcMotorEx.class, "ER");

        // Retrieve touch sensor if present (set to null by default)
        this.touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "LS"); // TODO: Initialize if needed
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        // Perform initialization
        init();

        // Debug info
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Constructor Initialized", "Success");
    }

    /**
     * Creates an Arm subsystem instance with debug mode disabled by default.
     *
     * @param opMode the active OpMode
     */
    public Arm(OpMode opMode) {
        this(opMode, false);
    }

    // ---------------------------------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------------------------------
    /**
     * Initializes the Arm subsystem (motor directions, zero-power behavior, PID tolerances, etc.).
     */
    public void init() {
        // Set motor directions
        angleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        angleRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero-power behavior to BRAKE
        angleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure PID tolerances
        anglePID.setTolerance(ANGLE_TOLERANCE);
        extendPID.setTolerance(EXTEND_TOLERANCE);
        extendPID.setTimeout(2);
        anglePID.setTimeout(3);

        // Reset encoders at initialization
        resetEncoders();

        // Debug info
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Initialization", "Completed");
    }

    public boolean isDebugMode(){
        return isDebugMode;
    }

    // ---------------------------------------------------------------------------------------------
    // Encoder Management
    // ---------------------------------------------------------------------------------------------
    /**
     * Resets both angle and extension encoders.
     */
    public void resetEncoders() {
        resetAngleEncoder();
        resetExtendEncoders();

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Encoders Reset", "Success");
    }

    /**
     * Resets angle encoders for both angle motors.
     */
    public void resetAngleEncoder() {
        angleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        angleLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Angle Encoder Reset", "Success");
    }

    /**
     * Resets extension encoders for both extension motors.
     */
    public void resetExtendEncoders() {
        extendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Extend Encoder Reset", "Success");
    }

    public double getLeftAngle() {
        return (MathUtil.convertTicksToDegrees(ANGLE_CPR, angleLeft.getCurrentPosition())) + ANGLE_OFFSET;
    }

    public double getRightAngle() {
        return MathUtil.convertTicksToDegrees(ANGLE_CPR, angleRight.getCurrentPosition()) + ANGLE_OFFSET;
    }

    // ---------------------------------------------------------------------------------------------
    // Angle Functions
    // ---------------------------------------------------------------------------------------------
    /**
     * Calculates the current average angle of the arm by reading both angle encoders.
     *
     * @return the arm angle in degrees (average of both motors)
     */
    public double getAngle() {
        double angle = (getLeftAngle());
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Angle (Average)", angle);

        return angle;
    }

    public static double getAngleOffset() {
        return ANGLE_OFFSET;
    }

    public static void setAngleOffset(double angleOffset) {
        ANGLE_OFFSET = angleOffset;
    }

    /**
     * Sets power for both angle motors.
     *
     * @param power the power to apply to angle motors
     */
    public void setPowerAngle(double power) {
        angleLeft.setPower(power);
        angleRight.setPower(power);

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Set Power Angle", power);
    }
    public void setPowerAngleWithF(double power) {
        double FF = calculateF();

        if (touchSensor.getState()){
            resetAngleEncoder();
            if (power <= 0){
                power = 0;
                FF = 0;
            }
        }
        setPowerAngle(power + FF);


    }

    /**
     * Calculates the feedforward term (F) based on the current angle of the arm.
     *
     * @return the feedforward value to help hold the arm in place against gravity
     */
    public double calculateF() {
        double feedforward = Math.cos(Math.toRadians(getAngle())) * KF * (MIN_EXTEND + getExtend()) / (MIN_EXTEND);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Calculate F", feedforward);

        return feedforward;
    }

    // ---------------------------------------------------------------------------------------------
    // Extension Functions
    // ---------------------------------------------------------------------------------------------4
    public double getLeftExtention(){
        return MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendLeft.getCurrentPosition());
    }
    public double getRightExtention(){
        return MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendRight.getCurrentPosition());
    }

    public double getMinExtend(){
        return MIN_EXTEND;
    }
    public static double getMaxExtend() {
        return MAX_EXTEND;
    }




    /**
     *
     * Calculates the current extension length by reading both extension encoders.
     *
     * @return the average extension in centimeters (or relevant unit)
     */
    public double getExtend() {
        double leftExtend = getLeftExtention();
        double  rightExtend = getRightExtention();
        double extend = leftExtend;

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Extend (Left)", leftExtend);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Extend (Right)", rightExtend);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Extend (Average)", extend);

        return extend;
    }

    /**
     * Sets power for both extension motors.
     *
     * @param power the power to apply to extension motors
     */
    public void setPowerExtend(double power) {
        extendLeft.setPower(power);
        extendRight.setPower(power);

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Set Power Extend", power);
    }

    public void setPowerExtendWithLimit(double power,boolean isIntaking){
        if (isIntaking &&getExtend()> getMaxExtend()-20 && power>0 && getAngle()< 30){
            power =0;
        }
        setPowerExtend(power);
    }
    // ---------------------------------------------------------------------------------------------
    // Actions to Control Arm
    // ---------------------------------------------------------------------------------------------
    /**
     * Creates an Action to move the arm angle to a desired setpoint.
     *
     * @param angle the target angle in degrees
     * @return an Action that moves the arm angle
     */
    public Action setAngle(double angle) {
        return new AtSetPointAngle(angle);
    }

    public Action moveAngle() {
        return new MoveAngle();
    }


    /**
     * Creates an Action to move the arm extension to a desired setpoint.
     *
     * @param extension the target extension in centimeters (or relevant unit)
     * @return an Action that moves the arm extension
     */
    public Action setExtension(double extension) {
        return new MoveExtension(extension);
    }

    /**
     * Action sequence to score a specimen:
     * 1. Raise arm to 60 degrees
     * 2. Sleep for 3 seconds
     * 3. Retract extension to 0
     * 4. Sleep for 3 seconds
     * 5. Move arm to 45 degrees
     *
     * @return a SequentialAction representing the entire specimen scoring routine
     */
    public Action scoreSpecimenAction() {
        return new SequentialAction(
                setAngle(60),
                new SleepAction(3),
                setExtension(MAX_EXTEND),
                new SleepAction(3),
                setAngle(45)
        );
    }

    /**
     Action sequence to score a sample:
     * 1. Raise arm to 90 degrees
     * 2. Sleep for 3 seconds
     * 3. Retract extension to 0
     *
     * @return a SequentialAction representing the sample scoring routine
     */
    public Action scoreSampleAction() {
        return new SequentialAction(
                setAngle(90),
                new SleepAction(3),
                setExtension(0)
        );
    }

    /**
     * Closes or resets the arm by:
     * 1. Retracting extension to 0
     * 2. Moving arm angle to -15 degrees
     *
     * @return a SequentialAction representing the 'close' routine
     */
    public Action closeAction() {
        return new SequentialAction(
                setExtension(0),
                setAngle(-15)
        );
    }

    /**
     * Positions the arm for intake by:
     * 1. Moving arm angle to 0
     * 2. Retracting extension to 0
     *
     * @return a SequentialAction representing the 'intake' routine
     */
    public Action intakeAction() {
        return new SequentialAction(
                setAngle(0),
                setExtension(0)
        );
    }

    // ---------------------------------------------------------------------------------------------
    // Inner Classes for Actions
    // ---------------------------------------------------------------------------------------------
    /**
     * Inner class representing an Action to move the arm extension to a specific setpoint.
     */
    public class MoveExtension implements Action {
        private double goal;
        public MoveExtension(double goal) {
            this.goal = goal;
            extendPID.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendPID.setSetPoint(goal);
            double power = extendPID.calculate(getExtend());
            setPowerExtend(power);

            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "Move Extension Power", power);
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "goalExtend", goal);
            // Return false until we've reached the setpoint
            return !extendPID.atSetPoint();
        }
    }

    /**
     * Inner class representing an Action to move the arm angle to a specific setpoint.
     */
    public class MoveAngle implements Action {

        public MoveAngle() {
            anglePID.reset();
            anglePID.setSetPoint(getAngle());
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isDebugMode){
                anglePID.setP(KP);
                anglePID.setD(KD);
                anglePID.setI(KI);
            }
            if (touchSensor.getState()){
                resetAngleEncoder();
            }
            DebugUtils.logDebug(telemetry,isDebugMode,SUBSYSTEM_NAME,"ts",touchSensor.getState());

            double angle = getAngle();
            double pidPower = anglePID.calculate(angle);
            setPowerAngleWithF(pidPower);
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "Move Angle PID Power", pidPower);
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "goal", anglePID.getSetPoint());
            // always return true for the F to update itself always
            return true;
        }
    }

    public class AtSetPointAngle implements Action {
        private double goal;

        public AtSetPointAngle(double goal) {
            this.goal = goal;
            anglePID.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            anglePID.setSetPoint(this.goal);
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "goal", goal);
            return !anglePID.atSetPoint();
        }
    }
}
