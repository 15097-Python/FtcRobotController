package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name="VariablePowerLauncherTellyOP")

public class VariablePowerLauncherTellyOP extends LinearOpMode {
    private DcMotorImplEx leftmotor = hardwareMap.get(DcMotorImplEx.class, "leftmotor");
    private DcMotorImplEx rightmotor = hardwareMap.get(DcMotorImplEx.class, "rightmotor");
    private static final double launcherwheelradiusm = .02;


    @Override
    public void runOpMode() {

        double motortargetspeedradians = 0;

        //zeros the encoders and sets the run using encoder mode
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        waitForStart();
        while (opModeIsActive()) {

            //increments the target speed with up and right while decrementing it with left and down
            motortargetspeedradians += gamepad1.dpadUpWasPressed() ? 1 : 0;
            motortargetspeedradians -= gamepad1.dpadDownWasPressed() ? 1 : 0;
            motortargetspeedradians += gamepad1.dpadRightWasPressed() ? .1 : 0;
            motortargetspeedradians -= gamepad1.dpadLeftWasPressed() ? .1 : 0;


            leftmotor.setVelocity(motortargetspeedradians);// 32.67 radians per second is the max
            rightmotor.setVelocity(-motortargetspeedradians);

            telemetry.addLine("All Speeds are in Radians Per Second");
            telemetry.addData("Motors' Target Rate of Rotation ", motortargetspeedradians);
            telemetry.addData("Left Motor Actual Rate of Rotation", leftmotor.getVelocity());
            telemetry.addData("Right Motor Actual Rate of Rotation", rightmotor.getVelocity());
            telemetry.addData("Left Motor Speed at Wheel Surface",leftmotor.getVelocity()*launcherwheelradiusm);
            telemetry.addData("Right Motor Speed at Wheel Surface",rightmotor.getVelocity()*launcherwheelradiusm);

            telemetry.update();
        }
    }
}