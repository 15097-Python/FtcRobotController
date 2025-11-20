package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@Autonomous(name = "DualNormalizedColorSensorTest")
public class ColorSensorTestOpMode extends LinearOpMode {


    int color = 0;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;

    @Override
    public void runOpMode() {

        // Initialize the sensors
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        float[] hsv1 = new float[3];
        float[] hsv2 = new float[3];

        // Optional: tweak sensor gain for stronger signal
        colorSensor1.setGain(5);  // Try values 1–10
        colorSensor2.setGain(5);

        while (opModeIsActive()) {

            // Sensor 1
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsv1);
            float hue1 = hsv1[0];

            // Sensor 2
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            Color.colorToHSV(colors2.toColor(), hsv2);
            float hue2 = hsv2[0];

            // Average the hue values
            float avgHue = (hue1 + hue2) / 2;
            String combinedColor = getColorName(avgHue);

            int red1 = (int) (colors1.red * 255);
            int green1 = (int) (colors1.green * 255);
            int blue1 = (int) (colors1.blue * 255);

            int red2 = (int) (colors2.red * 255);
            int green2 = (int) (colors2.green * 255);
            int blue2 = (int) (colors2.blue * 255);

            // Telemetry output
            telemetry.addLine("SENSOR 1");
            telemetry.addData("Hue1", "%.1f", hue1);
            telemetry.addData("Detected", getColorName(hue1));

            telemetry.addLine("SENSOR 2");
            telemetry.addData("Hue2", "%.1f", hue2);
            telemetry.addData("Detected", getColorName(hue2));

            telemetry.addLine("COMBINED RESULT");
            telemetry.addData("Average Hue", "%.1f", avgHue);
            telemetry.addData("Combined Color", combinedColor);

            telemetry.update();
        }
    }

    private String getColorName(float hue) {
        if (hue >= 140 && hue <= 160) {
            color = 1;
            return "GREEN";

        } else if (hue >= 215 && hue <= 245   ) {
            color = 2;
            return "PURPLE";
        } else if (hue == 120) {
            color = 0;
            return "OTHER";
        } else {
            color = 0;
            return "OTHER";
        }
    }
}
