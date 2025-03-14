package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONObject;
import org.json.JSONArray;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp
public class LLTest extends LinearOpMode {
    public static String LIMELIGHT_IP = "172.29.0.1"; // Replace with your Limelight's IP


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()){
            String urlString = "http://" + LIMELIGHT_IP + ":5807/results";

            try {
                // Open connection
                URL url = new URL(urlString);
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.connect();

                // Check response
                if (conn.getResponseCode() == 200) {
                    InputStreamReader reader = new InputStreamReader(conn.getInputStream());
                    StringBuilder result = new StringBuilder();
                    int ch;
                    while ((ch = reader.read()) != -1) {
                        result.append((char) ch);
                    }
                    reader.close();

                    // Parse JSON response
                    JSONObject json = new JSONObject(result.toString());
                    JSONArray retroTargets = json.getJSONArray("Retro");

                    // Extract and print raw corners
                    for (int i = 0; i < retroTargets.length(); i++) {
                        JSONObject target = retroTargets.getJSONObject(i);
                        JSONArray ptsArray = target.getJSONArray("pts");
                        telemetry.addData("Corners of target " + (i + 1)+ ": " , ptsArray);
                    }
                } else {
                    telemetry.addData("Failed to fetch data: " ,conn.getResponseCode());
                }
            } catch (Exception e) {
                telemetry.addData("error", e);
            }
            telemetry.update();
        }
    }
}