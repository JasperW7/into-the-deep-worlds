package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONObject;
import org.json.JSONArray;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp
public class LLAngle extends LinearOpMode {
    public static String LIMELIGHT_IP = "172.29.0.1"; // Replace with your Limelight's IP
    public static JSONArray p1,p2,p3,p4;
    double mid1x,mid1y,mid2x,mid2y;

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
                        if (ptsArray.length() == 4){
                            telemetry.addData("Corners of target " + (i + 1)+ ": " , ptsArray);
                            p1 = ptsArray.getJSONArray(0); p2 = ptsArray.getJSONArray(1); p3 = ptsArray.getJSONArray(2); p4 = ptsArray.getJSONArray(3);
                            double p1x = p1.getDouble(0),p1y = p1.getDouble(1), p2x = p2.getDouble(0),p2y = p2.getDouble(1),p3x = p3.getDouble(0),p3y = p3.getDouble(1),p4x = p4.getDouble(0),p4y = p4.getDouble(1);
                            double d1 = Math.pow(Math.pow(p1x-p2x,2)+Math.pow(p1y-p2y,2),0.5), d2 = Math.pow(Math.pow(p1x-p4x,2)+Math.pow(p1y-p4y,2),0.5);
                            if (d1>d2){
                                mid1x = (p1x+p2x)/2;mid1y = (p1y+p2y)/2;mid2x = (p3x+p4x)/2;mid2y = (p3y+p4y)/2;
                            }else{
                                mid1x = (p1x+p4x)/2;mid1y = (p1y+p4y)/2;mid2x = (p2x+p3x)/2;mid2y = (p2y+p3y)/2;
                            }
                            double adj = Math.abs(mid1x-mid2x), opp = Math.abs(mid1y-mid2y),hyp = Math.hypot(opp,adj);
                            double angle = Math.toDegrees(Math.atan2(opp,adj));
                            double [] meow = {mid1x,mid1y,mid2x,mid2y};
                            telemetry.addData("Mid",meow);
                            telemetry.addData("angle",angle);
                            break;


                        }


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