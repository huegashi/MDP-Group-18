package com.example.mdp_android.controllers;

import android.os.Handler;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import com.example.mdp_android.ui.grid.Map;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class RpiController {
    private static final String TAG = "RPI messages";
    private static BluetoothController bController = BluetoothControllerSingleton.getInstance(new Handler());

    public static String getRpiMessageType(String received) {
        try {
            JSONObject jsonObj = new JSONObject(received);
            String category = jsonObj.getString("cat");

            if (category.equals("location")) {
                return "robot";
            } else if (category.equals("image-rec")) {
                return "image";
            } else if (category.equals("PATH")) {
                return "path";
            } else if (category.equals("obstacles")) {
                return "obstacles";
            } else if (category.equals("status")) {
                return "status";
            } else if (category.equals("error")) {
                return "error";
            } else if (category.equals("mode")) {
                return "mode";
            } else if (category.equals("control")) {
                return "control";
            } else if (category.equals("info")) {
                return "info";
            }
        } catch (Exception e) {
            Log.e(TAG, "Failed to parse message type: ", e);
        }
        return "";
    }

    public static JSONObject readRpiMessages(String received) {
        try {
            Log.e(TAG, "Received object is: " + received);
            JSONObject jsonObj = new JSONObject(received);
            String category = jsonObj.getString("cat");

            if (category.equals("location")) {
                return jsonObj.getJSONObject("value");
            } else if (category.equals("image-rec")) {
                return jsonObj.getJSONObject("value");
            } else if (category.equals("PATH")) {
                return jsonObj.getJSONObject("value");
            } else if (category.equals("status")) {
                return jsonObj;
            } else if (category.equals("error")) {
                return jsonObj;
            }
        } catch (Exception e) {
            Log.e(TAG, "Failed to parse json: ", e);
        }
        return null;
    }

    public static JSONObject readSecondJSONMessages(String received) {
        try {
            Log.e(TAG, "Received object is: " + received);
            JSONObject jsonObj = new JSONObject(received);
            if (jsonObj.get("cat").equals("PATH")) {
                JSONObject path = jsonObj.getJSONObject("value");
                return path;
            }
        } catch (Exception e) {
            Log.e(TAG, "Failed to pass json: ", e);
        }
        return null;
    }

    public static String getRobotStatus(JSONObject robot) {
        String status = "";
        try {
            String x = robot.get("x").toString();
            String y = robot.get("y").toString();

            String direction;
            try {
                int dirValue = robot.getInt("d");
                switch (dirValue) {
                    case 0: direction = "N"; break;
                    case 1: direction = "E"; break;
                    case 2: direction = "S"; break;
                    case 3: direction = "W"; break;
                    default: direction = "N";
                }
            } catch (Exception e) {
                direction = robot.getString("dir");
            }

            status = "robot at (" + x + " , " + y + ") facing " + direction;
            Log.d(TAG, "robot current status: " + status);
        } catch (Exception e) {
            Log.d(TAG, "failed to parse json: " + e);
        }
        return status;
    }

    public static String getTargetStatus(JSONObject results) {
        String status = "";
        try {
            String obsId = results.get("obstacle_id").toString();
            String imgId = results.get("image_id").toString();
            status = "Obstacle " + obsId + " → Image " + imgId;
        } catch (Exception e) {
            Log.d(TAG, "failed to parse json: " + e);
        }
        return status;
    }

    public static ArrayList<ArrayList<Integer>> getPath(JSONObject results) {
        ArrayList<ArrayList<Integer>> path = new ArrayList<>();
        try {
            Log.d(TAG, "getting path");
            JSONArray pathJson = results.getJSONArray("path");
            Log.d(TAG, "path: " + pathJson);
            for (int i = 0; i < pathJson.length(); i++) {
                JSONArray coorJson = pathJson.getJSONArray(i);
                ArrayList<Integer> coor = new ArrayList<>();
                for (int j = 0; j < coorJson.length(); j++) {
                    coor.add(Integer.parseInt(coorJson.get(j).toString()));
                }
                path.add(coor);
            }
        } catch (Exception e) {
            Log.d(TAG, "failed to parse json: " + e);
        }
        return path;
    }

    public static JSONObject getMapDetails(String task, Map.Robot robot, ArrayList<Map.Obstacle> obstacles) {
        return getObstaclesMessage(obstacles, "0");
    }

    public static JSONObject getRobotDetails(Map.Robot robot) {
        JSONObject message = new JSONObject();
        try {
            JSONObject value = new JSONObject();
            value.put("x", robot.getX() - 1);
            value.put("y", robot.getY() - 1);

            String dir = robot.getDirection();
            int directionValue = 0;
            switch (dir) {
                case "N": directionValue = 0; break;
                case "E": directionValue = 2; break;
                case "S": directionValue = 4; break;
                case "W": directionValue = 6; break;
            }
            value.put("d", directionValue);

            message.put("cat", "location");
            message.put("value", value);

        } catch (Exception e) {
            Log.d(TAG, "Failed to create robot message: ", e);
        }
        return message;
    }

    public static JSONObject getObstaclesMessage(ArrayList<Map.Obstacle> obstacles, String mode) {
        JSONObject message = new JSONObject();
        JSONObject value = new JSONObject();
        JSONArray obstaclesArray = new JSONArray();

        try {
            for (Map.Obstacle obstacle : obstacles) {
                JSONObject obs = new JSONObject();
                obs.put("x", obstacle.getObsXCoor() - 1);
                obs.put("y", obstacle.getObsYCoor() - 1);
                obs.put("id", obstacle.getObsID());

                String dir = obstacle.getDirection();
                int directionValue = 0;
                switch (dir) {
                    case "N": directionValue = 0; break;
                    case "E": directionValue = 2; break;
                    case "S": directionValue = 4; break;
                    case "W": directionValue = 6; break;
                }
                obs.put("d", directionValue);

                obstaclesArray.put(obs);
            }

            value.put("obstacles", obstaclesArray);
            value.put("mode", mode);

            message.put("cat", "obstacles");
            message.put("value", value);

        } catch (Exception e) {
            Log.e(TAG, "Failed to create obstacles message: ", e);
        }
        return message;
    }

    public static JSONObject getStartCommand() {
        JSONObject message = new JSONObject();
        try {
            message.put("cat", "control");
            message.put("value", "start");
        } catch (Exception e) {
            Log.d(TAG, "Failed to create start command: ", e);
        }
        return message;
    }

    public static JSONObject getNavDetails(List<String> commands) {
        JSONObject message = new JSONObject();
        try {
            String command = commands.get(0);
            message.put("cat", "info");
            message.put("value", command);
        } catch (Exception e) {
            Log.d(TAG, "Failed to parse string into json: ", e);
        }
        return message;
    }

    public static void sendToRpi(JSONObject jsonObj) {
        try {
            bController.write(jsonObj.toString().getBytes(StandardCharsets.UTF_8));
            Log.d(TAG, "sendToRPi: \n" + jsonObj.toString(2));
        } catch (Exception e) {
            Log.e(TAG, "Failed to send message to rpi: ", e);
        }
    }
}