/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
/**
 * Add your docs here.
 */
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import java.util.Iterator;
import java.util.Map;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Collection<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  // kP = 5e-5;
  // kI = 1e-6;
  // kD = 0;
  // kIz = 0;
  // kFF = 0;
  // kMaxOutput = 1;
  // kMinOutput = -1;
  // maxRPM = 5700;

  static {

    // Tilt smart motion

    prefDict.put("tILKp", .0176);
    prefDict.put("tILKi", 0.000012);
    prefDict.put("tILKd", .0005);
    prefDict.put("tILKiz", .000001);
    prefDict.put("tILAcc", 500.);// rpm/sec motor - closed loop
    prefDict.put("tILTune", 0.);

    // Turret smart motion
    prefDict.put("tURKp", .0176);
    prefDict.put("tURKi", .000012);
    prefDict.put("tURKd", .000001);
    prefDict.put("tURKiz", 1.);
    prefDict.put("tURAcc", 500.);// rpm/sec motor
    prefDict.put("tURTune", 0.);

    // shooter velocity

    prefDict.put("sHFf", .00017);
    prefDict.put("sHKp", 3e-4);
    prefDict.put("sHkI", .0);
    prefDict.put("sHKd", .0);
    prefDict.put("sHKiz", 100.);
    prefDict.put("sHTune", 0.);

    // Drive
    prefDict.put("dRKp", .2);
    prefDict.put("dRKi", .0);
    prefDict.put("dRKd", .0);
    prefDict.put("dRKiz", .0);
    prefDict.put("dRKacc", 500.);// rpm/sec motor
    prefDict.put("dRTune", 0.);
  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = RobotContainer.prefs.getKeys();
    // v = (Vector<String>) RobotContainer.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && RobotContainer.prefs.containsKey(myArray[i])) {
        RobotContainer.prefs.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!RobotContainer.prefs.containsKey((tempString)))
        RobotContainer.prefs.putDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return RobotContainer.prefs.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs() {
    RobotContainer.prefs.removeAll();
  }

}
