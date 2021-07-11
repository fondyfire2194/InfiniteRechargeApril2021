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

  static {

    // Tilt smart motion

    prefDict.put("tIKp", .001);
    prefDict.put("tIKi", .001);
    prefDict.put("tIKd", 0.);
    prefDict.put("tIKiz", 1.);
    prefDict.put("tIMaxV", 250.);// 1000 deg per min
    prefDict.put("tIMaxA", 700.);// deg per min/sec
    prefDict.put("tITune", 0.);

    // tilt vel
    prefDict.put("tIKpv", .001);
    prefDict.put("tIKiv", .001);
    prefDict.put("tIKdv", 0.);
    prefDict.put("tIKizv", 1.);
    prefDict.put("tIMaxVv", 45.);// these are in revs(deg)per second
    prefDict.put("tIMaxAv", 125.);// deg per min/sec
    prefDict.put("tITunev", 0.);

    // Tilt Lock

    prefDict.put("TiLkP", 20.);
    prefDict.put("TiLkI", .01);
    prefDict.put("TiLkD", 0.5);
    prefDict.put("TiLkIZ", 0.001);
    prefDict.put("tiLTune", 0.);
    prefDict.put("TiltLockAdd", 0.);

    // Turret smart motion
    prefDict.put("tURKp", .00018);
    prefDict.put("tURKi", .0001);
    prefDict.put("tURKd", .0002);
    prefDict.put("tURKiz", 1.);
    prefDict.put("tURMaxV", 250.);// deg/sec motor
    prefDict.put("tURMaxA", 600.);// deg/sec motor
    prefDict.put("tURTune", 0.);

    // turret vel
    // Turret smart motion
    prefDict.put("tURKpv", .000);
    prefDict.put("tURKiv", .000);
    prefDict.put("tURKdv", .000);
    prefDict.put("tURKizv", 1.);
    prefDict.put("tURMaxVv", 250.);// deg/sec motor
    prefDict.put("tURMaxAv", 850.);// deg/sec motor
    prefDict.put("tURTunev", 0.);

    // Turret Lock

    prefDict.put("TuLkP", 50.0);
    prefDict.put("TuLkI", .00);
    prefDict.put("TuLkD", 0.);
    prefDict.put("TuLkIZ", 0.);
    prefDict.put("tuLTune", 0.);

    // shooter velocity

    prefDict.put("sHff", .016);// =1/maxMPS = 1/(5700/60 * .638) .016
    prefDict.put("sHkp", .01);
    prefDict.put("sHkI", .0001);
    prefDict.put("sHkd", 50.);
    prefDict.put("sHkiz", 10.);
    prefDict.put("sHTune", 0.);

    // Drive

    prefDict.put("dRKff", .22);
    prefDict.put("dRKp", .1);
    prefDict.put("dRKi", .0);
    prefDict.put("dRKd", .0);
    prefDict.put("dRKiz", .0);
    prefDict.put("dRStKp", .15);
    prefDict.put("dRacc", .1);

    prefDict.put("dRTune", 0.);

    // cell servo

    prefDict.put("CellRelPosn", .25);
    prefDict.put("CellHoldPosn", .1);
    prefDict.put("CellReleaseTime", .25);

    // left channel servo

    prefDict.put("LeftRelPosn", .25);
    prefDict.put("LeftHoldPosn", .1);

    // camera

    prefDict.put("LimelightHeight", .66);
    prefDict.put("LimelightMaxHeight", .686);

    //Set to 1 befor power up to log
    prefDict.put("LogTilt", 0.);
    prefDict.put("LogTurret", 0.);
    prefDict.put("LogShoot", 0.);
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
