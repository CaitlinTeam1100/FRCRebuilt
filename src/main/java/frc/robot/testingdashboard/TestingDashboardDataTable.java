/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.testingdashboard;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;

/**
 * TestingDashboardDataTable is a map of data groups to
 * sensor/testing values that can be mapped to a shuffleboard
 * tab.
 */
public class TestingDashboardDataTable {
  public enum TDValType  {
    NUMBER,
    STRING,
    SENDABLE,
    BOOLEAN
  }

  private final Hashtable<String, DataGroup> table;

  public static class DataGroup {
    public final ArrayList<String> names;
    public final Hashtable<String, GenericEntry> entries;
    public final Hashtable<String, TDValType> type;
    public final Hashtable<String, String> defaultString;
    public final Hashtable<String, Double> defaultDouble;
    public final Hashtable<String, Boolean> defaultBoolean;
    public final Hashtable<String, Object> defaultSendable;

    public DataGroup() {
      names = new ArrayList<String>();
      entries = new Hashtable<String, GenericEntry>();
      type = new Hashtable<String, TDValType>();
      defaultString = new Hashtable<String, String>();
      defaultDouble = new Hashtable<String, Double>();
      defaultBoolean = new Hashtable<String, Boolean>();
      defaultSendable = new Hashtable<String, Object>();
    }
  }

  public TestingDashboardDataTable() {
    table = new Hashtable<String, DataGroup>();
  }

  public boolean addName(String grp, String name) {
    if (table.containsKey(grp)) {
      DataGroup group = table.get(grp);
      if (!group.names.contains(name)) {
        group.names.add(name);
      } else {
        return false;
      }
    } else {
      DataGroup group = new DataGroup();
      group.names.add(name);
      table.put(grp, group);
    }
    return true;
  }
  
  public TDValType getType(String grpName, String name) {
    return table.get(grpName).type.get(name);
  }

  public void addDefaultStringValue(String grpName, String name, String value) {
    DataGroup group = table.get(grpName);
    if (group.names.contains(name)) {
        group.type.put(name,TDValType.STRING);
        group.defaultString.put(name,value);
    }
  }

  public void addDefaultNumberValue(String grpName, String name, double value) {
    DataGroup group = table.get(grpName);
    if (group.names.contains(name)) {
        group.type.put(name,TDValType.NUMBER);
        group.defaultDouble.put(name,Double.valueOf(value));
    }
  }

  public void addDefaultBooleanValue(String grpName, String name, boolean value) {
    DataGroup group = table.get(grpName);
    if (group.names.contains(name)) {
        group.type.put(name,TDValType.BOOLEAN);
        group.defaultBoolean.put(name,Boolean.valueOf(value));
    }
  }
  
  public void addDefaultSendableValue(String grpName, String name, Sendable sendable) {
    DataGroup group = table.get(grpName);
    if (group.names.contains(name)) {
        group.type.put(name,TDValType.SENDABLE);
        group.defaultSendable.put(name, sendable);
    }
  }

  public String getDefaultStringValue(String grpName, String name) {
    DataGroup group = table.get(grpName);
    return group.defaultString.get(name);
  }

  public double getDefaultNumberValue(String grpName, String name) {
    DataGroup group = table.get(grpName);
    return group.defaultDouble.get(name).doubleValue();
  }

  public boolean getDefaultBooleanValue(String grpName, String name) {
    DataGroup group = table.get(grpName);
    return group.defaultBoolean.get(name).booleanValue();
  }

  public Sendable getDefaultSendableValue(String grpName, String name) {
    DataGroup group = table.get(grpName);
    return (Sendable)group.defaultSendable.get(name);
  }

  /*
   *  This function adds the GenericEntry for a given
   *  named data item.
   */
  public void addEntry(String grpName, String name, GenericEntry entry) {
    DataGroup group = table.get(grpName);
    if (group.names.contains(name)) {
      group.entries.put(name,entry);
    }
  }

  public GenericEntry getEntry(String grpName, String str) {
    DataGroup group = table.get(grpName);
    return group.entries.get(str);
  }

  public Enumeration<String> getDataGroups() {
    return table.keys();
  }

  public DataGroup getDataList(String dataGrgName) {
    return table.get(dataGrgName);
  }
}