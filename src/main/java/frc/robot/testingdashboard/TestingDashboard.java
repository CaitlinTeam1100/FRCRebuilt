/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.testingdashboard;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Iterator;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.testingdashboard.TestingDashboardDataTable.DataGroup;
import frc.robot.testingdashboard.TestingDashboardDataTable.TDValType;

/**
 * This class sets up a testing dashboard using
 * WPILib's ShuffleBoard. The testing dashboard
 * contains a single tab for every tabName
 * containing the tabName status and all
 * commands associated with that tabName.
 * 
 * There is also a debug tab that contains sensor
 * variables and debug values.
 * 
 * This class is a singleton that should be
 * instantiated in the robotInit method
 * of the Robot class.
 */
public class TestingDashboard {
  private static TestingDashboard testingDashboard;
  private HashMap<String, TestingDashboardTab> testingTabs;
  boolean initialized = false;
    
  private TestingDashboard() {
    testingTabs = new HashMap<String, TestingDashboardTab>();
    initialized = false;
  }

  public static TestingDashboard getInstance() {
    if (testingDashboard == null) {
      testingDashboard = new TestingDashboard();
    }
    return testingDashboard;
  }

  private boolean hasTab(String tabName) {
    return testingTabs.containsKey(tabName);
  }

  private TestingDashboardTab getTab(String tabName) {
    return testingTabs.get(tabName);
  }

  /*
   * This function registers a tab with
   * the testing dashboard.
   */
  void registerTab(String tabName) {
    if (hasTab(tabName)) {
      // Subsystem has already been registered
      return;
    }
    TestingDashboardTab tdt = new TestingDashboardTab(tabName);
    testingTabs.put(tabName, tdt);
    System.out.println("Subsystem " + tabName + " registered with TestingDashboard");
  }
    
  /*
   * This function registers a command with a tabName
   * and a command group in the command table on the testing
   * dashboard.
   */
  void registerCommand(String tabName, String cmdGrpName, Command command) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for command does not exist!");
      return;
    }
    System.out.println("Adding command " + command.toString());
    tab.commandTable.add(cmdGrpName, command);
  }

  void registerNumber(String tabName, String dataGrpName, String dataName, double defaultValue) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    System.out.println("Adding data " + dataName);
    if (tab.dataTable.addName(dataGrpName, dataName))
      tab.dataTable.addDefaultNumberValue(dataGrpName, dataName, defaultValue);
  }

  void registerBoolean(String tabName, String dataGrpName, String dataName, boolean defaultValue) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    System.out.println("Adding data " + dataName);
    if (tab.dataTable.addName(dataGrpName, dataName))
      tab.dataTable.addDefaultBooleanValue(dataGrpName, dataName, defaultValue);
  }

  void registerString(String tabName, String dataGrpName, String dataName, String defaultValue) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    System.out.println("Adding String data " + dataName);
    if (tab.dataTable.addName(dataGrpName, dataName))
      tab.dataTable.addDefaultStringValue(dataGrpName, dataName, defaultValue);
  }

  void registerSendable(String tabName, String dataGrpName, String dataName, Sendable sendable) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    System.out.println("Adding Sendable data " + dataName);
    if (tab.dataTable.addName(dataGrpName, dataName))
      tab.dataTable.addDefaultSendableValue(dataGrpName, dataName, sendable);
  }

   void updateNumber(String tabName, String dataGrpName, String dataName, double value) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    tab.dataTable.getEntry(dataGrpName, dataName).setDouble(value);
  }

  void updateBoolean(String tabName, String dataGrpName, String dataName, boolean value) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    tab.dataTable.getEntry(dataGrpName, dataName).setBoolean(value);
  }

  void updateString(String tabName, String dataGrpName, String dataName, String value) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return;
    }
    tab.dataTable.getEntry(dataGrpName, dataName).setString(value);
  }

   double getNumber(String tabName, String dataGrpName, String dataName) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return 0;
    }
    if (initialized) {
      return tab.dataTable.getEntry(dataGrpName, dataName).getDouble(0.0);
    } else {
      return tab.dataTable.getDefaultNumberValue(dataGrpName, dataName);
    }
  }

  boolean getBoolean(String tabName, String dataGrpName, String dataName) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return false;
    }
    if (initialized) {
      return tab.dataTable.getEntry(dataGrpName, dataName).getBoolean(false);
    } else {
      return tab.dataTable.getDefaultBooleanValue(dataGrpName, dataName);
    }
  }

  String getString(String tabName, String dataGrpName, String dataName) {
    TestingDashboardTab tab = getTab(tabName);
    if (tab == null) {
      System.out.println("WARNING: Subsystem for data does not exist!");
      return "";
    }
    if (initialized) {
      return tab.dataTable.getEntry(dataGrpName, dataName).getString("");
    } else {
      return tab.dataTable.getDefaultStringValue(dataGrpName, dataName);
    }
  }

  /**
   * 
   */
  public void createTestingDashboard() {
    System.out.println("Creating Testing Dashboard");
    for (String tabName : testingTabs.keySet()) {
      // Create Shuffleboard Tab
      TestingDashboardTab tdt = testingTabs.get(tabName);
      tdt.tab = Shuffleboard.getTab(tdt.tabName);
      // Add Command Groups and Commands
      Enumeration<String> cmdGrpNames = tdt.commandTable.getCommandGroups();
      Iterator<String> it = cmdGrpNames.asIterator();
      System.out.println("Created tab for " + tdt.tabName + " tabName");
      int colpos = 0; // columns in shuffleboard tab
      while (it.hasNext()) {
        String cmdGrpName = it.next();
        System.out.println("Creating \"" + cmdGrpName + "\" command group");
        ShuffleboardLayout layout = tdt.tab.getLayout(cmdGrpName, BuiltInLayouts.kList);
        layout.withPosition(colpos,0);
        layout.withSize(1, tdt.commandTable.getCommandList(cmdGrpName).size());
        for (Command cmd : tdt.commandTable.getCommandList(cmdGrpName)) {
          layout.add(cmd);
        }
        colpos++;
      }

      // Add Data Entries
      Enumeration<String> dataGrpNames = tdt.dataTable.getDataGroups();
      Iterator<String> itd = dataGrpNames.asIterator();
      while (itd.hasNext()) {
        String dataGrpName = itd.next();
        System.out.println("Creating \"" + dataGrpName + "\" data group");

        @SuppressWarnings("unchecked")
        ArrayList<String> dataList = (ArrayList<String>)tdt.dataTable.getDataList(dataGrpName).names.clone();
        Collections.sort(dataList);

        ShuffleboardLayout layout = tdt.tab.getLayout(dataGrpName, BuiltInLayouts.kList);
        layout.withPosition(colpos,0);
        layout.withSize(8,dataList.size());

        for (int j = 0; j < dataList.size(); j++) {
          String entryName = dataList.get(j);
          double defaultNumberValue = 0;
          String defaultStringValue = "";
          boolean defaultBooleanValue = false;
          Sendable sendable;
          GenericEntry entry;
          TDValType type = tdt.dataTable.getType(dataGrpName, entryName);
          switch (type) {
            case NUMBER:
              defaultNumberValue = tdt.dataTable.getDefaultNumberValue(dataGrpName, entryName);
              entry = layout.add(entryName, defaultNumberValue).getEntry();
              tdt.dataTable.addEntry(dataGrpName, entryName, entry);
              break;
            case STRING:
              defaultStringValue = tdt.dataTable.getDefaultStringValue(dataGrpName, entryName);
              entry = layout.add(entryName, defaultStringValue).getEntry();
              tdt.dataTable.addEntry(dataGrpName, entryName, entry);
              break;
            case SENDABLE:
              sendable = tdt.dataTable.getDefaultSendableValue(dataGrpName, entryName);
              layout.add(entryName, sendable);
              break;
            case BOOLEAN:
              defaultBooleanValue = tdt.dataTable.getDefaultBooleanValue(dataGrpName, entryName);
              entry = layout.add(entryName, defaultBooleanValue).getEntry();
              tdt.dataTable.addEntry(dataGrpName, entryName, entry);
              break;
            default:
              System.out.println("ERROR: Type is " + type + " for data item \"" + entryName + "\"");
              break;
          }
        }
        colpos++;
      }

    }
    createDebugTab();
    initialized = true;
  }

  public void createDebugTab() {

  }
 
  public void updateDebugTab() {
  }
}