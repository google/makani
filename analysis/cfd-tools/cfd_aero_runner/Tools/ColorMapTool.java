/*
    Copyright 2020 Google LLC

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
 */
package Tools;

import java.io.*;
import java.util.*;
import star.base.neo.*;
import star.common.*;
import star.vis.*;

public class ColorMapTool {
    
  static String totalRangeYPlus = "wall_yplus_0_to_300_v2";
  static String highYPlusMap    = "high_y_plus";
  static String blueDkGnRedMap  = "blue-dkgreen-red";
  static String whiteWater      = "white-water";

  static String highContrastBands = "high-contrast-bands";

// Water but with white as the top of scale.
public static String getWhiteWaterString(Simulation tmpSim){
  UserLookupTable userLookupTable;
  String retString = whiteWater;
  try{
    userLookupTable = 
        ((UserLookupTable) tmpSim.get(LookupTableManager.class)
           .getObject(retString));
  }catch (NeoException e){
    userLookupTable = 
        (UserLookupTable) tmpSim.get(LookupTableManager.class)
            .createLookupTable();
    String propString = "{'Name': '"+retString+"'"
        + ", 'AlphaValues': [0.0, 1.0, 1.0, 1.0]"
        + ", 'ColorValues': [0.0, 1.0, 1.0, 1.0, 0.2, 0.9019607843137255, 0.9019607843137255, 0.9686274509803922, 0.3608786610878661, 0.14901960784313725, 0.48627450980392156, 1.0, 0.5355648535564853, 0.0, 0.4, 1.0, 0.7573221757322176, 0.0, 0.2, 0.6, 1.0, 0.0, 0.0, 0.4]"
        + ", 'ColorSpace': 0}\"";
    userLookupTable.setProperties(NeoProperty.fromString(propString));
  }
  return retString;
}

// Special colormap when viewing wall y+ from 0 to 300.
public static String getTotalRangeWallYPlusString(Simulation tmpSim){
  UserLookupTable userLookupTable;
  String retString = totalRangeYPlus;
  try{
    userLookupTable = 
        ((UserLookupTable) tmpSim.get(LookupTableManager.class)
            .getObject(retString));
  }catch (NeoException e){
    userLookupTable = 
        (UserLookupTable) tmpSim.get(LookupTableManager.class)
            .createLookupTable();
    String propString = "{'Name': '"+retString+"'"
        + ", 'AlphaValues': [0.0, 1.0, 1.0, 1.0]"
        + ", 'ColorValues': [0.0, 1.0, 1.0, 1.0, 0.016666, 0.0, 0.0, 1.0, 0.017, 1.0, 0.0, 1.0, 0.05196304849884527, 1.0, 0.0, 1.0, 0.0995, 1.0, 0.0, 1.0, 0.1, 0.0, 0.4, 0.0, 0.3333333, 0.0, 0.6, 0.0, 0.6666, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0]"
        + ", 'ColorSpace': 0}\"";
    userLookupTable.setProperties(NeoProperty.fromString(propString));
  }
  return retString;
}

public static String getHighWallYPlusString(Simulation tmpSim){
  UserLookupTable userLookupTable;
  String retString = highYPlusMap;
  try{
    userLookupTable = 
        ((UserLookupTable) tmpSim.get(LookupTableManager.class)
            .getObject(retString));
  }catch (NeoException e){
    userLookupTable = 
        (UserLookupTable) tmpSim.get(LookupTableManager.class)
            .createLookupTable();
    String propString = "{'Name': \'"+retString+"'"
        + ", 'AlphaValues': [0.0, 1.0, 1.0, 1.0]"
        + ", 'ColorValues': [0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.4, 0.0, 1.0, 1.0, 0.0, 0.0]"
        + ", 'ColorSpace': 1}\"";
    userLookupTable.setProperties(NeoProperty.fromString(propString));
  }
  return retString;
}
public static String getBlueDkGnRedString(Simulation tmpSim){
  UserLookupTable userLookupTable;
  String retString = blueDkGnRedMap;
  try{
    userLookupTable = 
        ((UserLookupTable) tmpSim.get(LookupTableManager.class)
            .getObject(retString));
  }catch (NeoException e){
    userLookupTable = 
        (UserLookupTable) tmpSim.get(LookupTableManager.class)
            .createLookupTable();
    String propString = "{'Name': \'"+retString+"'"
        + ", 'AlphaValues': [0.0, 1.0, 1.0, 1.0]"
        + ", 'ColorValues': [0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.6, 0.0, 1.0, 1.0, 0.0, 0.0]"
        + ", 'ColorSpace': 1}\"";
    userLookupTable.setProperties(NeoProperty.fromString(propString));
  }
  return retString;
}
public static String getHighContrastBands(Simulation tmpSim){
  UserLookupTable userLookupTable;
  String retString = highContrastBands;
  try{
    userLookupTable = 
        ((UserLookupTable) tmpSim.get(LookupTableManager.class)
            .getObject(retString));
  }catch (NeoException e){
    userLookupTable = 
        (UserLookupTable) tmpSim.get(LookupTableManager.class)
            .createLookupTable();
    String propString = "{'Name': \'"+retString+"'"
        + ", 'AlphaValues': [0.0, 0.0, 0.11538461538461539, 0.0, 0.15384615384615385, 1.0, 0.1902834008097166, 0.0, 0.39878542510121456, 0.0, 0.47368421052631576, 1.0, 0.52834008097166, 0.0, 0.5789473684210527, 0.5, 0.6356275303643725, 0.0, 0.7004048582995951, 0.0, 0.7348178137651822, 1.0, 0.7753036437246964, 0.0, 0.8299595141700404, 0.0, 0.9129554655870445, 0.5, 1.0, 1.0]"
        + ", 'ColorValues': [0.0, 0.2, 0.0, 0.2, 0.05263157894736842, 0.5, 0.0, 0.5, 0.10526315789473684, 0.4, 0.0, 0.6, 0.15789473684210525, 0.3, 0.0, 0.7, 0.21052631578947367, 0.2, 0.0, 0.8, 0.2631578947368421, 0.1, 0.0, 0.9, 0.3157894736842105, 0.0, 0.0, 1.0, 0.3684210526315789, 0.63, 0.63, 1.0, 0.42105263157894735, 0.0, 0.75, 1.0, 0.47368421052631576, 0.0, 1.0, 1.0, 0.5263157894736842, 0.1, 0.8, 0.7, 0.5789473684210527, 0.1, 0.9, 0.0, 0.631578947368421, 0.5, 1.0, 0.63, 0.6842105263157894, 0.75, 1.0, 0.25, 0.7368421052631579, 1.0, 1.0, 0.0, 0.7894736842105263, 1.0, 0.8, 0.1, 0.8421052631578947, 1.0, 0.6, 0.3, 0.894736842105263, 1.0, 0.67, 0.95, 0.9473684210526315, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0]"
        + ", 'ColorSpace': 0}\"";
    userLookupTable.setProperties(NeoProperty.fromString(propString));
  }
  return retString;
}
    
}// END CLASS COLORMAPTOOL
