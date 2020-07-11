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

import star.common.*;
import star.base.neo.*;
import star.base.report.Monitor;

public class CriterionTool {
  
  public static void criterionToolPrint(Simulation tmpSim,
      String toOutputWindow){
    tmpSim.println("CriterionTool MSG: " + toOutputWindow);
  }
  
  public static String criterionErrorMSG(String errorMSG){
    return "CriterionTool ERR: " + errorMSG;
  }
  
// General
public static SolverStoppingCriterion getCriterion(
    Simulation tmpSim, String critName){
  SolverStoppingCriterion tmpCrit = null;
  try{
    tmpCrit = tmpSim.getSolverStoppingCriterionManager().getObject(critName);
  }catch(NeoException e){
    throw new NeoException(criterionErrorMSG("Not found:" + critName + "!"));
  }
  return tmpCrit;
}

public static MonitorIterationStoppingCriterion getItMonStoppingCriterion(
    Simulation tmpSim, String critName){
  
  MonitorIterationStoppingCriterion tmpCrit;
  try{
    tmpCrit = 
        (MonitorIterationStoppingCriterion) getCriterion(tmpSim, critName);
  }catch(NeoException e){
    tmpCrit = tmpSim.getSolverStoppingCriterionManager()
        .createSolverStoppingCriterion(
            MonitorIterationStoppingCriterion.class, critName);
  }
  return tmpCrit;
}

// Solver Stopping Criterion
public static SolverStoppingCriterion setItMonStdDevCriteria(
    Simulation tmpSim, Monitor useThisMonitor, double stdDev, int nSamples){
  MonitorIterationStoppingCriterion itMonStoppingCriterion;
  String appEnd = " Criterion";
  String critName = useThisMonitor.getPresentationName()+appEnd;
  
  // Try/catch is to avoid setting monitor if this is already set up.
  try{
    itMonStoppingCriterion =
        (MonitorIterationStoppingCriterion) getCriterion(tmpSim, critName);
  }catch(NeoException e){
    itMonStoppingCriterion = getItMonStoppingCriterion(tmpSim, critName);
    itMonStoppingCriterion.setMonitor(useThisMonitor);
  }
  // Set it up to track and monitor the standard deviation.
  itMonStoppingCriterion.getCriterionOption()
      .setSelected(
          MonitorIterationStoppingCriterionOption.Type.STANDARD_DEVIATION);
  MonitorIterationStoppingCriterionStandardDeviationType itMonType = 
      ((MonitorIterationStoppingCriterionStandardDeviationType) 
          itMonStoppingCriterion.getCriterionType());
  itMonType.getStandardDeviation().setValue(stdDev);
  itMonType.setNumberSamples(nSamples);
  //
  return itMonStoppingCriterion;
}

// Solver Stopping Criterion
public static SolverStoppingCriterion setItMonRelativeChangeCriteria(
    Simulation tmpSim, Monitor useThisMonitor, double changePct){
  MonitorIterationStoppingCriterion itMonStoppingCriterion;
  String appEnd = " Criterion";
  String critName = useThisMonitor.getPresentationName()+appEnd;
  
  // Try/catch is to avoid setting monitor if this is already set up.
  try{
    itMonStoppingCriterion =
        (MonitorIterationStoppingCriterion) getCriterion(tmpSim, critName);
  }catch(NeoException e){
    itMonStoppingCriterion = getItMonStoppingCriterion(tmpSim, critName);
    itMonStoppingCriterion.setMonitor(useThisMonitor);
  }
  // Set it up to track and monitor the standard deviation.
  itMonStoppingCriterion.getCriterionOption()
      .setSelected(
          MonitorIterationStoppingCriterionOption.Type.RELATIVE_CHANGE);
  
  MonitorIterationStoppingCriterionRelativeChangeType itMonType = 
      ((MonitorIterationStoppingCriterionRelativeChangeType) 
          itMonStoppingCriterion.getCriterionType());
  itMonType.setRelativeChange(changePct);

  //
  return itMonStoppingCriterion;
}

 
}
