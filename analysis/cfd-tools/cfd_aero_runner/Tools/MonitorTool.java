/*
    Copyright 2020 Makani Technologies LLC

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

import Naming.*;
import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.report.*;

public class MonitorTool {
  static NamingConvention globalNames = new NamingConvention();

  private static void printMonitorTool(Simulation tmpSim,
          String toOutputWindow){
    tmpSim.getMonitorManager().getSimulation().println(toOutputWindow);
  }

  //=========================================
  //STAR-CCM+ tools to make life nicer
  //=========================================
  public static void groupResidualMonitors(Simulation tmpSim){
      ArrayList<ResidualMonitor> retList=new ArrayList();
      for(Monitor tmp:tmpSim.getMonitorManager().getMonitors()){
          if(tmp instanceof ResidualMonitor){
              retList.add((ResidualMonitor) tmp);
          }
      }
      getMonitorGroup(tmpSim,"Residuals").getGroupsManager()
              .groupObjects("Residuals",
                  new NeoObjectVector(retList.toArray()), true);
  }

  public static ReportMonitor getReportMonitor(Simulation tmpSim, String name){
      Monitor tmpMon=null;
      try{
          return (ReportMonitor) tmpSim.getMonitorManager().getMonitor(name);
      }catch(NeoException e){
          printMonitorTool(tmpSim,"Mon Tool: Report Monitor "+name
                  +" does not exist! Will likely fail!");
          return (ReportMonitor) tmpMon;
      }
  }

  //=========================================
  // DATA Collection Methods
  //=========================================
  //Methods to create iteration or unsteady monitors
  public static Monitor reportIterationMonitor(Simulation tmpSim,
          Report repInput,int plotLimit,
                                          int itFreq,int startIt){
      /* Create a iteration based Report Monitor */
      ReportMonitor repMonitor;
      String monName = repInput.getPresentationName()
              +globalNames.getItPostFix();
      try{
          repMonitor = (ReportMonitor) tmpSim.getMonitorManager().
                  getObject(monName);
      }catch(NeoException e){
          repMonitor = repInput.createMonitor();
          repMonitor.setPresentationName(monName);
      }
      repMonitor.setPlotLimit(plotLimit);
      StarUpdate starUpdater = repMonitor.getStarUpdate();
      IterationUpdateFrequency itUpdateFreq = 
          starUpdater.getIterationUpdateFrequency();
      itUpdateFreq.setIterations(itFreq);
      itUpdateFreq.setStart(startIt);    
      return repMonitor;
  }
  public static Monitor reportUnsteadyMonitor(Simulation tmpSim, Report stdyRep,
          int plotLimit, int tsFreq,int startTS){
      String stdyMonName=stdyRep.getPresentationName();
      ReportMonitor stdyRepMon=(ReportMonitor) stdyRep.getSimulation()
              .getMonitorManager().getObject(stdyMonName+
                      globalNames.getItPostFix());
      String unsName=stdyMonName.substring(0,stdyMonName.length())+
              globalNames.getUnsPostFix();
      //does this already exist
      ReportMonitor unsRepMon;
      try{
          unsRepMon = (ReportMonitor) tmpSim.getMonitorManager().
                  getObject(unsName);
      }catch(NeoException e){
          unsRepMon = ((ReportMonitor) stdyRepMon).getReport().createMonitor();
          unsRepMon.setPresentationName(unsName);
      }
      unsRepMon.copyProperties(stdyRepMon);
      unsRepMon.getStarUpdate().getUpdateModeOption()
          .setSelected(StarUpdateModeOption.Type.TIMESTEP);
      unsRepMon.getStarUpdate().getTimeStepUpdateFrequency().
              setTimeSteps(tsFreq);
      return unsRepMon;
  }
  //turn on/off any Report Based Monitor
  public static void disableAllReportBasedMonitors(Simulation tmpSim){
    for(Monitor tmpMon:tmpSim.getMonitorManager().getObjects()){
      if(tmpMon instanceof ReportMonitor){
        tmpMon.setEnabled(false);
      }
    }
  }
  public static void enableAllReportBasedMonitors(Simulation tmpSim){
    for(Monitor tmpMon:tmpSim.getMonitorManager().getObjects()){
      if(tmpMon instanceof ReportMonitor){
        tmpMon.setEnabled(true);
      }
    }
  }
  // control monitor iteration start value
  public static void setMonitorStartIteration(ReportMonitor repMon,
          int startIt){
    IterationUpdateFrequency iterationUpdateFrequency = 
      repMon.getStarUpdate().getIterationUpdateFrequency();
    iterationUpdateFrequency.setStart(startIt);
  }
  public static void setMonitorIterationFrequency(ReportMonitor repMon,
          int itFrequency){
    IterationUpdateFrequency iterationUpdateFrequency = 
      repMon.getStarUpdate().getIterationUpdateFrequency();
    iterationUpdateFrequency.setIterations(itFrequency);
  }
  // coarsen/refine Monitor frequency
  public static void setAllReportBasedMonitorFrequency(Simulation tmpSim,
          int itFrequency){
    for(Monitor tmpMon:tmpSim.getMonitorManager().getObjects()){
      if(tmpMon instanceof ReportMonitor){
        setMonitorIterationFrequency(((ReportMonitor) tmpMon),itFrequency);
      }
    }
  }

  //=========================================
  // STATISTICS ON MONITOR VALUES
  //=========================================
  public static double getLastMonitorSamplesStDev(Simulation tmpSim,
          String monName,int monSamples,
          double discMean){
      try{
          double[] tmpVec;
          ReportMonitor tmpMon = (ReportMonitor)tmpSim.getMonitorManager().
                  getMonitor(monName);
          tmpVec=tmpMon.getAllYValues();

          if(monSamples>tmpVec.length){
              printMonitorTool(tmpSim,
                      "MonTool MSG: Not enough stddev samples for "+monName);
              return 1.e38;
          }else{
              double squareSum =0.0;
              int firstIndx = (tmpVec.length-monSamples); 
              for(int i=0;i<=(monSamples-1);i++){
                  squareSum=squareSum+(tmpVec[firstIndx+i]-discMean)*(
                          tmpVec[firstIndx+i]-discMean);
              }
              return Math.sqrt(squareSum/monSamples);
          }
      }catch(NeoException e){
          printMonitorTool(tmpSim,"MonTool MSG: no ReportMonitor called: "+
                  monName);
          return 1.e38;
      }
  }
  public static double getLastMonitorSamplesMean(Simulation tmpSim,
          String monName,int monSamples){
    try{
        double[] tmpVec;
        ReportMonitor tmpMon = (ReportMonitor) tmpSim.getMonitorManager().
                getMonitor(monName);
        tmpVec=tmpMon.getAllYValues();
        tmpMon.getAllYValues();
        if(monSamples>tmpVec.length){
            printMonitorTool(tmpSim,"MonTool MSG: Not enough mean samples for "
                    +monName);
            return 1.e38;
        }else{
            int firstIndx = (tmpVec.length-monSamples);
            double tmpSum=0.0;
            for(int i=0;i<=(monSamples-1);i++){
             tmpSum=tmpSum+tmpVec[firstIndx+i];
            }
            return tmpSum/(monSamples);
        }
    }catch(NeoException e){
        printMonitorTool(tmpSim,"Warning, no ReportMonitor called: "+monName);
        return 1.e38;
    }
  }

  //=========================================
  // GENERAL UI CLEANUP
  //=========================================
  public static ClientServerObjectGroup getMonitorGroup(Simulation tmpSim,
          String groupName){
      ClientServerObjectGroup tmp;
      try{
          tmp=((ClientServerObjectGroup) tmpSim.getMonitorManager()
                  .getGroupsManager().getObject(groupName));
      }catch(NeoException e){
          tmpSim.getMonitorManager().getGroupsManager().createGroup(groupName);
          try{
            Thread.sleep(100);
          }catch(InterruptedException f){

          }

          tmp=((ClientServerObjectGroup) tmpSim.getMonitorManager()
                  .getGroupsManager().getObject(groupName));
      }
      return tmp;
  }
  public static void groupMonitors(Simulation tmpSim,String groupName,
          ArrayList<Monitor> toBeGrouped){
      //Groups
      getMonitorGroup(tmpSim,groupName).getGroupsManager().groupObjects(groupName,
        toBeGrouped, true);
  }

}




