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
package AeroToolKit;

import java.util.*;
import java.text.*;
import star.common.*;
import star.base.neo.*;
import star.base.report.*;

import Naming.*;

import Tools.DerivedPartTool;
import Tools.MonitorTool;
import Tools.ReportTool;
import Tools.SimTool;


public class PitotProbe{
  String pitotName;
  Part pitotPart;
  NamingConvention globalNames;
  
  Representation proxyRep;
  
  //Geometry
  double[] pitotLoc;
  
  //Derived Parts
  
  // Reports
  ArrayList<Report> pitotReports = new ArrayList();
  
  // Monitors 
  ArrayList<Monitor> pitotMonitors = new ArrayList();
  
  // Coordinate System
  LabCoordinateSystem labCsys;
  CartesianCoordinateSystem bodyCsys;
  CartesianCoordinateSystem inletCsys;
  
  public PitotProbe(Simulation tmpSim, CoordinateSystem rootCsys,Collection<? extends NamedObject> tmpParts,
    String pitotDescriptor, double[] xyzPos){
    
    // Initialize Global Names
    this.globalNames = new NamingConvention();
    this.proxyRep    = tmpSim.getRepresentationManager()
                        .getObject(globalNames.getProxyName());

    // gather Coordinate System w/ the name & make sure coordinate system will rotate w/ part
    long initTime= System.nanoTime();
    this.labCsys = tmpSim.getCoordinateSystemManager()
                     .getLabCoordinateSystem();
    this.bodyCsys = (CartesianCoordinateSystem) labCsys
                     .getLocalCoordinateSystemManager().getObject(globalNames.getBodyCsysName());
    this.inletCsys = (CartesianCoordinateSystem) bodyCsys
                     .getLocalCoordinateSystemManager().getObject(globalNames.getInletCsysName());
    this.pitotLoc = xyzPos;
    
    // Pitot Name
    NumberFormat formattedDouble=new DecimalFormat("0.###E0");
    String xStr=formattedDouble.format(xyzPos[0]);
    String yStr=formattedDouble.format(xyzPos[1]);
    String zStr=formattedDouble.format(xyzPos[2]);
    String locString = xStr+"_"+yStr+"_"+zStr;
    
    //deal with pitot + descriptor for objects that already exist in the simulation
    if(pitotDescriptor.contains(locString)){
      int tmpIndx = pitotDescriptor.indexOf(locString);
      pitotDescriptor = pitotDescriptor.substring(1, tmpIndx-1);
      tmpSim.println("    ...found pitot ID "+pitotDescriptor+"already exsts");
    }
    this.pitotName = "Pitot_" + pitotDescriptor + "_" + locString + " " + rootCsys.getPresentationName();
    
    long endTime = System.nanoTime();
    tmpSim.println("     ...time for names: "+(endTime-initTime)/1E9);
    
    // Associated Derived Part
    long initTime1 = System.nanoTime();
    this.pitotPart = DerivedPartTool.makePointPart(tmpSim, pitotName,rootCsys, xyzPos);
    long endTime1 = System.nanoTime();
    tmpSim.println("     ...time for making PointPart: "+(endTime1-initTime1)/1E9);
    
    // Reports
    long initTime2 = System.nanoTime();
    long initTime2_1 = System.nanoTime();
    this.pitotReports = makeProbeReports(tmpSim,pitotName,pitotPart,bodyCsys,inletCsys,proxyRep);
    long endTime2_1 = System.nanoTime();
    tmpSim.println("     ...time for actually making reports: "+(endTime2_1-initTime2_1)/1E9);
    long endTime2 = System.nanoTime();
    long initTime2_2 = System.nanoTime();
    ReportTool.groupReports(tmpSim,pitotName,pitotReports);
    long endTime2_2 = System.nanoTime();
    tmpSim.println("     ...time for grouping reports: "+(endTime2_2-initTime2_2)/1E9);
    tmpSim.println("     ...time for making reports: "+(endTime2-initTime2)/1E9);
    
    long initTime3 = System.nanoTime();
    setProbeFieldObjects(tmpParts);

    ReportTool.groupReports(tmpSim,pitotName,pitotReports);
    long endTime3 = System.nanoTime();
    tmpSim.println("     ...time for set objects: "+(endTime3-initTime3)/1E9);    
    
    // Monitors
    long initTime4 = System.nanoTime();
    this.pitotMonitors = makePitotIterationMonitors(tmpSim,pitotName,pitotReports);
    long endTime4 = System.nanoTime();
    tmpSim.println("     ...time for set monitors: "+(endTime4-initTime4)/1E9);        
    // Plots
    
  }

  
  private void setProbeFieldObjects(Collection<? extends NamedObject> tmpParts){
    //set pitot regions
    pitotPart.getInputParts().setObjects(tmpParts);
  }
  public ArrayList<Report> getProbeReports(){
    return pitotReports;
  }
  public Part getProbePart(){
    return pitotPart;
  }
  
  private static ArrayList<Report> makeProbeReports(Simulation tmpSim,String probeName, Part thisPart,
      CoordinateSystem bodyCsys, CoordinateSystem inletCsys, Representation thisRep){
    // Workhorse variables
    ArrayList<Report> tmpReports = new ArrayList();
    MaxReport tmpReport;
    String tmpReportName;
    
    // WIND & BODY VELOCITY REPORTS
    String humanDir="UKNOWN";
    VectorComponentFieldFunction tmpVCFF;
    PrimitiveFieldFunction velocityFF = 
            ((PrimitiveFieldFunction) tmpSim.
                    getFieldFunctionManager().getFunction("Velocity"));

    for(int i=0;i<3;i++){
      // inlet u, v, w
      if(i==0) humanDir = "i";
      if(i==1) humanDir = "j";
      if(i==2) humanDir = "k";
      tmpReportName=probeName+"_"+inletCsys.getPresentationName()+"_"+humanDir;
      tmpVCFF =    
        ((VectorComponentFieldFunction) velocityFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(i));
      tmpReport = 
        ReportTool.maxReport(tmpSim, tmpReportName, tmpVCFF, Collections.singleton(thisPart), thisRep);
      tmpReports.add(tmpReport);
      
      //
      tmpReportName=probeName+"_"+bodyCsys.getPresentationName()+"_"+humanDir;
      tmpVCFF =    
        ((VectorComponentFieldFunction) velocityFF.getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(i));
      tmpReport = 
        ReportTool.maxReport(tmpSim, tmpReportName, tmpVCFF, Collections.singleton(thisPart), thisRep);
      tmpReports.add(tmpReport);
    }

    // PRESSURES
    //Static Pressure
    String ffFQName = "StaticPressure";
    tmpReportName=probeName+"_"+ffFQName;
    PrimitiveFieldFunction staticPressFF = 
      ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction(ffFQName));
    tmpReport = 
        ReportTool.maxReport(tmpSim, tmpReportName, staticPressFF, Collections.singleton(thisPart), thisRep);
    tmpReports.add(tmpReport);
    return tmpReports;
    
  }
  
  private static ArrayList<Monitor> makePitotIterationMonitors(Simulation tmpSim,
    String thisName, ArrayList<Report> repList){
    ArrayList<Monitor> monList = new ArrayList();
    for(Report tmpRep:repList){
      Monitor tmpMon;
      tmpMon = MonitorTool.reportIterationMonitor(tmpSim,tmpRep,(int) 1.0E4,1,0);
      monList.add(tmpMon);
    }
    // group up and exit
    MonitorTool.groupMonitors(tmpSim, thisName, monList);
    
    return monList;
  }

  private void makePitotPlots(Simulation tmpSim){
    
  }

  
  
  public void sortReports(){
    //Sorts reports by alphabetical order
    List<Report> tmpList = pitotReports;
    Collections.sort(tmpList, new Comparator<Report>() {
      @Override
      public int compare(Report one, Report other) {
      return one.getPresentationName().compareTo(other.getPresentationName());
      }
    }); 
  }
  public void sortMonitors(){
    //Sorts monitors by alphabetical order
    List<Monitor> tmpList = pitotMonitors;
    Collections.sort(tmpList, new Comparator<Monitor>() {
      @Override
      public int compare(Monitor one, Monitor other) {
      return one.getPresentationName().compareTo(other.getPresentationName());
      }
    });
  }

  public String getName(){
    return pitotName;
  }
  
  public ArrayList<Monitor> getMonitors(){
    return pitotMonitors;
  }
  
  // FILE I/O METHODS
  public String dataToCSV(Simulation tmpSim, int numSamples){
    double aveVal = 0.0;
    double stdDev = 0.0;
    String retStr = "";
    String debugList = pitotName;
    for(Monitor tmpMonitor:pitotMonitors){
      String tmpName=tmpMonitor.getPresentationName();
      aveVal = MonitorTool.getLastMonitorSamplesMean(tmpSim,tmpName,numSamples);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(tmpSim,tmpName,numSamples,aveVal);
      
      retStr = retStr+","+aveVal+","+stdDev;
      debugList=debugList+","+tmpName;
    }
    return retStr;
  }

// End of class PitotProbe
} //End of class PitotProbe


