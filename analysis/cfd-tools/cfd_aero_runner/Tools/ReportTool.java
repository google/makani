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

import java.util.*;
import star.base.neo.*;

import star.base.report.*;
import star.common.*;
import star.flow.*;

/**
 *
 * The purpose of class ReportTool is to make generating and obtaining
 * common STAR-CCM+ Reports much more straight forward. 
 * 
 * Consider it primarily a common usage API shortcut tool.
 * 
 */
public class ReportTool {

public static void reportToolPrint(Simulation tmpSim, String toOutputWindow){
  tmpSim.println(toOutputWindow);
}

// General
public static Report getReport(Simulation tmpSim, String repName){
  Report tmpReport = null;
  try{
    tmpReport=tmpSim.getReportManager().getReport(repName);
  }catch(NeoException e){
    reportToolPrint(tmpSim,"RepTool MSG: No Report "+repName
            +" exists! Likely failure!");
  }
  return tmpReport;
}

// Force
public static ForceReport forceReport(Simulation tmpSim,String repName, 
  CoordinateSystem repCsys, double[] repDirection,
  Collection<? extends NamedObject> repParts,
  Representation repRepresentation){
  ForceReport fRep;
  try{
    fRep = (ForceReport) tmpSim.getReportManager().getObject(repName);
  }catch(NeoException e){
    fRep = tmpSim.getReportManager().createReport(ForceReport.class); 
    fRep.setPresentationName(repName);
  }
  fRep.setCoordinateSystem(repCsys);
  fRep.getDirection().setComponents(
          repDirection[0], repDirection[1],repDirection[2]);
  fRep.setRepresentation(repRepresentation);
  fRep.getParts().setObjects(repParts);
  return fRep;
}

public static ForceReport aSyncforceReport(Simulation tmpSim,String repName, 
  CoordinateSystem repCsys, double[] repDirection,
  Collection<? extends NamedObject> repParts,
  Representation repRepresentation){
  ForceReport fRep;
  try{
    fRep = (ForceReport) tmpSim.getReportManager().getObject(repName);
  }catch(NeoException e){
    tmpSim.getReportManager().createReportNoResult(ForceReport.class);
    fRep = tmpSim.getReportManager().createReport(ForceReport.class);
    fRep.setPresentationName(repName);
  }
  fRep.setCoordinateSystem(repCsys);
  fRep.getDirection().setComponents(
          repDirection[0], repDirection[1],repDirection[2]);
  fRep.setRepresentation(repRepresentation);
  fRep.getParts().setObjects(repParts);
  return fRep;
}

public static ForceReport forceReport(Simulation tmpSim, String repName, 
    CoordinateSystem repCsys, double[] repDirection,
    Collection<? extends NamedObject> repParts,
    Representation repRepresentation, String pressOrSkinFric){
  ForceReport fRepC = forceReport(tmpSim, repName, repCsys, repDirection,
      repParts, repRepresentation);
  if(pressOrSkinFric.equals("pressure")){
    fRepC.getForceOption().setSelected(ForceReportForceOption.Type.PRESSURE);
  }else if(pressOrSkinFric.equals("shear")){
    fRepC.getForceOption().setSelected(ForceReportForceOption.Type.SHEAR);    
  }else{
    throw new NeoException("Report Tool, forceReport: "
        + "Option is not recognized. Choose either "
        + "\"pressure\" or \"shear\". ");
  }

  return fRepC;
}
  //
  // Moment
  public static MomentReport momentReport(Simulation tmpSim,String repName, 
          CoordinateSystem repCsys, double[] repOrigin, double[] repAxis,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
      MomentReport mRepC;
      try{
          mRepC = (MomentReport) tmpSim.getReportManager().getObject(repName);
      }catch(NeoException e){
          mRepC = tmpSim.getReportManager().createReport(MomentReport.class); 
          try{
            Thread.sleep(100);
          }catch(InterruptedException f){
            
          }
          mRepC.setPresentationName(repName);
      }
      mRepC.getDirection().setComponents(repAxis[0], repAxis[1], repAxis[2]);
      mRepC.getOrigin().setComponents(repOrigin[0], repOrigin[1], repOrigin[2]);
      mRepC.setCoordinateSystem(repCsys);
      mRepC.setRepresentation(repRepresentation);
      mRepC.getParts().setObjects(repParts);
      return mRepC;
  }
  //
  // Force Coefficients
  public static ForceCoefficientReport forceCoefReport(
          Simulation tmpSim,String repName, 
          CoordinateSystem repCsys, double[] repDirection,
          double refRho, double refVel, double refArea,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
    ForceCoefficientReport fRepC;
    try{
        fRepC = (ForceCoefficientReport) tmpSim.getReportManager().
                getObject(repName);
    }catch(NeoException e){
        fRepC = tmpSim.getReportManager().createReport(
                ForceCoefficientReport.class); 
        try{
          Thread.sleep(100);
        }catch(InterruptedException f){

        }
        fRepC.setPresentationName(repName);
    }
    fRepC.setCoordinateSystem(repCsys);
    fRepC.getDirection().setComponents(
        repDirection[0], repDirection[1], repDirection[2]);
    fRepC.getReferenceDensity().setDefinition("" + refRho);
    fRepC.getReferenceVelocity().setDefinition("" + refVel);
    fRepC.getReferenceArea().setDefinition("" + refArea);
    fRepC.setRepresentation(repRepresentation);
    fRepC.getParts().setObjects(repParts);
    return fRepC;
  }
  public static ForceCoefficientReport forceCoefReport(Simulation tmpSim,
      String repName, CoordinateSystem repCsys, double[] repDirection,
      double refRho, double refVel, double refArea, 
      Collection<? extends NamedObject> repParts,
      Representation repRepresentation, String pressOrSkinFric){
      ForceCoefficientReport fRepC = forceCoefReport(tmpSim, repName,
          repCsys, repDirection, refRho, refVel, refArea, repParts, 
          repRepresentation);
    if(pressOrSkinFric.equals("pressure")){
      fRepC.getForceOption().setSelected(ForceReportForceOption.Type.PRESSURE);  
    }else if(pressOrSkinFric.equals("shear")){
      fRepC.getForceOption().setSelected(ForceReportForceOption.Type.SHEAR);    
    }else{
      throw new NeoException("Report Tool, forceReport: "
          + "Option is not recognized. Choose either "
          + "\"pressure\" or \"shear\". ");
    }
    return fRepC;
  }

  // Moment Coefficient
  public static MomentCoefficientReport momentCoefReport(
          Simulation tmpSim,String repName, 
          CoordinateSystem repCsys, double[] repOrigin, double[] repAxis,
          double refRho, double refVel, double refArea, double refRad,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
    /*Method creates/modifies a moment coefficient report */
    MomentCoefficientReport mRepC;
    try{
        mRepC = (MomentCoefficientReport) tmpSim.getReportManager().
                getObject(repName);
    }catch(NeoException e){
        mRepC = tmpSim.getReportManager().
                createReport(MomentCoefficientReport.class); 
        try{
          Thread.sleep(100);
        }catch(InterruptedException f){

        }
        mRepC.setPresentationName(repName);
    }
    mRepC.getReferenceArea().setValue(refArea);
    mRepC.getReferenceVelocity().setValue(refVel);
    mRepC.getReferenceDensity().setValue(refRho);

    mRepC.getDirection().setComponents(repAxis[0], repAxis[1], repAxis[2]);
    mRepC.getOrigin().setComponents(repOrigin[0], repOrigin[1], repOrigin[2]);
    mRepC.getReferenceRadius().setValue(refRad);
    mRepC.setCoordinateSystem(repCsys);
    mRepC.setRepresentation(repRepresentation);
    mRepC.getParts().setObjects(repParts);
    return mRepC;
  }
  //
  //Surface Integral
  public static SurfaceIntegralReport surfIntegralReport(
          Simulation tmpSim,String repName, FieldFunction tmpFF,
          Collection<? extends Boundary> tmpBnd,
          Representation repRepresentation){
      SurfaceIntegralReport retRep;
      try{
          retRep = (SurfaceIntegralReport) tmpSim.getReportManager().
                  getObject(repName);
      }catch(NeoException e){
          retRep = tmpSim.getReportManager().createReport(
                  SurfaceIntegralReport.class); 
          retRep.setPresentationName(repName);
      }
      retRep.setRepresentation(repRepresentation);
      retRep.setFieldFunction(tmpFF);
      retRep.getParts().setObjects(tmpBnd);
      return retRep;
  }
  //
  //Surface Average
  public static AreaAverageReport surfAveReport(Simulation tmpSim,
          String repName, FieldFunction tmpFF,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
      AreaAverageReport surfAveRep;
      try{
          surfAveRep = (AreaAverageReport) tmpSim.getReportManager().
                  getObject(repName);
      }catch(NeoException e){
          surfAveRep = tmpSim.getReportManager().
                  createReport(AreaAverageReport.class); 
          surfAveRep.setPresentationName(repName);
      }
      surfAveRep.setRepresentation(repRepresentation);
      surfAveRep.setFieldFunction(tmpFF);
      surfAveRep.getParts().setObjects(repParts);
      return surfAveRep;
  }

  //Sum
  public static SumReport sumReport(Simulation tmpSim,String repName,
          FieldFunction tmpFF,
  Collection<? extends NamedObject> repParts,
  Representation repRepresentation){
      SumReport tmpRep;
      try{
          tmpRep = (SumReport) tmpSim.getReportManager().getObject(repName);
      }catch(NeoException e){
          tmpRep = tmpSim.getReportManager().createReport(SumReport.class); 
          tmpRep.setPresentationName(repName);
      }
      tmpRep.setRepresentation(repRepresentation);
      tmpRep.setFieldFunction(tmpFF);
      tmpRep.getParts().setObjects(repParts);
      return tmpRep;
  }
  public static ExpressionReport getExpressionReport(Simulation tmpSim,
          String repName){
  ExpressionReport tmpRep;
   try{
       tmpRep = (ExpressionReport) tmpSim.getReportManager().getObject(repName);
   }catch(NeoException e){
       tmpRep = tmpSim.getReportManager().createReport(ExpressionReport.class); 
       tmpRep.setPresentationName(repName);
   }

   return tmpRep;
}

  //Max
  public static MaxReport maxReport(Simulation tmpSim,String repName,
          FieldFunction tmpFF,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
      MaxReport retRep;
      try{
          retRep = (MaxReport) tmpSim.getReportManager().getObject(repName);
      }catch(NeoException e){
          retRep = tmpSim.getReportManager().createReport(MaxReport.class); 
          retRep.setPresentationName(repName);
      }
      retRep.setRepresentation(repRepresentation);
      //retRep.setScalar(tmpFF);
      retRep.setFieldFunction(tmpFF);
      retRep.getParts().setObjects(repParts);
      return retRep;
  }

  //Min
  public static MinReport minReport(Simulation tmpSim,String repName,
          FieldFunction tmpFF,
          Collection<? extends NamedObject> repParts,
          Representation repRepresentation){
      MinReport retRep;
      try{
          retRep = (MinReport) tmpSim.getReportManager().getObject(repName);
      }catch(NeoException e){
          retRep = tmpSim.getReportManager().createReport(MinReport.class); 
          retRep.setPresentationName(repName);
      }
      retRep.setRepresentation(repRepresentation);
      retRep.setFieldFunction(tmpFF);
      retRep.getParts().setObjects(repParts);
      return retRep;
  }
  //
  //HPC Reports
  public static void makeHPCReports(Simulation tmpSim, int iterationPerCheck){
      //
      CumulativeElapsedTimeReport wallTime;
      IteratorElapsedTimeReport itPerSec;
      //
      IterationMaximumMemoryReport resMem;
      IterationMaximumMemoryReport virtMem;
      IterationMaximumMemoryReport resWatermark;
      IterationMaximumMemoryReport virtWatermark;
      int itPerMemCheck=iterationPerCheck;
      Units memUnits = 
        ((Units) tmpSim.getReportManager().getSimulation().getUnitsManager().
                getObject("GiB"));
      //Wall time
      try{
          wallTime=(CumulativeElapsedTimeReport) tmpSim.getReportManager().
                  getReport("Wall Time");
      }catch(NeoException e){
          wallTime = 
            tmpSim.getReportManager().createReport(
                    CumulativeElapsedTimeReport.class);
          wallTime.setPresentationName("Wall Time");
      }
      wallTime.setUnits((Units) tmpSim.getReportManager().getSimulation().
              getUnitsManager().getObject("hr"));

      // Seconds per Iteration
      try{
          itPerSec = (IteratorElapsedTimeReport) tmpSim.getReportManager().
                  getReport("Seconds Per Iteration");
      }catch(NeoException e){
          itPerSec = 
            tmpSim.getReportManager().createReport(
                    IteratorElapsedTimeReport.class);
          itPerSec.setPresentationName("Seconds Per Iteration");
      }

      // Memory Management
      try{
          resMem =(IterationMaximumMemoryReport) tmpSim.getReportManager().
                  getReport("RES Memory");
      }catch(NeoException e){
          resMem = 
            tmpSim.getReportManager().createReport(
                    IterationMaximumMemoryReport.class);
          resMem.setPresentationName("RES Memory");
      }
      resMem.getMemoryReportMetricOption().setSelected(
              MemoryReportMetricOption.Type.RESIDENT);
      resMem.setUnits(memUnits);
      resMem.setSamplingFrequency(itPerMemCheck);
      //
      try{
          virtMem =(IterationMaximumMemoryReport) tmpSim.getReportManager().
                  getReport("VIRT Memory");
      }catch(NeoException e){
          virtMem = 
            tmpSim.getReportManager().createReport(
                    IterationMaximumMemoryReport.class);
          virtMem.setPresentationName("VIRT Memory");
      }
      virtMem.getMemoryReportMetricOption().setSelected(
              MemoryReportMetricOption.Type.VIRTUAL);
      virtMem.setUnits(memUnits);
      virtMem.setSamplingFrequency(itPerMemCheck);
      //
      try{
          resWatermark =(IterationMaximumMemoryReport) 
                  tmpSim.getReportManager().getReport("RES Watermark Memory");
      }catch(NeoException e){
          resWatermark = 
            tmpSim.getReportManager().
                    createReport(IterationMaximumMemoryReport.class);
          resWatermark.setPresentationName("RES Watermark Memory");
      }
      resWatermark.getMemoryReportMetricOption().
              setSelected(MemoryReportMetricOption.Type.RESIDENTHWM);
      resWatermark.setUnits(memUnits);
      resWatermark.setSamplingFrequency(itPerMemCheck);
      //
      try{
          virtWatermark =(IterationMaximumMemoryReport) tmpSim.
                  getReportManager().getReport("VIRT Watermark Memory");
      }catch(NeoException e){
          virtWatermark = 
            tmpSim.getReportManager().createReport(
                    IterationMaximumMemoryReport.class);
          virtWatermark.setPresentationName("VIRT Watermark Memory");
      }
      virtWatermark.getMemoryReportMetricOption().
              setSelected(MemoryReportMetricOption.Type.VIRTUALHWM);
      virtWatermark.setUnits(memUnits);
      virtWatermark.setSamplingFrequency(itPerMemCheck);

  }
  //
  // Grouping methods
  public static ClientServerObjectGroup getReportGroup(Simulation tmpSim,
          String groupName){
      ClientServerObjectGroup tmp;
      try{
          tmp=((ClientServerObjectGroup) tmpSim.getReportManager().
                  getGroupsManager().getObject(groupName));
      }catch(NeoException e){
          tmpSim.getReportManager().getGroupsManager().createGroup(groupName);
          try{
            Thread.sleep(100);
          }catch(InterruptedException f){
          }
          tmp=((ClientServerObjectGroup) tmpSim.getReportManager().
                  getGroupsManager().getObject(groupName));
      }

      return tmp;
  }
  public static void groupReports(Simulation tmpSim, String groupName,
          Collection<Report> toBeGrouped){
      //Groups
      getReportGroup(tmpSim,groupName).getGroupsManager().
              groupObjects(groupName, toBeGrouped, true);

  }

}
