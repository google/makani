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

package GeometricObject;

import java.util.*;

import star.common.*;
import star.base.report.*;

import star.flow.MomentCoefficientReport;
import star.flow.MomentReport;

import Tools.*;
import star.flow.ForceCoefficientReport;

public class DeflectableControlSurface extends AerodynamicSurface {

CartesianCoordinateSystem localCsys;

// Meshing configuration information
double deflectionAngle; // [deg]

// Necessary Reports
ArrayList<Report> localReports;

// Necessary Iteration Monitors
ArrayList<Monitor> localIterMonitors;

//Optional Unsteady Monitors
ArrayList<Monitor> localUnsMonitors;


public DeflectableControlSurface(Simulation tmpSim,
        ArrayList<PartSurface> geomSurfs,int preFixSize,
        CartesianCoordinateSystem cadCsys) {
    //Instaniate superclass
    super(tmpSim,geomSurfs,preFixSize);

    this.cadCsys=cadCsys;

    localReports = new ArrayList();
    localIterMonitors = new ArrayList();
    localUnsMonitors = new ArrayList();

}


// Printing out methods
@Override
public String csvData(Simulation tmpSim){
  String tmpStr = "";
  double meanVal;
  double stddevVal;
  String repType;

  //Name
  tmpStr = tmpStr + cntrlName;

  //Rotation angle
  tmpStr = tmpStr + "," + deflectionAngle;

  //Meshing data
  tmpStr = tmpStr + "," + getCustomTargetSurfaceSize();
  tmpStr = tmpStr + "," + getCustomMinSurfaceSize();
  tmpStr = tmpStr + "," + getNPtsOnCircle();
  tmpStr = tmpStr + "," + getCustomNumPrismLayers();
  tmpStr = tmpStr + "," + getCustomPrismAbsThickness();
  tmpStr = tmpStr + "," + getCustomFirstCellThickness();

  //Wall y+ data
  for(Report tmpRep : wallYplusReports){
    meanVal = getMeanReportItVal(tmpRep, itPostFix, statSamples);
    tmpStr = tmpStr + "," + meanVal;
  }

  //Forces and Moments data
  repType = this.getStarPrefix() + bodyStr;
  for(int i = 0; i <= 2; i++){
    meanVal = getMeanForceCoef(tmpSim, repType, i, statSamples);
    stddevVal = getStdDevForceCoef(tmpSim, repType, i, statSamples, meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;
    
    // Force from pressure only.
    meanVal = getMeanPressureForceCoef(tmpSim, repType, i, statSamples);
    stddevVal = getStdDevPressureForceCoef(
        tmpSim, repType, i, statSamples, meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;

    // Force from shear only.
    meanVal = getMeanShearForceCoef(tmpSim, repType, i, statSamples);
    stddevVal = getStdDevShearForceCoef(
        tmpSim, repType, i, statSamples, meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;
  }

  repType=this.getStarPrefix() + aeroStr;
  for(int i=0; i<=2 ; i++){
    meanVal = getMeanForceCoef(tmpSim,repType,i,statSamples);
    stddevVal = getStdDevForceCoef(tmpSim,repType,i,statSamples,meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;
    
    // Force from pressure only.
    meanVal = getMeanPressureForceCoef(tmpSim, repType, i, statSamples);
    stddevVal = getStdDevPressureForceCoef(
        tmpSim, repType, i, statSamples, meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;
    
    // Force from shear only.
    meanVal = getMeanShearForceCoef(tmpSim, repType, i, statSamples);
    stddevVal = getStdDevShearForceCoef(
        tmpSim, repType, i, statSamples, meanVal);
    tmpStr = tmpStr + "," + meanVal + "," + stddevVal;
  }

  //=========================================
  //CUSTOM DATA DO NOT ACCIDENTALLY ERASE!
  //=========================================
  //Local Moment Data
  meanVal=MonitorTool.getLastMonitorSamplesMean(
      tmpSim, super.getStarPrefix() + "Local " + cntrlName + " - CmX" + globalNames.getItPostFix(),
      statSamples);
  stddevVal=MonitorTool.getLastMonitorSamplesStDev(
      tmpSim, super.getStarPrefix() + "Local " + cntrlName + " - CmX" + globalNames.getItPostFix(),
      statSamples,meanVal);
  tmpStr+=","+meanVal+","+stddevVal;
  return tmpStr;
}

@Override
public void setReferenceValues(double refRho, double refVel, double refArea,
    double[] refRadii){
    super.setReferenceValues(refRho, refVel, refArea, refRadii);
    setLocalRefVel(refVel);
    setLocalRefRho(refRho);
    setLocalRefMomR(refRadii);
    setLocalRefArea(refArea);


}
public void setLocalRefVel(double newVal){
  for(Report tmpReport:localReports){
    if(tmpReport instanceof ForceCoefficientReport){
      ((ForceCoefficientReport) tmpReport)
          .getReferenceVelocity().setValue(newVal);
    }
    if(tmpReport instanceof MomentCoefficientReport){
      ((MomentCoefficientReport) tmpReport)
          .getReferenceVelocity().setValue(newVal);
    }
  }
}
public void setLocalRefRho(double newVal){
  for(Report tmpReport:localReports){
    if(tmpReport instanceof ForceCoefficientReport){
      ((ForceCoefficientReport) tmpReport)
          .getReferenceDensity().setValue(newVal);
    }
    if(tmpReport instanceof MomentCoefficientReport){
      ((MomentCoefficientReport) tmpReport)
          .getReferenceDensity().setValue(newVal);
    }
  }
}
public void setLocalRefMomR(double[] newVals){
  for(Report tmpReport:localReports){
    if(tmpReport instanceof MomentCoefficientReport){
      double xAxisVal =
          ((MomentCoefficientReport) tmpReport)
              .getDirection().getInternalVector().getComponent(0);
      double yAxisVal =
          ((MomentCoefficientReport) tmpReport)
              .getDirection().getInternalVector().getComponent(1);
      double zAxisVal =
          ((MomentCoefficientReport) tmpReport)
              .getDirection().getInternalVector().getComponent(2);
      if(Math.abs(xAxisVal-1.0) > 1e-5){
          ((MomentCoefficientReport) tmpReport)
              .getReferenceRadius().setValue(newVals[0]);
      }else if(Math.abs(yAxisVal-1.0)>1e-5){
          ((MomentCoefficientReport) tmpReport)
              .getReferenceRadius().setValue(newVals[1]);
      }else if(Math.abs(zAxisVal-1.0)>1e-5){
          ((MomentCoefficientReport) tmpReport)
              .getReferenceRadius().setValue(newVals[2]);
      }
    }
  }
}
public void setLocalRefArea(double newVal){
  for(Report tmpReport : localReports){
    if(tmpReport instanceof ForceCoefficientReport){
      ((ForceCoefficientReport) tmpReport).getReferenceArea().setValue(newVal);
    }
    if(tmpReport instanceof MomentCoefficientReport){
      ((MomentCoefficientReport) tmpReport).getReferenceArea().setValue(newVal);
    }
  }
}


//Coordinate Systems
public CartesianCoordinateSystem getRotationCsys(){
  return localCsys;
}
public double[] getRotationAxis(){
  return localCsys.getBasis0().toDoubleArray();
}

public void setBodyCsys(Simulation tmpSim, 
    CartesianCoordinateSystem newBodyCsys){
  this.bodyCsys = newBodyCsys;
  this.localCsys =
      SimTool.equalLevelCoordToNestedCoord(tmpSim, cadCsys, bodyCsys);
}
public CartesianCoordinateSystem getLocalCsys(){
  return this.localCsys;
}

// Mesh Configuration
public double getDeflectionAngle(){
  /* Method returns current value stored as deflection Angle */
  return deflectionAngle;
}
public void setDeflectionAngle(double newVal){
  deflectionAngle=newVal;
}
// Local Reports

@Override
public void makeReports(Simulation tmpSim){
  super.makeReports(tmpSim);
  //Local stuff
  makeLocalMomentReports(tmpSim,localCsys);
}
public double getLocalMoment(Simulation tmpSim){
  return ((MomentCoefficientReport) ReportTool
          .getReport(tmpSim, "Local " + cntrlName + " - MX"))
      .getReportMonitorValue();
}
public ArrayList<Report> getLocalReports(){
    return localReports;
}
private void makeLocalMomentReports(Simulation tmpSim, CoordinateSystem relCsys){
    //Declare
    MomentReport mX; MomentCoefficientReport cMX;
    //Moment Report
    mX = ReportTool.momentReport(tmpSim, 
        super.getStarPrefix() + "Local " + cntrlName + " - MX",
        relCsys, zeroOrigin, xVector, geomSurfs, proxyRep);
    if(!localReports.contains(mX)) localReports.add(mX);
    //Moment Coefficient Report
    cMX=ReportTool.momentCoefReport(tmpSim,
        super.getStarPrefix() + "Local " + cntrlName + " - CmX",
        relCsys, zeroOrigin, zVector, refRho, refVel, refArea, refMomR[0],
        geomSurfs, proxyRep);
    if(!localReports.contains(cMX)) localReports.add(cMX);
}
@Override
public ArrayList<Monitor> getAllMonitors(){
  ArrayList<Monitor> allMonitors = new ArrayList();
  allMonitors.addAll(super.getAllMonitors());
  allMonitors.addAll(localIterMonitors);
  allMonitors.addAll(localUnsMonitors);
  return allMonitors;
}

@Override
public void TagReports(Simulation tmpSim, Collection<Tag> theseTags){
  super.TagReports(tmpSim, theseTags);
  for(Report tmpReport : localReports){
    TagTool.addTags(tmpSim, tmpReport, theseTags);
  }
}


//Local Monitors
@Override
public void makeIterationMonitors(Simulation tmpSim, int plotSamplesLimit,int freq, int startIT){
  super.makeIterationMonitors(tmpSim,plotSamplesLimit, freq, startIT);
  localIterMonitors.addAll(AerodynamicSurface.generateIterationMonitors(
      localReports, plotSamplesLimit, freq, startIT));
}
@Override
public void makeUnsteadyMonitors(Simulation tmpSim, int plotSamplesLimit,int freq,int startTS){
  super.makeUnsteadyMonitors(tmpSim, plotSamplesLimit, freq, startTS);
  localUnsMonitors.addAll(
      AerodynamicSurface.generateIterationMonitors(localReports, 
          plotSamplesLimit,freq,startTS)
      );
}
///
}// End DeflectableControlSurface
