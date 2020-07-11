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
import star.flow.ForceReport;
import star.flow.MomentReport;
import star.flow.ForceCoefficientReport;
import star.flow.MomentCoefficientReport;

import Tools.*;
import Naming.*;
import MeshTools.*;
import java.io.File;
import star.base.neo.NeoException;
import star.meshing.SurfaceCustomMeshControl;
import star.vis.*;


/**
 * Aerodynamic Surface Class
 *  Broad construction of any aerodynamic surface in a simulation
 * 
 */
public class AerodynamicSurface {
// CLASS VARIABLES
static Representation proxyRep;
static NamingConvention globalNames;
final double SMALL_EPS = 1.0E-6;

//axis shorthand
static double[] zeroOrigin = {0.0, 0.0, 0.0};
static double[] xVector    = {1.0, 0.0, 0.0};
static double[] yVector    = {0.0, 1.0, 0.0};
static double[] zVector    = {0.0, 0.0, 1.0};

//Reporting Types
static String bodyStr="Body";
static String aeroStr="Aero";

//Statistics
static int statSamples=500;
static int[] xyRes={2000,1000};

// INSTANCE VARIABLES
String starObjPreFix = "";

//Geometry Information
String cntrlName="";
GeometryPart geomPart;
Collection<PartSurface> geomSurfs = new ArrayList();
int preFixSize = 4;
String preFix = "xxxx";

//Region
Region parentRegion;

// Coordinate Systems
String bodyCsysName;
String inletCsysName;
String windCsysName;
LabCoordinateSystem labCsys;
CartesianCoordinateSystem cadCsys;
CartesianCoordinateSystem bodyCsys;
CoordinateSystem inletCsys;

// Reference Values
double[] refMomR; //      [m] reference Moment Radius about {x,y,z}
double refArea;   //     [m2] reference Area
double refVel;    //    [m/s] reference Velocity
double refRho;    //  [kg/m3] reference Density
double refLengthScale; // [m] reference Length Scale

// Meshing information
SurfaceCustomMeshControl surfCustomMeshSettings;
SurfaceCustomMeshControl surfCustomMeshSettingsTE;
SurfaceCustomMeshControl volCustomMeshSettings;
SurfaceCustomMeshControl volCustomMeshSettingsTE;

//Meshing flags
boolean useCustomSurfaceMeshSettings=false;
double  customTargetSurfaceSize=-1.;
double  customMinSurfaceSize=-1.;
double  customNPtsOnCircle=-1.;

boolean useCustomVolumeMeshSettings=false;
double  customFirstCellThickness=-1.;
double  customPrismThickness=-1.;
int     customPrismLayers=-1;

// Necessary Reports
ArrayList<Report> aeroReports = new ArrayList();
ArrayList<Report> bodyReports = new ArrayList();
ArrayList<Report> wallYplusReports = new ArrayList();

//Monitors
String itPostFix;
String unsPostFix;

// Necessary Iteration Monitors
ArrayList<Monitor> aeroIterMonitors = new ArrayList();
ArrayList<Monitor> bodyIterMonitors = new ArrayList();
ArrayList<Monitor> wallYplusIterMonitors = new ArrayList();

// Optional Unsteady Monitors
ArrayList<Monitor> aeroUnsMonitors = new ArrayList();
ArrayList<Monitor> bodyUnsMonitors = new ArrayList();

// Plots
ArrayList<StarPlot> allPlots = new ArrayList();
ArrayList<StarPlot> allUserPlots = new ArrayList();
ArrayList<StarPlot> allCFDPlots = new ArrayList();

// Scenes
double[] surfFocalPt = {0.,0.,0.};
double objectSize  = 1.0;
Scene genericScene;  //used for generating generic object images

ArrayList<VisView> objectViews = new ArrayList();
ArrayList<Scene> allScenes = new ArrayList();
ArrayList<Scene> allUserScenes = new ArrayList();
ArrayList<Scene> allUserVRScenes = new ArrayList();
ArrayList<Scene> allCFDScenes = new ArrayList();
ArrayList<Scene> allCFDVRScenes = new ArrayList();
    
public AerodynamicSurface(Simulation inputSimu,
        ArrayList<PartSurface> geomSurfs,int preFixSize){

  //Familiarize
  globalNames = new NamingConvention();
  proxyRep    = inputSimu.getRepresentationManager()
                .getObject(globalNames.getProxyName());

  //Geometry
  this.geomSurfs=geomSurfs;
  this.geomPart=geomSurfs.get(0).getPart(); //all surfs better have XXXX
  this.cntrlName=geomSurfs.get(0).getPresentationName();
  this.preFixSize=preFixSize;
  this.preFix=cntrlName.substring(0,preFixSize-1);

  //Monitor stuff
  this.itPostFix=globalNames.getItPostFix();
  this.unsPostFix=globalNames.getUnsPostFix();

  //Coordinate System Stuff
  try{
    this.bodyCsysName  = globalNames.getBodyCsysName();
    this.inletCsysName = globalNames.getInletCsysName();
    this.windCsysName  = globalNames.getWindCsysName();
    this.labCsys=inputSimu.getCoordinateSystemManager()
            .getLabCoordinateSystem();
    this.bodyCsys=(CartesianCoordinateSystem) labCsys
            .getLocalCoordinateSystemManager().getObject(bodyCsysName);
    this.inletCsys=bodyCsys.getLocalCoordinateSystemManager().
            getObject(inletCsysName);
  }catch(NeoException e){
  }

}

//====================
// PUBLIC METHODS
//====================
// STAR-CCM+ Special Object Prefix Identifiers
public String getStarPrefix(){
  return starObjPreFix;
}
public void setStarObjectPrefix(String newObjectPrefix){
 this.starObjPreFix = newObjectPrefix; 
}
public void appendStarObjectPrefix(String appendString){
 this.starObjPreFix = this.starObjPreFix + appendString; 
}
public void prependStarObjectPrefix(String preFixString){
 this.starObjPreFix = preFixString + this.starObjPreFix; 
}

// Printing out methods
public void print(Simulation tmpSim){
  tmpSim.println("Aerodynamic Surface: " + cntrlName);
  for(PartSurface tmp : geomSurfs){
    tmpSim.println("  contains: " + tmp.getPresentationName());
  }
}
public String csvData(Simulation tmpSim){
  String tmpStr="";
  double meanVal;
  double stddevVal;

  String repType;

  //Name
  tmpStr=tmpStr+cntrlName;

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
      tmpStr=tmpStr + "," + meanVal;
  }

  //Forces and Moments data
  repType=this.getStarPrefix() + bodyStr;
  tmpSim.println("repType: "+repType);
  for(int i = 0; i <= 2; i++){
    // Force from pressure and shear.
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
  tmpSim.println("repType: "+repType);
  for(int i = 0; i <= 2; i++){
    // Force from pressure and shear.
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
  return tmpStr;
}
public String getName(){
  return cntrlName;
}
public void setName(String newName){
  cntrlName = newName;
}

// Meshing
public void setSurfMeshCustomControl(SurfaceCustomMeshControl newControl){
  surfCustomMeshSettings = newControl;
}
public void setSurfMeshCustomTEControl(SurfaceCustomMeshControl newControl){
  surfCustomMeshSettingsTE = newControl;
}
public void setVolMeshCustomControl(SurfaceCustomMeshControl newControl){
  volCustomMeshSettings = newControl;
}
public void setVolMeshCustomTEControl(SurfaceCustomMeshControl newControl){
  volCustomMeshSettingsTE = newControl;
}
public SurfaceCustomMeshControl getSurfMeshCustomControl(){
  return surfCustomMeshSettings;
}
public SurfaceCustomMeshControl getSurfMeshCustomTEControl(){
  return surfCustomMeshSettingsTE;
}
public SurfaceCustomMeshControl getVolMeshCustomControl(){
  return volCustomMeshSettings;
}
public SurfaceCustomMeshControl getVolMeshCustomTEControl(){
  return volCustomMeshSettingsTE;
}
public void getAnySurfaceMeshCustomControlSettings(){
  customTargetSurfaceSize = 
      MeshOpTool.getSurfCustTargetSize(surfCustomMeshSettings);
  customMinSurfaceSize = 
      MeshOpTool.getSurfCustMinSize(surfCustomMeshSettings);
  customNPtsOnCircle = 
      MeshOpTool.getSurfCustNPtsOnCircle(surfCustomMeshSettings);
}
public void getAnyVolumeMeshCustomControlSettings(){
  customFirstCellThickness =
      MeshOpTool.getCustomPrismNearWall(volCustomMeshSettings);
  customPrismThickness =
      MeshOpTool.getCustomPrismThick(volCustomMeshSettings);
  customPrismLayers =
      MeshOpTool.getCustomNumPrismLayers(volCustomMeshSettings);
}

// Surface Mesh
// -gets
public double getCustomTargetSurfaceSize(){
  return customTargetSurfaceSize;
}
public double getCustomMinSurfaceSize(){
  return customMinSurfaceSize;
}
public double getNPtsOnCircle(){
  return customNPtsOnCircle;
}
// -sets
public void setCustomTargetSurfaceSize(double newVal){
  customTargetSurfaceSize = newVal;
  MeshOpTool.surfCustTargSize(surfCustomMeshSettings  , "Relative", newVal);
  MeshOpTool.surfCustTargSize(surfCustomMeshSettingsTE, "Relative", newVal);
}
public void setCustomMinSurfaceSize(double newVal){
  customMinSurfaceSize = newVal;
  MeshOpTool.surfCustMinSize(surfCustomMeshSettings  , "Relative", newVal);
  MeshOpTool.surfCustMinSize(surfCustomMeshSettingsTE, "Relative", newVal);
}
public void setCustomNPtsOnCircle(double newVal){
  customNPtsOnCircle = newVal;
  MeshOpTool.surfNCircle(surfCustomMeshSettings  , newVal);
  MeshOpTool.surfNCircle(surfCustomMeshSettingsTE, newVal);
}

// Volume Mesh - Prism Layers
// -gets
public double getCustomFirstCellThickness(){
  return customFirstCellThickness;
}
public double getCustomPrismAbsThickness(){
  return customPrismThickness;
}
public int getCustomNumPrismLayers(){
  return customPrismLayers;
}
// -sets
public void useCustomPrismMeshSettings(boolean useCustom){
  MeshOpTool.surfPrismOverride(volCustomMeshSettings, true);
  //TE
  MeshOpTool.surfPrismOverride(volCustomMeshSettingsTE, true);
}
public void setCustomFirstCellThickness(double newVal){
  customFirstCellThickness = newVal;
  MeshOpTool.surfPrismNearWall(volCustomMeshSettings, newVal);
  //TE
  MeshOpTool.surfPrismNearWall(volCustomMeshSettingsTE, newVal * 4.0);
}
public void setCustomPrismAbsThickness(double newVal){
  customPrismThickness = newVal;
  MeshOpTool.surfPrismThick(volCustomMeshSettings, "Absolute", newVal);
  //TE
  MeshOpTool.surfPrismThick(volCustomMeshSettingsTE, "Absolute", newVal * 0.25);
}
public void setCustomNumPrismLayers(int newVal){
  customPrismLayers = newVal;
  MeshOpTool.surfPrismNumLayers(volCustomMeshSettings, newVal);
  //TE
  MeshOpTool.surfPrismNumLayers(volCustomMeshSettingsTE, newVal / 2);
}

//Regions
public void setRegion(Region tmpRegion){
  parentRegion = tmpRegion;
}
public Boundary setUpBoundary(String bndyName){
  Boundary tmpBndy;
  try{
    tmpBndy = parentRegion.getBoundaryManager().getBoundary(bndyName);
  }catch(NeoException e){
    tmpBndy = 
        parentRegion.getBoundaryManager().createEmptyBoundary(bndyName);
  }
  return tmpBndy;
}
public void setUniqueBoundary(String bndyID){
  Boundary tmpBndy = setUpBoundary(globalNames.getBndyName(bndyID));
  String bndyName = tmpBndy.getPresentationName();
  //add necessary surfaces to the boundary
  tmpBndy.getPartSurfaceGroup().setObjects(geomSurfs);
  //Set BC Type
  tmpBndy.setBoundaryType(WallBoundary.class);
}
public void addToGroupBoundary(String bndyID){
  Boundary tmpBndy = setUpBoundary(globalNames.getBndyName(bndyID));
  //add necessary surfaces to the boundary
  Collection<PartSurface> tmpColl = tmpBndy.getPartSurfaceGroup().getObjects();
  tmpColl.addAll(geomSurfs);
  tmpBndy.getPartSurfaceGroup().setObjects(tmpColl);
  //Set BC Type
  tmpBndy.setBoundaryType(WallBoundary.class);
}

// Coordinate Systems
// Geometry Info
public void setPrefix(String newPrefix){
  preFix=newPrefix;
}
public String getPrefix(){
  return preFix;
}
public int getPrefixSize(){
    return preFixSize;
}
public Collection<PartSurface> getPartSurfaces(){
    return geomSurfs;
}
public void setBodyCsys(CartesianCoordinateSystem newBodyCsys){
  this.bodyCsys = newBodyCsys;
}
public void setInletCsys(CoordinateSystem newInletCsys){
  this.inletCsys = newInletCsys;
}
public CartesianCoordinateSystem getBodyCsys(){
  return this.bodyCsys;
}
public CoordinateSystem getInletCsys(){
  return this.inletCsys;
}

//
// Reference Values
public void setReferenceValues(double refRho, double refVel, double refArea,
    double[] refRadius){
  // Reference Values
  setRefVel(refVel);
  setRefArea(refArea);
  setRefMomR(refRadius);
  setRefRho(refRho);
}
public double getRefVel(){
  return refVel;
}
public double getRefRho(){
  return refRho;
}
public double[] getRefMomR(){
  return refMomR;
}
public double getRefArea(){
  return refArea;
}
//
public void setRefVel(double newVal){
  refVel=newVal;
  ArrayList<Report> bothAeroBodyReports=new ArrayList();
  bothAeroBodyReports.addAll(aeroReports);
  bothAeroBodyReports.addAll(bodyReports);
  for(Report tmpReport:bothAeroBodyReports){
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
public void setRefRho(double newVal){
  refRho=newVal;
  ArrayList<Report> bothAeroBodyReports=new ArrayList();
  bothAeroBodyReports.addAll(aeroReports);
  bothAeroBodyReports.addAll(bodyReports);
  for(Report tmpReport:bothAeroBodyReports){
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
public void setRefMomR(double[] newVals){
  refMomR=newVals;
  ArrayList<Report> bothAeroBodyReports=new ArrayList();
  bothAeroBodyReports.addAll(aeroReports);
  bothAeroBodyReports.addAll(bodyReports);
  for(Report tmpReport:bothAeroBodyReports){
    if(tmpReport instanceof MomentCoefficientReport){
      double xAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(0);
      double yAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(1);
      double zAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(2);
      if(Math.abs(xAxisVal-1.0) > SMALL_EPS){
        ((MomentCoefficientReport) tmpReport)
            .getReferenceRadius().setValue(newVals[0]);
      }else if(Math.abs(yAxisVal-1.0) > SMALL_EPS){
        ((MomentCoefficientReport) tmpReport)
            .getReferenceRadius().setValue(newVals[1]);
      }else if(Math.abs(zAxisVal-1.0) > SMALL_EPS){
        ((MomentCoefficientReport) tmpReport)
            .getReferenceRadius().setValue(newVals[2]);
      }
    }
  }
}
public void setRefArea(double newVal){
  refArea=newVal;
  ArrayList<Report> bothAeroBodyReports=new ArrayList();
  bothAeroBodyReports.addAll(aeroReports);
  bothAeroBodyReports.addAll(bodyReports);

  for(Report tmpReport:bothAeroBodyReports){
    if(tmpReport instanceof ForceCoefficientReport){
        ((ForceCoefficientReport) tmpReport)
            .getReferenceArea().setValue(newVal);
    }
    if(tmpReport instanceof MomentCoefficientReport){
        ((MomentCoefficientReport) tmpReport)
            .getReferenceArea().setValue(newVal);
    }
  }
}
//
// Reports
//
public void TagReports(Simulation tmpSim, Collection<Tag> theseTags){
  aeroReports.forEach((tmpReport) -> {
    TagTool.addTags(tmpSim, tmpReport, theseTags);
  });
  bodyReports.forEach((tmpReport) -> {
    TagTool.addTags(tmpSim, tmpReport, theseTags);
  });
  wallYplusReports.forEach((tmpReport) -> {
    TagTool.addTags(tmpSim, tmpReport, theseTags);
  });  
}
public void makeReports(Simulation tmpSim){
  ArrayList<Report> tmpList = new ArrayList();
  //Aero Reports
  tmpList.addAll(
      generateForceAndMomentReports(tmpSim, cntrlName, 
          this.getStarPrefix() + "Aero", geomSurfs, refRho, refVel, refArea,
          refMomR, inletCsys,
          proxyRep)
      );
  for(Report tmpR : tmpList){
    if(!aeroReports.contains(tmpR)){
      aeroReports.add(tmpR);
    }
  }
  tmpList.clear();
  tmpList.addAll(
      generateForceAndMomentReports(tmpSim, cntrlName,
          this.getStarPrefix() + "Body", geomSurfs, refRho, refVel, refArea,
          refMomR, bodyCsys,
          proxyRep)
      );
  for(Report tmpR : tmpList){
    if(!bodyReports.contains(tmpR)){
      bodyReports.add(tmpR);
    }
  }
  //wall y plus
  tmpList.clear();
  tmpList.addAll(generateYPlusReports(tmpSim, cntrlName,
      this.getStarPrefix(), geomSurfs, proxyRep));
  for(Report tmpR : tmpList){
    if(!wallYplusReports.contains(tmpR)){
      wallYplusReports.add(tmpR);
    }
  }

  //Groupings
  ArrayList<Report> toGroup = new ArrayList();
  toGroup.addAll(aeroReports);
  toGroup.addAll(bodyReports);
  toGroup.addAll(wallYplusReports);
}
    
public void makeIterationMonitors(Simulation tmpSim,
    int plotSamplesLimit, int freq, int startIT){
  ArrayList<Monitor> tmpList = new ArrayList();
  //Make monitors
  //Aero Monitors


  tmpList.addAll(generateIterationMonitors(aeroReports,
          plotSamplesLimit, freq, startIT));
  for(Monitor tmp : tmpList){
    if(!aeroIterMonitors.contains(tmp)){
      aeroIterMonitors.add(tmp);
    }
  }
  tmpList.clear();
  tmpList.addAll(generateIterationMonitors(bodyReports,
          plotSamplesLimit, freq, startIT));
  for(Monitor tmp : tmpList){
    if(!bodyIterMonitors.contains(tmp)){
      bodyIterMonitors.add(tmp);
    }
  }
  tmpList.clear();
  tmpList.addAll(generateIterationMonitors(wallYplusReports,
          plotSamplesLimit, freq, startIT));
  for(Monitor tmp : tmpList){
    if(!wallYplusIterMonitors.contains(tmp)){
      wallYplusIterMonitors.add(tmp);
    }
  }

  //group monitors
  ArrayList<Monitor> toGroup = new ArrayList();
  toGroup.addAll(aeroIterMonitors);
  toGroup.addAll(bodyIterMonitors);
  toGroup.addAll(wallYplusIterMonitors);

}
public void makeUnsteadyMonitors(Simulation tmpSim, 
    int plotSamplesLimit, int freq, int startTS){
  //Make monitors
  aeroIterMonitors.addAll(generateUnsteadyMonitors(aeroReports,
          plotSamplesLimit, freq, startTS));
  bodyIterMonitors.addAll(generateUnsteadyMonitors(bodyReports,
          plotSamplesLimit, freq, startTS));
  wallYplusIterMonitors.addAll(generateUnsteadyMonitors(wallYplusReports,
          plotSamplesLimit, freq, startTS));

  //group monitors
  ArrayList<Monitor> toGroup = new ArrayList();
  toGroup.addAll(aeroUnsMonitors);
  toGroup.addAll(bodyUnsMonitors);
}
public ArrayList<Monitor> getAllMonitors(){
  ArrayList<Monitor> allMonitors = new ArrayList();
  allMonitors.addAll(aeroIterMonitors);
  allMonitors.addAll(bodyIterMonitors);
  allMonitors.addAll(wallYplusIterMonitors);
  allMonitors.addAll(aeroUnsMonitors);
  allMonitors.addAll(bodyUnsMonitors);

  return allMonitors;
}
//
public double getBodyCX(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim, 
          this.getStarPrefix() + "Body " + cntrlName + " - CX"))
      .getReportMonitorValue();
}
public double getBodyCY(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim,this.getStarPrefix() + "Body " + cntrlName + " - CY"))
      .getReportMonitorValue();
}
public double getBodyCZ(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim,this.getStarPrefix() + "Body " + cntrlName + " - CZ"))
      .getReportMonitorValue();
}
public double getAeroCD(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim,this.getStarPrefix() + "Aero " + cntrlName + " - CD"))
      .getReportMonitorValue();
}
public double getAeroCY(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim,this.getStarPrefix() + "Aero " + cntrlName + " - CY"))
      .getReportMonitorValue();
}
public double getAeroCL(Simulation tmpSim){
  return ((ForceCoefficientReport) ReportTool
      .getReport(tmpSim,this.getStarPrefix() + "Aero " + cntrlName + " - CL"))
      .getReportMonitorValue();
}
public ArrayList<Report> getAeroReports(){
  return aeroReports;
}
public ArrayList<Report> getBodyReports(){
  return bodyReports;
}
//
// Statistics
public static double getMeanReportItVal(Report tmpRep, String itPostFix,
    int nSamples){
  String repName = tmpRep.getPresentationName();
  String monName = repName+itPostFix;
  double meanVal = MonitorTool.getLastMonitorSamplesMean(
      tmpRep.getSimulation(), monName, nSamples);
  return meanVal;
}
public static double getMeanReportStddevItVal(Report tmpRep, String itPostFix,
    int nSamples,double meanVal){
  String repName = tmpRep.getPresentationName();
  String monName = repName+itPostFix;
  double stdDev = MonitorTool.getLastMonitorSamplesStDev(
      tmpRep.getSimulation(), monName, nSamples, meanVal);
  return stdDev;
}
public double getMeanForceCoef(Simulation tmpSim, 
    String preFix, int coefDir, int numSamp){

  String postFix = "";
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir == 0){
      postFix = " - CD";
    }else if(coefDir == 1){
      postFix = " - CY";
    }else if(coefDir == 2){
      postFix = " - CL";
    }else{
      postFix = " - " + coefDir;
    }

  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix = " - CX";
    }else if(coefDir==1){
      postFix = " - CY";
    }else if(coefDir==2){
      postFix = " - CZ";
    }else{
      postFix = " - " + coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + appEnd;
  return MonitorTool.getLastMonitorSamplesMean(tmpSim, monName, numSamp);
}
public double getMeanPressureForceCoef(Simulation tmpSim, 
    String preFix, int coefDir, int numSamp){

  String postFix = "";
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir == 0){
      postFix = " - CD";
    }else if(coefDir == 1){
      postFix = " - CY";
    }else if(coefDir == 2){
      postFix = " - CL";
    }else{
      postFix = " - " + coefDir;
    }

  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix = " - CX";
    }else if(coefDir==1){
      postFix = " - CY";
    }else if(coefDir==2){
      postFix = " - CZ";
    }else{
      postFix = " - " + coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + "_pressure" + appEnd;
  return MonitorTool.getLastMonitorSamplesMean(tmpSim, monName, numSamp);
}
public double getMeanShearForceCoef(Simulation tmpSim, 
    String preFix, int coefDir, int numSamp){

  String postFix = "";
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir == 0){
      postFix=" - CD";
    }else if(coefDir == 1){
      postFix=" - CY";
    }else if(coefDir == 2){
      postFix=" - CL";
    }else{
      postFix=" - " + coefDir;
    }

  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix=" - CX";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CZ";
    }else{
      postFix=" - " + coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + "_shear" + appEnd;
  return MonitorTool.getLastMonitorSamplesMean(tmpSim, monName, numSamp);
}
public double getStdDevForceCoef(Simulation tmpSim,
    String preFix,int coefDir, int numSamples, double meanVal){
  String postFix = globalNames.getItPostFix();
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir==0){
      postFix=" - CD";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CL";
    }else{
      postFix=" - "+coefDir;
    }
  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix=" - CX";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CZ";
    }else{
      postFix=" - "+coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + appEnd;
  return MonitorTool
      .getLastMonitorSamplesStDev(tmpSim, monName, numSamples, meanVal);
}
public double getStdDevPressureForceCoef(Simulation tmpSim,
    String preFix,int coefDir, int numSamples, double meanVal){
  String postFix = "";
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir==0){
      postFix=" - CD";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CL";
    }else{
      postFix=" - "+coefDir;
    }
  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix=" - CX";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CZ";
    }else{
      postFix=" - "+coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + "_pressure" + appEnd;
  return MonitorTool
      .getLastMonitorSamplesStDev(tmpSim, monName, numSamples, meanVal);
}
public double getStdDevShearForceCoef(Simulation tmpSim,
    String preFix,int coefDir, int numSamples, double meanVal){
  String postFix = "";
  String appEnd = globalNames.getItPostFix();
  if(preFix.equals(this.getStarPrefix() + "Aero")){
    if(coefDir==0){
      postFix=" - CD";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CL";
    }else{
      postFix=" - "+coefDir;
    }
  }else if(preFix.equals(this.getStarPrefix() + "Body")){
    if(coefDir==0){
      postFix=" - CX";
    }else if(coefDir==1){
      postFix=" - CY";
    }else if(coefDir==2){
      postFix=" - CZ";
    }else{
      postFix=" - "+coefDir;
    }
  }
  String monName = preFix + " " + cntrlName + postFix + "_shear" + appEnd;
  return MonitorTool
      .getLastMonitorSamplesStDev(tmpSim, monName, numSamples, meanVal);
}
//
//
// PLOTS
public void initPlots(boolean isUnsteady){
  // Figure out what kind of plot it needs to have
  String appEnd ="";
  if(isUnsteady){
    appEnd = unsPostFix;
  }else{
    appEnd = itPostFix;
  }
}
public ArrayList<StarPlot> getAllPlots(){
  return allPlots;
}
public Collection<StarPlot> getAllUserPlots(){
  return allPlots;
}
    
// SCENES
public void setViewFocalPt(double[] newPt){
  surfFocalPt[0]=newPt[0];
  surfFocalPt[1]=newPt[1];
  surfFocalPt[2]=newPt[2];
}
public void setObjectSize(double newSize){
  objectSize=newSize;
}

public void initViewList(Simulation tmpSim, double lengthScale){
  //Views are per Body Csys convention
  VisView tmpView;

  //Assumption for Aerodynamic surfaces is that the ObjectSize is its
  // longitudinal length and that the thickness of the surface is approximately
  // 30% of that length.
  double sceneAngle = 15; //deg
  double[] camCoord={0.0, 0.0, 0.0};
  double[] negY = {yVector[0], -yVector[1], yVector[2]};
  double[] negZ = {zVector[0], zVector[1], -zVector[2]};

  // negative x
  camCoord[0] = surfFocalPt[0] - objectSize * 2.0 * 0.3 / Math.atan(sceneAngle);
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "XN_" + cntrlName, labCsys, surfFocalPt, camCoord, negZ, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // positive x
  camCoord[0] = surfFocalPt[0] + objectSize * 2.0 * 0.3 / Math.atan(sceneAngle);
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "XP_" + cntrlName, labCsys, surfFocalPt, camCoord, negZ, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // negative y
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1] - objectSize * 2.0 * 0.3 / Math.atan(sceneAngle);
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "YN_" + cntrlName, labCsys, surfFocalPt, camCoord, negZ, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // positive y
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1] + objectSize * 2.0 * 0.3 / Math.atan(sceneAngle);
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "YP_" + cntrlName, labCsys, surfFocalPt, camCoord, negZ, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // negative z
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2] + objectSize * 2.0 / Math.atan(sceneAngle);
  tmpView=SceneTool.getView(tmpSim,
      "ZN_" + cntrlName, labCsys, surfFocalPt, camCoord, xVector, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // positive z
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2] - objectSize * 2.0 / Math.atan(sceneAngle);
  tmpView=SceneTool.getView(tmpSim,
      "ZP_" + cntrlName, labCsys, surfFocalPt, camCoord, xVector, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // high trail
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "HT_" + cntrlName, bodyCsys, surfFocalPt, camCoord, negY, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // low trail
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "LT_" + cntrlName, bodyCsys, surfFocalPt, camCoord, negY, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // low lead
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "LL_" + cntrlName, bodyCsys, surfFocalPt, camCoord, negY, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

  // high lead
  camCoord[0] = surfFocalPt[0];
  camCoord[1] = surfFocalPt[1];
  camCoord[2] = surfFocalPt[2];
  tmpView=SceneTool.getView(tmpSim,
      "HL_" + cntrlName, bodyCsys, surfFocalPt, camCoord, negY, true);
  if(!objectViews.contains(tmpView)) objectViews.add(tmpView);

}
private void initGenericScene(Simulation tmpSim,CoordinateSystem viewCsys,
                                     double[] axisCameraPos){
  // General
  Scene tmpScene;
  String tmpSceneName;
  String sceneTitle;
  Annotation sceneTitleAnn;
  //
  // Displayers
  PartDisplayer tmpPD; //part displayer
  ScalarDisplayer tmpSD; // scalar displayer
  //
  // Scene Setup
  tmpSceneName = cntrlName + " Generic Scene";
  genericScene = SceneTool.getScene(tmpSim, tmpSceneName);
  sceneTitle = tmpSceneName.substring(4);
  genericScene.setAxesVisible(false);
  //
  // Geometry Displayer
  tmpPD=SceneTool.getPartDisplayer(genericScene, "Generic Part", null, proxyRep);

  //  Scalar Displayer
  tmpSD=SceneTool.getScalarDisplayer(genericScene, "Generic Scalar", null);
  tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
  //
  // Camera & Scene Properties
  sceneTitleAnn=SceneTool
          .getTextAnnotation(tmpSim, sceneTitle, sceneTitle, 0.035,
                             new double[]{0.025, 0.94,0.}, true);
  SceneTool.addAnnotation(genericScene, sceneTitleAnn);
  SceneTool.setSceneView(genericScene, surfFocalPt, axisCameraPos,
                         xVector, viewCsys, 1.0, 1);
  SceneTool.removeDefaultLogo(genericScene);
  genericScene.close(true);
}
private void printObjectScenes(Simulation tmpSim, String savePath){
  /* printGenericScalarScenes outputs Object Scalar images to the
     specified toDirectory path
  */
  //
  // Generic Displayer Names
  String scalarName = "Generic Scalar";
  String partName   = "Generic Part";
  //
  // Scene Displayers
  PartDisplayer tmpPD;
  ScalarDisplayer tmpSD;
  tmpPD=SceneTool.getPartDisplayerFromScene(genericScene, partName);
  tmpSD=SceneTool.getScalarDisplayerFromScene(genericScene, scalarName);
  //
  // Reset all displayers
  tmpPD.getInputParts().setQuery(null);
  tmpSD.getInputParts().setQuery(null);
  //
  //
  String whichDir;

  //=======================
  // USER SURFACE IMAGES
  //=======================
  whichDir = savePath + File.separator + "USER";
  //Skin Friction Coefficient
  tmpSD.getInputParts().setObjects(geomSurfs);
  SceneTool.setScalarDisplayerField(tmpSD, "SkinFrictionCoefficient",
      "Off", "Off", 0.0, 0.005, proxyRep);
  SceneTool.setScalarColormap(tmpSD,"high-contrast", 64, 6, true, "Left Side")
      .getLegend().setLabelFormat("%-#5.3f");
  SceneTool.outputSceneFromViews(
      genericScene, whichDir, "Cf", xyRes, objectViews);

  //Pressure Coefficient
  tmpSD.getInputParts().setObjects(geomSurfs);
  SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient",
      "Off", "Off", -8.0, 2.0, proxyRep);
  SceneTool.setScalarColormap(tmpSD, ColorMapTool.getBlueDkGnRedString(tmpSim),
      64, 6, true, "Left Side")
      .getLegend().setLabelFormat("%-#5.3f");
  SceneTool.outputSceneFromViews(genericScene,whichDir,"Cp",xyRes,objectViews);

  //=======================
  // CFD SURFACE IMAGES
  //=======================
  whichDir = savePath + File.separator + "CFD";
  //Pressure Coefficient
  tmpSD.getInputParts().setObjects(geomSurfs);
  SceneTool.setScalarDisplayerField(tmpSD,"WallYplus",
          "Off","Off",0.0,300.0,proxyRep);
  SceneTool.setScalarColormap(tmpSD,
      ColorMapTool.getTotalRangeWallYPlusString(tmpSim),
      64, 6, true,"Left Side")
      .getLegend().setLabelFormat("%-#5.3f");
  SceneTool.outputSceneFromViews(genericScene, whichDir, "all_y_plus",
                                 xyRes, objectViews);
}

// Tags
public void addTags(Simulation tmpSim, Collection<Tag> allTags){
  TagTool.addTags(tmpSim, geomPart, allTags);
}


//
//====================
// PRIVATE METHODS
//====================
// Reports
private ArrayList<Report> generateForceAndMomentReports(
    Simulation tmpSim, String myName, String preFix, 
    Collection <PartSurface> mySurfs, double myRho,double myVel, double myArea, 
    double[] myMomR,CoordinateSystem relCsys, Representation thisRep){
  ArrayList<Report> tmpRepList = new ArrayList();  
  String rootName = preFix + " " + myName;

  //Instantiate
  ForceReport  fX; ForceReport  fY; ForceReport  fZ;
  MomentReport mX; MomentReport mY; MomentReport mZ;
  ForceReport  fX_pressure; ForceReport  fY_pressure; ForceReport  fZ_pressure;
  MomentReport mX_pressure; MomentReport mY_pressure; MomentReport mZ_pressure;
  ForceReport  fX_shear; ForceReport  fY_shear; ForceReport  fZ_shear;
  MomentReport mX_shear; MomentReport mY_shear; MomentReport shear;
  String xName = "";
  String yName = "";
  String zName = "";
  ForceCoefficientReport cX; MomentCoefficientReport cMX;
  ForceCoefficientReport cX_pressure; MomentCoefficientReport cMX_pressure;
  ForceCoefficientReport cX_shear; MomentCoefficientReport cMX_shear;
  ForceCoefficientReport cY; MomentCoefficientReport cMY;
  ForceCoefficientReport cY_pressure; MomentCoefficientReport cMY_pressure;
  ForceCoefficientReport cY_shear; MomentCoefficientReport cMY_shear;  
  ForceCoefficientReport cZ; MomentCoefficientReport cMZ;
  ForceCoefficientReport cZ_pressure; MomentCoefficientReport cMZ_pressure;
  ForceCoefficientReport cZ_shear; MomentCoefficientReport cMZ_shear;
  //Forces
  fX=ReportTool.forceReport(
      tmpSim,rootName + " - FX", relCsys, xVector, mySurfs, thisRep);
  fX_pressure = ReportTool.forceReport(
      tmpSim,rootName + " - FX_pressure", relCsys, zVector, mySurfs,
          thisRep, "pressure");
  fX_shear = ReportTool.forceReport(
      tmpSim,rootName + " - FX_shear",relCsys,zVector,mySurfs,
          thisRep,"shear");
  try{
    Thread.sleep(500);
  }catch(InterruptedException e){

  }        
  fY=ReportTool.forceReport(
      tmpSim,rootName + " - FY",relCsys,yVector,mySurfs,thisRep);
  fY_pressure = ReportTool.forceReport(
      tmpSim,rootName + " - FY_pressure", relCsys, zVector, mySurfs,
          thisRep, "pressure");
  fY_shear = ReportTool.forceReport(
      tmpSim,rootName + " - FY_shear",relCsys,zVector,mySurfs,
          thisRep,"shear");
  try{
    Thread.sleep(500);
  }catch(InterruptedException e){

  }        
  fZ = ReportTool.forceReport(
      tmpSim,rootName + " - FZ",relCsys,zVector,mySurfs,thisRep);
  fZ_pressure = ReportTool.forceReport(
      tmpSim,rootName + " - FZ_pressure", relCsys, zVector, mySurfs,
          thisRep, "pressure");
  fZ_shear = ReportTool.forceReport(
      tmpSim,rootName + " - FZ_shear",relCsys,zVector,mySurfs,
          thisRep,"shear");
  try{
    Thread.sleep(500);
  }catch(InterruptedException e){

  }        
  tmpRepList.add(fX); tmpRepList.add(fX_pressure); tmpRepList.add(fX_shear);
  tmpRepList.add(fY); tmpRepList.add(fY_pressure); tmpRepList.add(fY_shear);
  tmpRepList.add(fZ); tmpRepList.add(fZ_pressure); tmpRepList.add(fZ_shear);
  //
  //Moments
  mX=ReportTool.momentReport(
      tmpSim, rootName + " - MX", relCsys, zeroOrigin, xVector,
      mySurfs, thisRep);
  mY=ReportTool.momentReport(
      tmpSim,rootName + " - MY", relCsys, zeroOrigin, yVector,
      mySurfs, thisRep);
  mZ=ReportTool.momentReport(
      tmpSim, rootName + " - MZ", relCsys, zeroOrigin, zVector,
      mySurfs, thisRep);
  tmpRepList.add(mX);
  tmpRepList.add(mY);
  tmpRepList.add(mZ);
  //
  //Force Coefficients
  if(preFix.equals(this.getStarPrefix() + "Aero")){
      xName = " - CD"; yName = " - CY"; zName = " - CL";
  }else if(preFix.equals(this.getStarPrefix() + "Body")){
      xName = " - CX"; yName = " - CY"; zName = " - CZ";
  }
  cX = ReportTool.forceCoefReport(tmpSim,
      rootName + xName, relCsys, xVector, myRho, myVel, myArea,
      mySurfs, thisRep);
  cX_pressure = ReportTool.forceCoefReport(tmpSim,
      rootName + xName + "_pressure", relCsys, xVector, myRho, myVel, myArea,
      mySurfs, thisRep,"pressure");
  cX_shear = ReportTool.forceCoefReport(tmpSim,
      rootName + xName + "_shear", relCsys, xVector, myRho, myVel, myArea,
      mySurfs, thisRep,"shear");

  cY = ReportTool.forceCoefReport(tmpSim, rootName + yName, relCsys, yVector, 
      myRho, myVel, myArea, mySurfs, thisRep);
  cY_pressure = ReportTool.forceCoefReport(tmpSim, 
      rootName + yName + "_pressure", relCsys, yVector, 
      myRho, myVel, myArea, mySurfs, thisRep, "pressure");
  cY_shear = ReportTool.forceCoefReport(tmpSim, 
      rootName + yName + "_shear", relCsys, yVector, 
      myRho, myVel, myArea, mySurfs, thisRep, "shear");

  cZ = ReportTool.forceCoefReport(tmpSim, rootName + zName, relCsys, zVector, 
      myRho, myVel, myArea, mySurfs, thisRep);
  cZ_pressure = ReportTool.forceCoefReport(tmpSim,
      rootName + zName + "_pressure", relCsys, zVector, 
      myRho, myVel, myArea, mySurfs, thisRep, "pressure");
  cZ_shear = ReportTool.forceCoefReport(tmpSim,
      rootName + zName + "_shear", relCsys, zVector, 
      myRho, myVel, myArea, mySurfs, thisRep, "shear");

  tmpRepList.add(cX); tmpRepList.add(cX_pressure); tmpRepList.add(cX_shear);
  tmpRepList.add(cY); tmpRepList.add(cY_pressure); tmpRepList.add(cY_shear);
  tmpRepList.add(cZ); tmpRepList.add(cZ_pressure); tmpRepList.add(cZ_shear);

  //Moment Coefficients
  cMX=ReportTool.momentCoefReport(tmpSim, rootName + " - CmX", relCsys,
      zeroOrigin, xVector, myRho, myVel, myArea, myMomR[0], mySurfs, thisRep);
  cMY=ReportTool.momentCoefReport(tmpSim, rootName + " - CmY", relCsys,
      zeroOrigin, yVector, myRho, myVel, myArea, myMomR[1], mySurfs, thisRep);
  cMZ=ReportTool.momentCoefReport(tmpSim, rootName + " - CmZ", relCsys,
      zeroOrigin, zVector, myRho, myVel, myArea, myMomR[2], mySurfs, thisRep);
  tmpRepList.add(cMX);
  tmpRepList.add(cMY);
  tmpRepList.add(cMZ);
  return tmpRepList;
}
private static ArrayList<Report> generateYPlusReports(Simulation tmpSim,
    String cntrlName, String preFix, Collection<PartSurface> theseSurfs,
    Representation thisRep){
  ArrayList<Report> retList = new ArrayList();
  String rootName = preFix + "Wall Y+ " + cntrlName;
  PrimitiveFieldFunction pFF =
      ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager()
          .getFunction("WallYplus")
      );
  MaxReport maxRep = ReportTool.maxReport(tmpSim,
      rootName + " Max", pFF, theseSurfs, thisRep);
  MinReport minRep = ReportTool.minReport(tmpSim,
      rootName + " Min", pFF, theseSurfs, thisRep);
  AreaAverageReport aveRep = ReportTool.surfAveReport(tmpSim,
      rootName + " Surf Ave", pFF, theseSurfs, thisRep);
  retList.add(aveRep); // probably most useful
  retList.add(maxRep); // watch for high values
  retList.add(minRep); // Min: least useful since stagnation points are low y+.
  return retList;
}

// CLASS METHODS
// Monitors
public static ArrayList<Monitor> generateIterationMonitors(
    ArrayList<Report> reportList, int plotLimit, int itFreq, int startIt){
  ArrayList<Monitor> retList=new ArrayList();
  for(Report tmp:reportList){
    Monitor tmpMon = MonitorTool.reportIterationMonitor(
        tmp.getSimulation(), tmp, plotLimit, itFreq, startIt);
    if(!retList.contains(tmpMon)) retList.add(tmpMon);
  }
  return retList;
}
public static ArrayList<Monitor> generateUnsteadyMonitors(
    ArrayList<Report> reportList, int plotLimit, int tsFreq, int startTS){
    ArrayList<Monitor> retList = new ArrayList();
    for(Report tmp:reportList){
        Monitor tmpMon = MonitorTool.reportUnsteadyMonitor(
            tmp.getSimulation(), tmp, plotLimit, tsFreq, startTS);
        if(!retList.contains(tmpMon)) retList.add(tmpMon);
    }
    return retList;
}

///
}// END AerodynamicSurface