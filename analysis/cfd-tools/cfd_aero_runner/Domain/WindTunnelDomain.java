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
package Domain;

import AeroToolKit.*;
import GeometricObject.*;
import MeshTools.*;
import PhysicsTools.*;
import Domain.*;
import Naming.*;
import Tools.*;
import Solvers.*;

import java.io.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.*;

import star.base.neo.*;
import star.base.report.*;
import star.common.*;
import star.energy.*;
import star.flow.*;
import star.meshing.*;
import star.prismmesher.*;
import star.turbulence.*;
import star.vis.*;

public class WindTunnelDomain extends Domain{

// Utilitiy variables
final double SMALL_EPS = 1.0E-6;

// Useful Vectors
double[] xVector    = { 1.,  0.,  0.};
double[] negativeX  = {-1.,  0.,  0.};
double[] yVector    = { 0.,  1.,  0.};
double[] negativeY  = { 0., -1.,  0.};
double[] zVector    = { 0.,  0.,  1.};
double[] negativeZ  = { 0.,  0., -1.};
double[] zeroVector = { 0.,  0.,  0.};

double[] wtAxisRot = {0., 0., 1.}; //windtunnel geometries always about Z axis

// Objects inside the wind tunnel
ArrayList<OversetDomain>      allOversetDomains = new ArrayList();
ArrayList<AerodynamicSurface> allAirfoilObjects = new ArrayList();

// Physics
double alphaAngle = 0.0;
boolean isFS;
boolean is2D;
CFD_Physics wtPhysics;

// Reference Values
double chordLength;
double refRe;
double refArea;
double[] refMomR;
double highMachNumber = 0.4; // Signifies high Mach number settings

// Regions
//Default Freestream values
double fsTVR = 10.0;
double fsTi  = 0.01;
Boundary inletBndy;
Boundary outletBndy;

// Mesh Configuration
double oldTurnTableAngleInDeg = 0.0;

// Coordinate Systems
CartesianCoordinateSystem wtCsys; // This is the Lab coordinate system for the WindTunnel Domain.
CartesianCoordinateSystem bodyCsys;
CartesianCoordinateSystem inletCsys;

// Representations
Representation proxyRep;

// Tags
Tag primaryWTDomainTag;

// Airfoils
ArrayList<String> airfoilStrList;

// Boundary Layer Probes
ArrayList<BoundaryLayerProbe> allBoundaryLayerProbes;

// Data statistics
int statSamples = 500;

// Scenes
ArrayList<Scene> allWTScenes = new ArrayList();

// Annotations
Collection<Annotation> standardAnnotations;
Collection<Annotation> wtAnnotations = new ArrayList();

// Visualization
int[]    xyRes = {2000, 1000};
double   viewAngleDistanceOffset = Math.atan(15.0 * Math.PI / 180.0);
double[] wt2DCameraPos = {0., 0., 0.};

public WindTunnelDomain(Simulation tmpSim, GeometryPart geomPartIN){
  super(tmpSim, geomPartIN);
  this.airfoilStrList = globalNames.getAirfoilStrList();
}

public void setIs2D(boolean isWT2D){
  this.is2D = isWT2D;
}
public boolean is2D(){
  return this.is2D;
}
public void set2DWTViewPositions(Simulation tmpSim){
  wt2DCameraPos[0] =  -0.25 * this.getChordLengthFromParameters(tmpSim);
  wt2DCameraPos[2] =   1.0  * this.getChordLengthFromParameters(tmpSim) * viewAngleDistanceOffset;
}
// REPRESENTATIONS
public void setProxyRepresentations(Representation newProxyRep){
  this.proxyRep = newProxyRep;
}

// TAGS
public void setPrimaryDomainTag(Tag newTag){
  primaryWTDomainTag = newTag;
}
public Tag getPrimaryDomainTag(){
  return primaryWTDomainTag;
}

// GEOMETRY PARTS
public ArrayList<GeometryPart> getGeometryPartsOfControlledObjects(){
  ArrayList<GeometryPart> relevantParts = new ArrayList();
  relevantParts.add(domainPart);
  for(OversetDomain tmpOSDomain : allOversetDomains){
    relevantParts.add(tmpOSDomain.getDomainPart());
  }
  return relevantParts;
}
public ArrayList<Region> getRegionsOfControlledObjects(){
  ArrayList<Region> relevantRegions = new ArrayList();
  relevantRegions.add(domainRegion);
  for(OversetDomain tmpOSDomain : allOversetDomains){
    relevantRegions.add(tmpOSDomain.getRegion());
  }
  return relevantRegions;
}

// COORDINATE SYSTEMS
public void setWTCsys(CartesianCoordinateSystem newWTCsys){
  this.wtCsys = newWTCsys;
}
public void setBodyCsys(CartesianCoordinateSystem newCsys){
  bodyCsys = newCsys;
}
public void setInletCsys(CartesianCoordinateSystem newCsys){
  inletCsys = newCsys;
}
public CartesianCoordinateSystem getBodyCsys(){
  return bodyCsys;
}
@Override
public CartesianCoordinateSystem getInletCsys(){
  return inletCsys;
}

// PHYSICS
public CFD_Physics getPhysics(){
  return wtPhysics;
}
public void setPhysics(CFD_Physics newPhysics){
  wtPhysics = newPhysics;
}
public void setIsFS(boolean containsFSBoundary){
  isFS = false;
}
public PhysicsContinuum getPhysicsContinuum(){
  return wtPhysics.getContinuum();
}
public boolean isPhysicsSet(){
  if(wtPhysics == null){
    return false;
  }
  return true;
}

// REFERENCE VALUES
public void setRefRe(double newVal){
  refRe = newVal;
}
public double getRefRe(){
  return refRe;
}

private String getChordParamName(){
  return
      this.getPrimaryDomainTag().getPresentationName() + "." + "ChordLength";
}
private String getReferenceAreaParamName(){
  return
      this.getPrimaryDomainTag().getPresentationName() + "." + "ReferenceArea";
}
private String getMomentReferenceVectorParamName(){
  return
      this.getPrimaryDomainTag().getPresentationName() + "." + "ReferenceMomentR";
}
  
public boolean checkIfWTChordLengthExists(Simulation tmpSim){
  String chordLengthParamName = getChordParamName();
  ScalarGlobalParameter referenceChordLengthParam = 
      SimTool.getScalarSimulationParameter(tmpSim, chordLengthParamName);
  if(referenceChordLengthParam.getQuantity().getSIValue() < SMALL_EPS){
    return false;
  }
  return true;
}
public void setChordLength(Simulation tmpSim, double newChordLength){
  String chordLengthParamName = getChordParamName();
  ScalarGlobalParameter wtChord = 
      SimTool.getScalarSimulationParameter(tmpSim, chordLengthParamName);
  wtChord.getQuantity().setValue(newChordLength);
  chordLength = newChordLength;
}
public double getChordLengthFromParameters(Simulation tmpSim){
  String chordLengthParamName = getChordParamName();
  ScalarGlobalParameter wtChord = 
      SimTool.getScalarSimulationParameter(tmpSim, chordLengthParamName);
  this.chordLength = wtChord.getQuantity().getSIValue();
  return chordLength;
}
public double getChordLength(){
  return chordLength;
}
public boolean checkIfWTReferenceAreaExists(Simulation tmpSim){
  String chordLengthParamName = getReferenceAreaParamName();
  ScalarGlobalParameter referenceAreaParam = 
      SimTool.getScalarSimulationParameter(tmpSim, chordLengthParamName);
  if(referenceAreaParam.getQuantity().getSIValue() < SMALL_EPS){
    return false;
  }
  return true;
}
public void setReferenceArea(Simulation tmpSim, double newReferenceAreaValue){
  String refAreaParamName = getReferenceAreaParamName();
  ScalarGlobalParameter wtReferenceArea = 
      SimTool.getScalarSimulationParameter(tmpSim, refAreaParamName);
  wtReferenceArea.getQuantity().setValue(newReferenceAreaValue);
  refArea = newReferenceAreaValue;
}
public double getReferenceAreaFromParameters(Simulation tmpSim){
  String refAreaParamName = getReferenceAreaParamName();
  ScalarGlobalParameter wtChord = 
      SimTool.getScalarSimulationParameter(tmpSim, refAreaParamName);
  this.refArea = wtChord.getQuantity().getSIValue();
  return refArea;
}
public double getReferenceArea(){
  return refArea;
}
public boolean checkIfWTMomentReferenceVectorExists(Simulation tmpSim){
  String refMomentReferenceVectorParamName = getMomentReferenceVectorParamName();
  VectorGlobalParameter wtMomentReferenceVector = 
      SimTool.getVectorSimulationParameter(tmpSim, refMomentReferenceVectorParamName);
  double[] tempMomR = DoubleVector.valueOf(
      wtMomentReferenceVector.getQuantity().getDefinition()).toDoubleArray();
  if(tempMomR[0] < SMALL_EPS && tempMomR[1] < SMALL_EPS && tempMomR[2] < SMALL_EPS ){
    return false;
  }
  return true;
}
public void setMomentReferenceVector(Simulation tmpSim, double[] newReferenceMomentVector){
  String refMomentReferenceVectorParamName = getMomentReferenceVectorParamName();
  VectorGlobalParameter wtMomentReferenceVector = 
      SimTool.getVectorSimulationParameter(tmpSim, refMomentReferenceVectorParamName);
  wtMomentReferenceVector.getQuantity()
      .setVector(new DoubleVector(newReferenceMomentVector));
  this.refMomR = newReferenceMomentVector;
}
public double[] getMomentReferenceVectorFromParameters(Simulation tmpSim){
  String refMomentReferenceVectorParamName = getMomentReferenceVectorParamName();
  VectorGlobalParameter wtMomentReferenceVector = 
      SimTool.getVectorSimulationParameter(tmpSim, refMomentReferenceVectorParamName);
  this.refMomR = DoubleVector.valueOf(
      wtMomentReferenceVector.getQuantity().getDefinition()).toDoubleArray();
  return refMomR;
}
public double[] getRefMomR(){
  return refMomR;
}

// GEOMETRY VALUES
public void setWTAlphaAngle(Simulation tmpSim){
  if(!super.isFreeStreamDomain()){ //non-freestream cases
    String assemblyOpName = "Rotate " + super.getName() + "Assembly";
    alphaAngle = MeshOpTool.getRotationAngle(tmpSim, assemblyOpName);
    for(OversetDomain tmpDomain : getAllOversetDomains()){
      tmpSim.println("Looking for alpha in:" +tmpDomain.getName());
      if(tmpDomain.getName().startsWith("OS1")){
        alphaAngle = tmpDomain.getDomainAngle();
        tmpSim.println(" alpha angle: "+alphaAngle);
      }
    }
  }else{
    //use the Y-axis Lab x-component to get the alpha
    alphaAngle = Math.asin(getInletCsys().getBasis1().get(0)) * 180. / Math.PI;
  }
}
public double getAlphaAngle(){
  return alphaAngle;
}

// MESHING
@Override
public void setDomainSurfMeshOp(AutoMeshOperation newOp){
  // Wind Tunnel Domains can only belong to a single surface mesh operation.
  // Wind Tunnel Domains control their child overset domains' surface mesh
  // operations.
  super.setDomainSurfMeshOp(newOp);
  for(OversetDomain osDomain : allOversetDomains){
    osDomain.setDomainSurfMeshOp(newOp);
  }
}
@Override
public void setDomainVolumeMeshOp(AutoMeshOperation newOp){
  // Wind Tunnel Domains can only belong to a single volume mesh operation.
  // Wind Tunnel Domains control their child overset domains' volume mesh
  // operations.
  super.setDomainVolumeMeshOp(newOp);
  for(OversetDomain osDomain : allOversetDomains){
    osDomain.setDomainVolumeMeshOp(newOp);
  }
}
public ArrayList<AutoMeshOperation> getAssociatedAutoMeshOps(){
  ArrayList<AutoMeshOperation> retList = new ArrayList();
  retList.add(domainSurfMeshOp);
  retList.add(domainVolumeMeshOp);
  return retList;
}
private static void setupMeshMetrics(Simulation tmpSim, ArrayList<Region> myRegs){
  DerivedPartTool.highValueCells(
      tmpSim, myRegs, "Metric High Skewness Angle","SkewnessAngle",95.0
      );
  DerivedPartTool.lowValueCells(
      tmpSim, myRegs, "Metric Low Cell Quality","CellQuality",1e-4
      );
  DerivedPartTool.lowValueCells(
      tmpSim, myRegs, "Metric Low Cell Volume","Volume",1e-4
      );
  DerivedPartTool.lowValueCells(
      tmpSim, myRegs, "Metric Low Face Validity","FaceValidity",0.95
      );
  DerivedPartTool.highValueCells(
      tmpSim, myRegs, "Prism Layer Cells","PrismLayerCells",0.0
      );
}
  
// OVERSET DOMAIN CONTROLS
public void addOversetDomain(OversetDomain newOSDomain){
  /* Adds an OversetDomain to the list of OversetDomains controlled by this
     WindTunnelDomain.
  */
  if(!allOversetDomains.contains(newOSDomain)){
    allOversetDomains.add(newOSDomain);
  }
}
public ArrayList<OversetDomain> getAllOversetDomains(){
  return allOversetDomains;
}  
public void createOversetInterfaces(Simulation tmpSim){
  /* Creates all overset interfaces across this WindTunnelDomain.
     Assumes that a Physics continuua already exists in the WindTunnelDomain.
  */
  //Interface to all required domains
  for(Domain tmpDomain : allOversetDomains){
    // Primary interface from overset region to this wind tunnel domain.
    IndirectRegionInterface wtInterface = 
        ((OversetDomain) tmpDomain).interfaceToDomain(this);
    TagTool.addTags(tmpSim, wtInterface, 
        Collections.singleton(this.getPrimaryDomainTag()));
    // TODO: Extend for overlapping overset domains.
  }
  tmpSim.println("Finished interfaces");
}

// AIRFOIL METHODS
public void instantiateAirfoils(Simulation tmpSim){
  //===============================
  // Airfoils
  //===============================
  instantiateAirfoilObjects(tmpSim);

  // Airfoils should instantiate with any existing custom mesh settings.
  tmpSim.println("WT MSG: Airfoils checking custom mesh settings.");
  for(AerodynamicSurface tmpFoil : allAirfoilObjects){
    tmpSim.println("WT MSG: Airfoil " + tmpFoil.getName());
    tmpFoil.getAnySurfaceMeshCustomControlSettings();
    tmpFoil.getAnyVolumeMeshCustomControlSettings();
  }
}
public ArrayList<GeometryPart> getRawAirfoilParts(
    Simulation tmpSim, Collection<GeometryPart> rawPartList,
    GeometryPart avoidPart){
  /* Figures out what Objects are actually in the windtunnel */
  /* Method to determine which part contains the domain */
  Collection<GeometryPart> tmpPartList = new ArrayList();
  for(GeometryPart tmpPart : rawPartList){
      tmpPartList.add(tmpPart);
  }
  rawPartList.remove(avoidPart);
  return MeshOpTool.findGeometryParts(tmpSim, tmpPartList, airfoilStrList);
}
public void setAllAirfoilObjects(ArrayList<AerodynamicSurface> newAirfoils){
  allAirfoilObjects = newAirfoils;
}
public ArrayList<AerodynamicSurface> getAllAirfoilObjects(){
  return allAirfoilObjects;
}
public void updateAirfoilReferenceValues(Simulation tmpSim){
  if(allAirfoilObjects.size() > 0){
    String wtTagName = primaryWTDomainTag.getPresentationName();
    Tag wtTag = primaryWTDomainTag;
    setupTotalAirfoilReports(tmpSim, wtTagName + "." + "Aero", inletCsys)
        .forEach(tmpReport ->{
          TagTool.addTags(tmpSim, tmpReport, Collections.singleton(wtTag));
      });
    setupTotalAirfoilReports(tmpSim, wtTagName + "." + "Body", inletCsys)
        .forEach(tmpReport ->{
          TagTool.addTags(tmpSim, tmpReport, Collections.singleton(wtTag));
      });
  }
  //Set airfoil reference values
  for(AerodynamicSurface airFoil : allAirfoilObjects){
    airFoil.setReferenceValues(refRho, refVel, refArea, refMomR);
  }
}
public ArrayList<String> getAirfoilNames(Simulation tmpSim){
  instantiateAirfoilObjects(tmpSim);

  ArrayList<String> airFoilNames = new ArrayList();
  for(AerodynamicSurface tmpFoil:allAirfoilObjects){
    airFoilNames.add(tmpFoil.getName());
  }
  return airFoilNames;
}
public void instantiateAirfoilObjects(Simulation tmpSim){
  /* instantiateAirfoilObjects is used to instantiate airfoil objects.
     Airfoil objects an be extracted from the following domains:
      Windtunnel Domain
      Overset Domain
  */

  if(allAirfoilObjects.isEmpty()){
    tmpSim.println("WT MSG: Instantiating Airfoil Objects...");
    int airfoilPreFix = airfoilStrList.get(0).length();
    double[] momentAxis = {chordLength, chordLength, chordLength};

    // TODO: extend to the parent wind tunnel domain in case this case is
    // a freestream, single airfoil type of case.
    ArrayList<Domain> allDomainObjects = new ArrayList();
    allDomainObjects.addAll(allOversetDomains);
    
    // Set up airfoil boundaries (if they exist).
    for(Domain tmpDomain : allDomainObjects){
      GeometryPart tmpDomainPart = tmpDomain.getDomainPart();

      // Get associated automated surface and volume mesh operations that the
      // domain is associated with.
      // If domainPart is created from a boolean mesh operation
      // we can know the parent parts of the mesh operation which means we know:
      //   1) The name of the CAD airfoil 
      //   2) everything about the CAD geometry manipulations,
      //      i.e. rotation operation & the angle at
      //      which that rotation operation is set!
      //   3) The custom controls and settings
      // Look through the Part Surface list to see if there are any airfoil
      // prefixes in this domain's part list
      for(String tmpFoilStr : airfoilStrList){
        ArrayList<PartSurface> foilSurfaces = 
            GeometryTool.filterPartSurfacesByString(tmpDomainPart, tmpFoilStr);
        // If there are no surfaces that meet this description, there are no
        // airfoil objects that need to be made. Go on to the next airfoil name
        // in the list.
        if(foilSurfaces.isEmpty()){
          break;
        }
        GeometryPart cadParent = null;
        String cadCSysName = "";

        // domainParts are MeshOperationPart type in the following conditions:
        //  1) the domainPart was a wind tunnel with a CAD subtraction (airfoil)
        if(tmpDomainPart instanceof MeshOperationPart){
          // If domainPart was created from a boolean operation, it is not the
          // original CAD part. 
          MeshOperation domainOp =
              ((MeshOperationPart) tmpDomainPart).getOperation();

          // Also get the operation that created the
          // domainPart.
          // Find the specific parent CAD parts that created this
          // MeshOperationPart. Get the associated coordinate system of the
          // parent CAD part.
          //
          // Get the all possible GeometryParts that could have an Airfoil
          // object located inside of it. Filter the each of these candidates'
          // PartSurfaces to collect all the possible airfoil PartSurfaces.
          Collection<GeometryPart> domainParts =
              domainOp.getInputGeometryObjects().getLeafParts();
          Collection<GeometryPart> allRawAirfoilParts =
              getRawAirfoilParts(tmpSim,domainParts,tmpDomainPart);

          boolean didIFindAParent = false;
          for(GeometryPart tmpParent : allRawAirfoilParts){
            for(PartSurface tmpSurf : tmpParent.getPartSurfaces()){
              if(tmpSurf.getPresentationName().startsWith(tmpFoilStr) || 
                  tmpSurf.getPresentationName().contains("."+tmpFoilStr)){
                cadParent = tmpParent;
                cadCSysName = tmpParent.getPresentationName();
                didIFindAParent = true;
                break;
              }
            }
            if(didIFindAParent){
              break;
            }
          }
        // Overset domain types are required to be pre-cut CAD parts and should
        // not be the result of mesh operations.
        }else if(tmpDomain instanceof OversetDomain){
          cadParent = tmpDomainPart; // a dummy assignment

          // foilSurfaces are a list of PartSurface that all match the corret
          // airfoil element name. That's why we only need the first index one.
          // TODO: This assumes that the foilSurfaces are alphabetically
          // set up in the tree such that the first PartSurface named is the
          // correct single string name.
          cadCSysName = getOversetFoilName(foilSurfaces.get(0));
        }

        if(cadCSysName.equals("") || (cadParent == null)){
            tmpSim.println("WT MSG: cadCsysName or cadParent is empty!");
        }
        // Create DeflectableControlSurface Object
        // It is expected that the airfoil CAD Csys Name already exists as a
        // nested coordinate in the wind tunnel domain coordinate system.
        CartesianCoordinateSystem cadCSys =
            SimTool.getNestedCoordinate(wtCsys, cadCSysName);

        //Make airfoil object 
        //Instantiate the object!
        DeflectableControlSurface newFoil = new DeflectableControlSurface(
            tmpSim, foilSurfaces, airfoilPreFix, cadCSys
            );
        newFoil.setBodyCsys(tmpSim, tmpDomain.getBodyCsys());
        newFoil.setInletCsys(tmpDomain.getInletCsys());
        newFoil.setName(cadCSysName);
        newFoil.setRegion(tmpDomain.getRegion());
        newFoil.setUniqueBoundary(tmpFoilStr);

        // Set Tag and Prefix
        newFoil.addTags(tmpSim,
            Collections.singleton(this.getPrimaryDomainTag()));
        newFoil.setStarObjectPrefix(
            this.getPrimaryDomainTag().getPresentationName() + ".");

        allAirfoilObjects.add(newFoil);

        // If the Domain is OversetDomain, then the OversetDomain is responsible
        // for tracking the object inside of itself.
        if(tmpDomain instanceof OversetDomain){
          ((OversetDomain) tmpDomain).trackAirfoilObject(newFoil);
        }
        // Let Airfoil know about its new name.
        String foilName = newFoil.getName();
        // Airfoils are responsible for tracking their own reference values for
        // post-simulation reporting.
        newFoil.setReferenceValues(refRho, refVel, refArea, momentAxis);
        // Airfoils that are within an Overset Domain do not need to track their
        // individual angles as Overset Domains rotate everything.
        // Airfoils that are cut out of the wind tunnel must track their
        // individual rotations.
        if(tmpDomain instanceof OversetDomain){
          newFoil.setDeflectionAngle(0.0);
        }else{
          newFoil.setDeflectionAngle(
              MeshOpTool.getRotationAngle(tmpSim, "Rotate " + foilName)
              );
        }
        // If airfoil deflection angle is non-zero, the local body coordinate
        // must be
        // re-rotated to the correct alignment.
        if(Math.abs(newFoil.getDeflectionAngle()) > 1e-5){
            CartesianCoordinateSystem tmpCSys = newFoil.getRotationCsys();
            tmpCSys.setBasis0(new DoubleVector(xVector));
            tmpCSys.setBasis1(new DoubleVector(yVector));
            SimTool.rotateCoordinateSystem(tmpSim, newFoil.getDeflectionAngle(),
                zVector, tmpCSys, tmpCSys);
        }

        // Meshing Controls
        // Airfoils create their own Surface Control to control the mesh on a
        // per-airfoil element basis. Airfoil Objects control their own mesh
        // settings.
        AutoMeshOperation domSurfMeshOp = tmpDomain.getDomainSurfMeshOp();
        AutoMeshOperation domVolMeshOp = tmpDomain.getDomainVolumeMeshOp();

        // Title of Airfoil Custom surface settings.
        String cntrlTitleStr = tmpFoilStr +"00 Element "
            + tmpFoilStr.charAt(tmpFoilStr.length() - 1);

        // Let airfoil know about its surface mesh controls.
        SurfaceCustomMeshControl surfSurfControl =
            MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr);
        newFoil.setSurfMeshCustomControl(surfSurfControl);
        SurfaceCustomMeshControl surfSurfControl_TE =
            MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr+" TE");
        newFoil.setSurfMeshCustomTEControl(surfSurfControl_TE);

        // Let airfoil know about its volume mesh controls
        SurfaceCustomMeshControl volSurfControl =
            MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr);
        newFoil.setVolMeshCustomControl(volSurfControl);
        SurfaceCustomMeshControl volSurfControlTE =
            MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr+" TE");
        newFoil.setVolMeshCustomTEControl(volSurfControlTE);

        // Tne try catch protects against the case where there is no physics
        // instantiated for the domain. 
        try{
            //Make sure airfoil knows about its Reports
            newFoil.makeReports(tmpSim);
            newFoil.TagReports(
                tmpSim, Collections.singleton(this.getPrimaryDomainTag()));
            //Make sure airfoil knows about its Monitors
            int nPlotSamples=(int) 1.e5;
            int itMonFreq = 1;
            int itMonStart= 1;
            newFoil.makeIterationMonitors(
                tmpSim, nPlotSamples, itMonFreq, itMonStart);
            // Tag the foil monitors
            for(Monitor tmpMon : newFoil.getAllMonitors()){
              TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
            }
        }catch(NeoException e){
        }
      }
    }
  }
}
public void setup2DAirfoilTotalCenterOfPressure(
    Simulation tmpSim, ArrayList<Report> airfoilTotalReportsList){
  /* setup2DAirfoilTotalCenterOfPressure sets up the field functions required
     to obtain a center of pressure calculations on any 2D/Quasi 2D airfoil.

     Thie method assumes that setupTotalAirfoilReports has already been run.

     Center of pressure is automatically monitored and reported now. If this
     becomes too computationally heavy, then it will get reverted.
  */
  int nPlotSamples=(int) 5.e4;
  int tsMonFreq = 1;
  int tsMonStart= 1;
  ArrayList<PartSurface> allSurfaces=new ArrayList();

  // Create total part surfaces
  for(AerodynamicSurface tmp:allAirfoilObjects){
      for(PartSurface tmpSurf:tmp.getPartSurfaces()){
          allSurfaces.add(tmpSurf);
      }
  }

  // Setup User Field Functions
  UserFieldFunction uFF1 =
      SimTool.getUserFF(tmpSim,this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP - xF_y");
  UserFieldFunction uFF2 =
      SimTool.getUserFF(tmpSim,this.getPrimaryDomainTag().getPresentationName()
        + "." +  "Airfoil Total - CoP - yF_x");

  String csysFFRefName = "@CoordinateSystem(\"" + this.bodyCsys.getQualifiedName().replace("->",".") + "\")";
  uFF1.setDefinition("$$Centroid(" + csysFFRefName + ")[0]*(dot([0,1,0],(alternateValue($Pressure,0)*$$Area(" + csysFFRefName + "))))");
  uFF1.setFunctionName("cop_xf_y_" + this.getPrimaryDomainTag().getPresentationName());
  uFF2.setDefinition("$$Centroid(" + csysFFRefName + ")[1]*(dot([1,0,0],(alternateValue($Pressure,0)*$$Area(" + csysFFRefName + "))))");
  uFF2.setFunctionName("cop_yf_x_" + this.getPrimaryDomainTag().getPresentationName());

  // Setup Sum Reports to perform numerical integration
  SumReport xF_y = ReportTool.sumReport(tmpSim,
      this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP - Sum xF_y", uFF1, allSurfaces, proxyRep);
  xF_y.setFieldFunction(uFF1);
  TagTool.addTag(tmpSim, xF_y, this.getPrimaryDomainTag());
  SumReport yF_x = ReportTool.sumReport(tmpSim,
      this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP - Sum yF_x", uFF2, allSurfaces, proxyRep);
  yF_x.setFieldFunction(uFF2);
  TagTool.addTag(tmpSim, yF_x, this.getPrimaryDomainTag());

  // Get total element force in X and Y directions (Lab frame)
  ForceReport bodyFX = ReportTool.forceReport(tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Body Fx", bodyCsys, xVector, allSurfaces, proxyRep);
  ForceReport bodyFY = ReportTool.forceReport(tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Body Fy", bodyCsys, yVector, allSurfaces, proxyRep);  

  // Setup CoP x,y values
  ExpressionReport tmpRepX =
      ReportTool.getExpressionReport(tmpSim,
          this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP x");
  TagTool.addTag(tmpSim, tmpRepX, this.getPrimaryDomainTag());
  tmpRepX.setDefinition(
      "${" + xF_y.getQualifiedName() + "}/(${" + bodyFY.getQualifiedName()+"}+1.0E-20)");

  ExpressionReport tmpRepY =
      ReportTool.getExpressionReport(tmpSim,
          this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP y");
  TagTool.addTag(tmpSim, tmpRepY, this.getPrimaryDomainTag());
  tmpRepY.setDefinition(
      "${" + yF_x.getQualifiedName() + "}/(${" + bodyFX.getQualifiedName()+"}+1.0E-20)");

  Monitor tmpRepXMon = MonitorTool.reportIterationMonitor(
          tmpSim, tmpRepX, nPlotSamples, tsMonFreq, tsMonStart);
  TagTool.addTag(tmpSim, tmpRepXMon, primaryWTDomainTag);
  Monitor tmpRepYMon = MonitorTool.reportIterationMonitor(
          tmpSim, tmpRepY, nPlotSamples, tsMonFreq, tsMonStart);
  TagTool.addTag(tmpSim, tmpRepYMon, primaryWTDomainTag);
  
  tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total CmZ");
  MonitorPlot copXPlot = PlotTool.getMonitorPlot(
      tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP x",
      this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP x");
  TagTool.addTag(tmpSim, copXPlot, primaryWTDomainTag);
  MonitorPlot copYPlot = PlotTool.getMonitorPlot(
      tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP y",
      this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Total - CoP y");
  TagTool.addTag(tmpSim, copXPlot, primaryWTDomainTag);

  copXPlot.close();
  copYPlot.close();
  
  PlotTool.addDataSet(tmpSim, copXPlot, tmpRepXMon.getPresentationName(), "CoP X-Location in Body");
  PlotTool.addDataSet(tmpSim, copYPlot, tmpRepYMon.getPresentationName(), "CoP Y-Location in Body");

}
public void setup2DAirfoilTotalWallYPlus(Simulation tmpSim){
  /* setup2DAirfoilTotalWallYPlus creates a wall y+ plot that contains all the
     airfoil elements.
  */
  ArrayList<PartSurface> allSurfaces=new ArrayList();
  // Create total part surfaces
  for(AerodynamicSurface tmp:allAirfoilObjects){
      for(PartSurface tmpSurf:tmp.getPartSurfaces()){
          allSurfaces.add(tmpSurf);
      }
  }
  XYPlot tmpXYPlot =
      PlotTool.getXYPlot(tmpSim,this.getPrimaryDomainTag().getPresentationName()
        + "." +  "All Airfoil Wall y+", "Wall Y +");
  TagTool.addTag(tmpSim, tmpXYPlot, this.getPrimaryDomainTag());
  tmpXYPlot.getParts().setObjects(allSurfaces);
  tmpXYPlot.setRepresentation(proxyRep);
  PlotTool.setXYPlotXVectorScalar(tmpSim, tmpXYPlot, bodyCsys, "Position", 0);
  PlotTool.setXYPlotYPrimitiveScalar(tmpSim, tmpXYPlot, "WallYplus");

  // set limits to body csys origin +- chord length
  Cartesian2DAxisManager cartesian2DAxisManager = 
    ((Cartesian2DAxisManager) tmpXYPlot.getAxisManager());
  Cartesian2DAxis cartesian2DAxis = 
    ((Cartesian2DAxis) cartesian2DAxisManager.getAxis("Bottom Axis"));
  cartesian2DAxis.setReverse(true);
  double xMin = 
      bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()-chordLength;
  double xMax = 
      bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()+chordLength;
  PlotTool.setPlotXAxis(
      tmpXYPlot, "Position in Body Csys x [m]", xMin, xMax,
      chordLength / 4.0, 3
      );
  //
  PlotTool.setPlotYToLog(tmpXYPlot, true);
  YAxisType yAxisType_0 = 
    ((YAxisType) tmpXYPlot.getYAxes().getAxisType("Y Type 1"));
  tmpXYPlot.close();
}
public void setup2DAirfoilTotalCfPlot(Simulation tmpSim){
  // Skin friction coefficient plot for airfoils
  // Each element is a new legend entry in the plot
  if(allAirfoilObjects.size()>0){
    XYPlot tmpXYPlot;
    tmpXYPlot = PlotTool.getXYPlot(tmpSim,
        this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Cf", "Skin Friction Coefficient");
    TagTool.addTag(tmpSim, tmpXYPlot, primaryWTDomainTag);
    tmpXYPlot.setRepresentation(proxyRep);
    // x-axis setup (body coordinate system)
    tmpSim.println("WT 2D MSG: Setting up position FF x-axis");
    AxisType xAxisType = tmpXYPlot.getXAxisType();
    xAxisType.setMode(AxisTypeMode.SCALAR);
    FieldFunctionUnits ffUnits = xAxisType.getScalarFunction();
    PrimitiveFieldFunction pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("Position"));
    VectorComponentFieldFunction vectorCFF = 
      ((VectorComponentFieldFunction) pFF.getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(0));
    ffUnits.setFieldFunction(vectorCFF);
    // set limits to body csys origin +- chord length
    Cartesian2DAxisManager cartesian2DAxisManager = 
      ((Cartesian2DAxisManager) tmpXYPlot.getAxisManager());
    Cartesian2DAxis cartesian2DAxis = 
      ((Cartesian2DAxis) cartesian2DAxisManager.getAxis("Bottom Axis"));
    cartesian2DAxis.setReverse(true);
    double xMin=bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()-chordLength;
    double xMax=bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()+chordLength;
    PlotTool.setPlotXAxis(tmpXYPlot, "Position in Body Csys x [m]", xMin, xMax,chordLength/4.0, 3);
    PlotTool.setPlotYAxis(tmpXYPlot, "Skin Friction Coefficient", 0.0, 0.05,0.01, 4);        

    // y-axis setup (Skin Friction Coefficient) - one for each object
    YAxisType yAxisType_0 = 
      ((YAxisType) tmpXYPlot.getYAxes().getAxisType("Y Type 1"));
    FieldFunctionUnits fieldFunctionUnits_0 = 
      yAxisType_0.getScalarFunction();
    SkinFrictionCoefficientFunction skinFrictionCoefficientFunction_0 = 
      ((SkinFrictionCoefficientFunction) tmpSim.getFieldFunctionManager().getFunction("SkinFrictionCoefficient"));
    yAxisType_0.setSmooth(true);
    fieldFunctionUnits_0.setFieldFunction(skinFrictionCoefficientFunction_0);

    ArrayList<PartSurface> tmpSurfs = new ArrayList();
    for(AerodynamicSurface tmpFoil:allAirfoilObjects){
      tmpSurfs.addAll(tmpFoil.getPartSurfaces());
    }
    tmpSim.println("WT 2D MSG: Setting objects");
    tmpXYPlot.setObjects(tmpSurfs);

    //set all to skin friction coefficient
    tmpSim.println("WT 2D MSG: Setting objects to display Cf");
    tmpXYPlot.close();
  }
}
public void setup2DAirfoilTotalCpPlot(Simulation tmpSim){
    // Skin friction coefficient plot for airfoils
    // Each element is a new legend entry in the plot
    if(allAirfoilObjects.size()>0){
      XYPlot tmpXYPlot;
      tmpXYPlot = PlotTool.getXYPlot(tmpSim,
          this.getPrimaryDomainTag().getPresentationName()
        + "." + "Airfoil Cp", "Pressure Coefficient");
      TagTool.addTag(tmpSim, tmpXYPlot, primaryWTDomainTag);
      tmpXYPlot.setRepresentation(proxyRep);
      // x-axis setup (body coordinate system)
      AxisType xAxisType = tmpXYPlot.getXAxisType();
      xAxisType.setMode(AxisTypeMode.SCALAR);
      FieldFunctionUnits ffUnits = xAxisType.getScalarFunction();
      PrimitiveFieldFunction pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("Position"));
      VectorComponentFieldFunction vectorCFF = 
        ((VectorComponentFieldFunction) pFF.getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(0));
      ffUnits.setFieldFunction(vectorCFF);
      // set limits to body csys origin +- chord length
      Cartesian2DAxisManager cartesian2DAxisManager = 
        ((Cartesian2DAxisManager) tmpXYPlot.getAxisManager());
      Cartesian2DAxis cartesian2DAxis = 
        ((Cartesian2DAxis) cartesian2DAxisManager.getAxis("Bottom Axis"));
      cartesian2DAxis.setReverse(true);
      double xMin=bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()-chordLength;
      double xMax=bodyCsys.getOrigin().getInternalVector().get(0).doubleValue()+chordLength;
      PlotTool.setPlotXAxis(tmpXYPlot, "Position in Body Csys x [m]", xMin, xMax,chordLength/4.0, 3);
      PlotTool.setPlotYAxis(tmpXYPlot, "Pressure Coefficient", -10.0, 3.0, 1.0, 4);
      Cartesian2DAxis cartesian2DAxis_0 = 
        ((Cartesian2DAxis) cartesian2DAxisManager.getAxis("Left Axis"));
      cartesian2DAxis_0.setReverse(true);

      // y-axis setup (Skin Friction Coefficient) - one for each object
      YAxisType yAxisType_0 = 
        ((YAxisType) tmpXYPlot.getYAxes().getAxisType("Y Type 1"));
      FieldFunctionUnits fieldFunctionUnits_0 = 
        yAxisType_0.getScalarFunction();
      PressureCoefficientFunction pressureCoefFunc = 
        ((PressureCoefficientFunction) tmpSim.getFieldFunctionManager().getFunction("PressureCoefficient"));
      yAxisType_0.setSmooth(true);
      fieldFunctionUnits_0.setFieldFunction(pressureCoefFunc);
      ArrayList<PartSurface> tmpSurfs = new ArrayList();
      for(AerodynamicSurface tmpFoil:allAirfoilObjects){
        tmpSurfs.addAll(tmpFoil.getPartSurfaces());
      }
      tmpXYPlot.setObjects(tmpSurfs);
      //set all to skin friction coefficient
      tmpXYPlot.close();
    }
}
private String getTotalAirfoilReportCSVData(Simulation tmpSim,int numSamples){
  String wtTagName = this.getPrimaryDomainTag().getPresentationName();
  String retString="";
  //
  String rootName = wtTagName + "." + "Aero Airfoil Total";
  String appEnd=" - It";
  String tmpName;
  //
  double aveVal;
  double stdDev;
  Report tmpRep;
  // special
  double cLcD;

  tmpName=rootName+" - CL";
  tmpRep = ReportTool.getReport(tmpSim,tmpName);
  aveVal = SimTool.getMeanReportItVal(tmpRep,appEnd,numSamples);
  stdDev = SimTool.getMeanReportStddevItVal(tmpRep,appEnd,numSamples,aveVal);
  cLcD=aveVal;
  retString = retString+aveVal;//+","+stdDev; //first entry has no comma
  //
  tmpName=rootName+" - CD";
  tmpRep = ReportTool.getReport(tmpSim,tmpName);
  aveVal = SimTool.getMeanReportItVal(tmpRep,appEnd,numSamples);
  stdDev = SimTool.getMeanReportStddevItVal(tmpRep,appEnd,numSamples,aveVal);
  cLcD=cLcD/aveVal;
  retString = retString+","+aveVal;//+","+stdDev;
  //
  tmpName=rootName+" - CmZ";
  tmpRep = ReportTool.getReport(tmpSim,tmpName);
  aveVal = SimTool.getMeanReportItVal(tmpRep,appEnd,numSamples);
  stdDev = SimTool.getMeanReportStddevItVal(tmpRep,appEnd,numSamples,aveVal);
  retString = retString+","+aveVal;//+","+stdDev;
  //CL/CD
  retString = retString+","+cLcD;
  //

  return retString;
}

public ArrayList<Report> getAeroTotalAirfoilReports(Simulation tmpSim){
  /* Returns the 3 primary force directions
  */
  String tagName = this.primaryWTDomainTag.getPresentationName();
  String rootName = tagName + ".Aero Airfoil Total";
  ArrayList<Report> tmpList = new ArrayList();
  //Force Coefficients.
  String nameCD = rootName + " - CD";
  String nameCL = rootName + " - CL"; 
  String nameCM = rootName + " - CmZ";
  tmpList.add(ReportTool.getReport(tmpSim, nameCL));
  tmpList.add(ReportTool.getReport(tmpSim, nameCD));
  tmpList.add(ReportTool.getReport(tmpSim, nameCM));
  return tmpList;
}

private ArrayList<Report> setupTotalAirfoilReports(Simulation tmpSim, 
    String preFix, CoordinateSystem relCsys){
  /* setupTotalAirfoilPerformanceReports sets up the forces and moments and the
     forces and moment coefficients for the entirety of the aiirfoil that is
     collected inside the domain.
  */
  String rootName = preFix + " Airfoil Total";
  ArrayList<PartSurface> allSurfaces = new ArrayList();
  ArrayList<Report> tmpRepList = new ArrayList();
  // Create total part surfaces.
  for(AerodynamicSurface tmp : allAirfoilObjects){
    for(PartSurface tmpSurf : tmp.getPartSurfaces()){
      allSurfaces.add(tmpSurf);
    }
  }

  // Instantiate.
  ForceReport  fX; ForceReport  fY; ForceReport  fZ;
  MomentReport mX; MomentReport mY; MomentReport mZ;
  String xName = "";
  String yName = "";
  String zName = "";
  ForceCoefficientReport cX; MomentCoefficientReport cMX;
  ForceCoefficientReport cY; MomentCoefficientReport cMY;
  ForceCoefficientReport cZ; MomentCoefficientReport cMZ;

  // Forces.
  fX = ReportTool.forceReport(
      tmpSim, rootName + " - FX", relCsys, xVector, allSurfaces, proxyRep
      );
  fY = ReportTool.forceReport(
      tmpSim, rootName + " - FY", relCsys, yVector, allSurfaces, proxyRep
      );
  fZ = ReportTool.forceReport(
      tmpSim, rootName + " - FZ", relCsys, zVector, allSurfaces, proxyRep
      );

  if(!tmpRepList.contains(fX)) tmpRepList.add(fX);
  if(!tmpRepList.contains(fY)) tmpRepList.add(fY);
  if(!tmpRepList.contains(fZ)) tmpRepList.add(fZ);

  //Moments
  mX = ReportTool.momentReport(tmpSim,
      rootName+" - MX", relCsys, zeroVector, xVector, allSurfaces, proxyRep
      );
  mY = ReportTool.momentReport(tmpSim,
      rootName+" - MY", relCsys, zeroVector, yVector, allSurfaces, proxyRep
      );
  mZ = ReportTool.momentReport(tmpSim,
      rootName+" - MZ", relCsys, zeroVector, zVector, allSurfaces, proxyRep
      );
  if(!tmpRepList.contains(mX)) tmpRepList.add(mX);
  if(!tmpRepList.contains(mY)) tmpRepList.add(mY);
  if(!tmpRepList.contains(mZ)) tmpRepList.add(mZ);
  
  //Force Coefficients.
  if(preFix.contains("Aero")){
      xName=" - CD"; yName=" - CY"; zName=" - CL";
  }else if(preFix.contains("Body")){
      xName=" - CX"; yName=" - CY"; zName=" - CZ";
  }
  cX = ReportTool.forceCoefReport(tmpSim,
      rootName + xName, relCsys, xVector, refRho, refVel, refArea,
      allSurfaces, proxyRep
      );
  cY = ReportTool.forceCoefReport(tmpSim,
      rootName + yName, relCsys, yVector, refRho, refVel, refArea,
      allSurfaces, proxyRep
      );
  cZ = ReportTool.forceCoefReport(tmpSim,
      rootName + zName, relCsys, yVector, refRho, refVel, refArea,
      allSurfaces, proxyRep
      );
  if(!tmpRepList.contains(cX)) tmpRepList.add(cX);
  if(!tmpRepList.contains(cY)) tmpRepList.add(cY);
  if(!tmpRepList.contains(cZ)) tmpRepList.add(cZ);

  //Moment Coefficients
  cMX = ReportTool.momentCoefReport(tmpSim,
      rootName+" - CmX",relCsys, zeroVector, xVector,
      refRho, refVel, refArea, chordLength, allSurfaces, proxyRep);
  cMY = ReportTool.momentCoefReport(tmpSim,
      rootName+" - CmY",relCsys, zeroVector, yVector,
      refRho, refVel, refArea, chordLength, allSurfaces, proxyRep);
  cMZ = ReportTool.momentCoefReport(tmpSim,
      rootName+" - CmZ",relCsys, zeroVector, zVector, refRho,
      refVel, refArea, chordLength, allSurfaces, proxyRep);
  if(!tmpRepList.contains(cMX)) tmpRepList.add(cMX);
  if(!tmpRepList.contains(cMY)) tmpRepList.add(cMY);
  if(!tmpRepList.contains(cMZ)) tmpRepList.add(cMZ);

  return tmpRepList;
}
public void applyAirfoilStandardMeshSettings(Simulation tmpSim){
  for(AerodynamicSurface tmpFoil : allAirfoilObjects){
    //tmpSim.println("Apply mesh settings for: "+tmpFoil.getName());
    // custom Surface Mesh controls per airfoil
    //   Best practice values:
    double foilTargetPct   =  12.5;
    double foilMinPct      =  3.125;
    double foilNPtsCircle  =  54.0;
    double foilTETargetPct =   foilTargetPct*0.25;
    double foilTEMinPct    =   foilTargetPct*0.125;

    //SURFACE MESH SETTINGS
    // Apply custom surface mesh settings on the airfoils.
    SurfaceCustomMeshControl surfSurfControl = 
        tmpFoil.getSurfMeshCustomControl();
    tmpFoil.setSurfMeshCustomControl(surfSurfControl);
    MeshOpTool.surfFilter(      surfSurfControl,2);
    MeshOpTool.surfCustTargSize(surfSurfControl,"Relative",foilTargetPct);
    MeshOpTool.surfCustMinSize( surfSurfControl,"Relative",foilMinPct);
    MeshOpTool.surfNCircle(     surfSurfControl,foilNPtsCircle);

    //
    // Apply custom surface mesh settings on the trailing edges of the airfoils.
    SurfaceCustomMeshControl surfSurfControl_TE = 
        tmpFoil.getSurfMeshCustomTEControl();
    tmpFoil.setSurfMeshCustomTEControl(surfSurfControl_TE);
    MeshOpTool.surfFilter_TE(  surfSurfControl_TE,2);
    MeshOpTool.surfCustMinSize(surfSurfControl_TE,"Relative", foilTEMinPct);
    MeshOpTool.surfEdgeProx(   surfSurfControl_TE,1);

    //VOLUME MESH SETTINGS
    // Defaults
    int nPrismLayers = 28; //use even #s
    double prismThickness=chordLength*0.025;
    double firstCellThick=chordLength*5e-6;
    double wakeControlAngle=20.0;

    //Custom volume control settings
    SurfaceCustomMeshControl volSurfControl= tmpFoil.getVolMeshCustomControl();
    tmpFoil.setVolMeshCustomControl(volSurfControl);
    //main body control
    MeshOpTool.surfFilter(volSurfControl,2);
    MeshOpTool.surfPrismThick( volSurfControl,"Absolute",prismThickness);
    MeshOpTool.surfPrismNearWall(volSurfControl,firstCellThick);
    MeshOpTool.surfPrismOverride( volSurfControl, true);
    MeshOpTool.surfPrismNumLayers(volSurfControl,nPrismLayers);

    // Create automatic wake refinement coordinate system.
    // Use a specific CAD Zero axis so that the mesh operation does not go out
    // on coordinate system rotations.
    double wakeMultiplier = 1.0;
    CartesianCoordinateSystem cadZeroBodyCsys = SimTool.getNestedCoordinate(
         wtCsys, "CAD Zero " + bodyCsys.getPresentationName()
        );
    MeshOpTool.activateWakeVolumeControl(cadZeroBodyCsys, volSurfControl,
        chordLength, foilTargetPct * wakeMultiplier, wakeControlAngle
        );
    tmpFoil.setVolMeshCustomControl(volSurfControl);

    //trailing edges
    SurfaceCustomMeshControl volSurfControl_TE =
        tmpFoil.getVolMeshCustomTEControl();
    MeshOpTool.surfFilter_TE(  volSurfControl_TE, 2);
    MeshOpTool.surfPrismThick(
        volSurfControl_TE, "Absolute", prismThickness * 0.25
        );
    MeshOpTool.surfPrismNearWall(volSurfControl_TE, firstCellThick * 4.0);
    MeshOpTool.surfPrismOverride(volSurfControl_TE, true);
    MeshOpTool.surfPrismNumLayers(volSurfControl_TE, nPrismLayers / 2);
    MeshOpTool.activateWakeVolumeControl(
        cadZeroBodyCsys, volSurfControl_TE, chordLength,
        foilTargetPct * wakeMultiplier, wakeControlAngle);
    tmpFoil.setVolMeshCustomTEControl(volSurfControl_TE);
  }
}
public void setAirfoilSurfaceMeshSettings(ArrayList<Boolean> af_CustomMesh,
  ArrayList<Double>  af_TargetSize, ArrayList<Double> af_MinSize,
  ArrayList<Double> af_NPtsOnCircle){

  int aF=0; //airFoil element #
  for(boolean customFoil:af_CustomMesh){
    if(customFoil){
      DeflectableControlSurface thisFoil =
          (DeflectableControlSurface) getAirfoilElement( aF);
      //Set custom surface sizes
      thisFoil.setCustomTargetSurfaceSize(af_TargetSize.get(aF));
      thisFoil.setCustomMinSurfaceSize(af_MinSize.get(aF));
      thisFoil.setCustomNPtsOnCircle(af_NPtsOnCircle.get(aF));
    }
    aF+=1;
  }
}
public void setAirfoilVolumeMeshSettings(Simulation tmpSim,
  ArrayList<Boolean> af_CustomMesh,
  ArrayList<Boolean> af_LowYPlus,ArrayList<Boolean> af_HighYPlus,
  ArrayList<Double>  af_PrismAbsThick, ArrayList<Double> af_1stCellThick,
  ArrayList<Integer> af_NPrisms){

  int aF = 0; //airFoil element #
  for(boolean customFoil : af_CustomMesh){
    //truly custom mesh settings
    if(customFoil){
      DeflectableControlSurface thisFoil =
          (DeflectableControlSurface) getAirfoilElement(aF);
      //Set custom prism mesh settings
      thisFoil.setCustomFirstCellThickness(af_1stCellThick.get(aF));
      thisFoil.setCustomPrismAbsThickness(af_PrismAbsThick.get(aF));
      thisFoil.setCustomNumPrismLayers(af_NPrisms.get(aF));
    }
    //custom mesh settings applied for this one
    aF+=1;
  }
}
public void modifyAirfoilSetAngles(Simulation tmpSim, double[] newAngles){
  int afElementNumber=0;
  for(AerodynamicSurface tmpAF : allAirfoilObjects){
    double tmpAngle=newAngles[afElementNumber];
    //cannot rotate an Overset Domain's airfoil object
    //  can only rotate the overset domain
    boolean airfoilIsOverset=false;
    for(OversetDomain tmpDomain:allOversetDomains){
      for(AerodynamicSurface tmpSurf:tmpDomain.getAirfoilObjects()){
      }
      if(tmpDomain.getAirfoilObjects().contains(tmpAF)){
        airfoilIsOverset=true;
      }
    }

    if(tmpAngle<360.0&&!airfoilIsOverset){
      modifyAirfoilSetAngle(afElementNumber,tmpAngle);
      //Mesh in place specific
      String foilOpName="Rotate "+tmpAF.getName();
      MeshOpTool.setRotationAngle(tmpSim,foilOpName, tmpAngle);
      MeshOpTool.setRotationAxis(tmpSim,foilOpName, new double[] {0.,0.,-1.0});

      if(Math.abs(tmpAngle)>1e-5){
        CartesianCoordinateSystem tmpCSys =
            ((DeflectableControlSurface) tmpAF).getRotationCsys();
        tmpCSys.setBasis0(new DoubleVector( new double[] {1.0,0.0,0.0}));
        tmpCSys.setBasis1(new DoubleVector( new double[] {0.0,1.0,0.0}));
      }
    }
    afElementNumber+=1;
  }
}
private void modifyAirfoilSetAngle(int elementNumber,double newAngle){
  DeflectableControlSurface thisFoil=
          (DeflectableControlSurface) getAirfoilElement(elementNumber);
  thisFoil.setDeflectionAngle(newAngle);
}
public void modifyAirfoilSurfaceMeshSettings(
    int elementNumber,double targetSize,double minSize,int nPtsOnCircle){
  DeflectableControlSurface thisFoil=
          (DeflectableControlSurface) getAirfoilElement(elementNumber);
  double oldTargetSize = thisFoil.getCustomTargetSurfaceSize();
  double oldMinSize = thisFoil.getCustomMinSurfaceSize();
  double oldPtsOnCircle = thisFoil.getNPtsOnCircle();
  if(Math.abs(oldTargetSize-targetSize) > 1.0e-5 ||
      Math.abs(oldMinSize-minSize) > 1.e-5 ||
      (oldPtsOnCircle-nPtsOnCircle) > 1.0e-5){
    thisFoil.setCustomTargetSurfaceSize(targetSize);
    thisFoil.setCustomMinSurfaceSize(minSize);
    thisFoil.setCustomNPtsOnCircle(nPtsOnCircle);
  }
}
public void modifyAirfoilPrismMeshSettings(
    int elementNumber, double prismAbsoluteThickness,
    double firstCellThickness, int numPrisms){
  DeflectableControlSurface thisFoil=
          (DeflectableControlSurface) getAirfoilElement(elementNumber);
  double oldPrismThickness = thisFoil.getCustomPrismAbsThickness();
  double oldFirstCellThickness = thisFoil.getCustomFirstCellThickness();
  int oldNumberPrisms = thisFoil.getCustomNumPrismLayers();
  if(Math.abs(prismAbsoluteThickness-oldPrismThickness) > 1.0e-5 ||
      Math.abs(firstCellThickness-oldFirstCellThickness) > 1.0e-5 ||
      (numPrisms-oldNumberPrisms) > 0){
    thisFoil.setCustomNumPrismLayers(numPrisms);
    thisFoil.setCustomFirstCellThickness(firstCellThickness);
    thisFoil.setCustomPrismAbsThickness(prismAbsoluteThickness);
  }
}
private void removeEmptyAirfoilStr(ArrayList<GeometryPart> airfoilParts){
  ArrayList<String> retList = new ArrayList();
  for(GeometryPart tmpPart : airfoilParts){
    for(PartSurface tmpSurf : tmpPart.getPartSurfaces()){
      String tmpName = tmpSurf.getPresentationName();
      String preFix = tmpName.substring(0,2);
      if(airfoilStrList.contains(preFix)){
         retList.add(preFix);
      }
      break;
    }
  }
  airfoilStrList.retainAll(retList);
}

// BOUNDARY LAYER PROBES
public void instantiateBoundaryLayerProbes(Simulation tmpSim){
  /* Method instantiateBoundaryLayerProbes (BLPs) figures out what existing
     BoundaryLayerProbe objects already exist in the simulation and instantiates
     their objects. This method is strictly for extraction of existing objects
     in a sim. If there already BLP objects in the domain, the process is
     skipped.
  */
    tmpSim.println("WT MSG: Instantiating Boundary Layer Probe Objects...");
  if(!(allBoundaryLayerProbes.size() > 0)){
    // Find BLP parts
    Collection<GeometryPart> allBLPCADParts = getBoundaryLayerProbeParts(tmpSim);

    for(GeometryPart tmpPart : allBLPCADParts){
      tmpSim.println(tmpPart.getPresentationName());
      BoundaryLayerProbe thisBLP = new BoundaryLayerProbe(tmpSim, tmpPart, wtCsys);
      thisBLP.getProbePart().setPresentationName(this.getName() + "." + thisBLP.getName());
      thisBLP.setBodyCsys(this.bodyCsys);
      thisBLP.setProbeRegions(this.getRegionsOfControlledObjects());
      
      ArrayList<Boundary> allWallBoundaries = new ArrayList();
      for(Region thisReg : this.getRegionsOfControlledObjects()){
          for(Boundary thisBndy : thisReg.getBoundaryManager().getObjects()){
              if (thisBndy.getBoundaryType() instanceof WallBoundary){
                  allWallBoundaries.add(thisBndy);
              }
          }
      }
      thisBLP.setProbeBoundaries(allWallBoundaries);

      //Make sure Boundary Layer Probes knows this probe exists!
      this.addBoundaryLayerProbe(tmpSim, thisBLP);
    }
  }
}
private ArrayList<GeometryPart> getBoundaryLayerProbeParts(Simulation tmpSim){
  /* Figures out what Boundary Layer Probe Parts are actually in the windtunnel. */
  Collection<GeometryPart> allGeomParts = tmpSim.getGeometryPartManager().getObjects();
  ArrayList<GeometryPart> retArr= new ArrayList();
  for(GeometryPart tmpPart : allGeomParts){
    if(tmpPart instanceof CompositePart){
      for(GeometryPart childPart:((CompositePart) tmpPart).getChildParts().getObjects()){
        String childPartName=childPart.getPresentationName();
        if(childPartName.startsWith(globalNames.getBLPPreFix()) &&
              TagTool.getObjectTags(childPart).contains(primaryWTDomainTag)){
          retArr.add(childPart);
        }
      }
    }else{
      String tmpPartName=tmpPart.getPresentationName();
      if(tmpPartName.startsWith(globalNames.getBLPPreFix())){
          retArr.add(tmpPart);
      }
    }
  }
  return retArr;
}

private void outputBLPMeshStatusScene(
    Simulation tmpSim,String outputDir, String fileName){
  SystemTool.touchDirectoryChain(tmpSim, outputDir);

  // Prism Layer Resolution Graphic
  Scene tmpCFDScene;
  PartDisplayer tmpPD;
  ArrayList<Part> failBLPParts = new ArrayList();
  ArrayList<Part> marginalBLPParts = new ArrayList();
  ArrayList<Part> passBLPParts = new ArrayList();
  ArrayList<Part> highYPlusBLPParts = new ArrayList();
  ArrayList<Part> unknownBLPParts = new ArrayList();
  
  String wtTagName = this.getPrimaryDomainTag().getPresentationName();

  for(BoundaryLayerProbe tmpProbe : allBoundaryLayerProbes){
    if(tmpProbe.getResolutionStatus().equals("FAIL")){
      failBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("HIGH Y+")){
      highYPlusBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("PASS")){
      passBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("MARGINAL")){
      marginalBLPParts.add(tmpProbe.getProbePart());
    }else{
      unknownBLPParts.add(tmpProbe.getProbePart());
    }
  }
  tmpCFDScene = SceneTool.getScene(
      tmpSim, wtTagName+"."+"CFD Boundary Layer Probe Status");

  ArrayList<Domain> allDomainObjects = new ArrayList();
  allDomainObjects.add(this);
  allDomainObjects.addAll(allOversetDomains);
  ArrayList<Region> regionObjects = new ArrayList();
  for(Domain tmpDomain : allDomainObjects){
    regionObjects.add(tmpDomain.getRegion());
  }
  ArrayList<Boundary> wallBoundaries = new ArrayList();
  for(Domain tmpDomain : allDomainObjects){
    for(Boundary tmpBndy : 
        tmpDomain.getRegion().getBoundaryManager().getBoundaries()
        ){
      if(tmpBndy.getBoundaryType() instanceof WallBoundary){
        wallBoundaries.add(tmpBndy);
      }
    }
  }
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "Mesh", regionObjects, proxyRep
      );
  tmpPD.getInputParts().addParts(wallBoundaries);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("slate gray")));
  tmpPD.setMesh(true);
  tmpPD.setMeshColor(new DoubleVector(PlotTool.getColor("gainsboro")));
  // FAIL = red
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "FAIL", failBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("red")));
  // MARGINAL = yellow
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "MARGINAL", marginalBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("yellow")));
  // PASS = bright green
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "PASS", passBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);      
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("bright green")));
  // HIGH Y+ = magenta
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "HIGHYPLUS", highYPlusBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);     
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("magenta")));
  // UNKNWON STATUS = black
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "UNKNOWN", unknownBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);     
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("black")));
  //standard annotations
  SceneTool.removeDefaultLogo(tmpCFDScene);
  for(Annotation tmpAnn : standardAnnotations){
    SceneTool.addAnnotation(tmpCFDScene,tmpAnn);
  }

  // generic save out name
  tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("blp_status");

  // print out
  double[] wt2DfocalPt = {wt2DCameraPos[0], 0., 0.};
  VisView wt2DViewZoomed = SceneTool.getView(tmpSim, 
      wtTagName + "." + "WT 2D Auto View Zoomed",
      bodyCsys, wt2DfocalPt, wt2DCameraPos, yVector, true);
  wt2DViewZoomed.getParallelScale().setValue(0.5 * chordLength);
  Units units_0 = 
    ((Units) tmpSim.getUnitsManager().getObject("m"));
  wt2DViewZoomed.getFocalPointCoordinate().setCoordinate(
      units_0, units_0, units_0, new DoubleVector(
          new double[] {wt2DCameraPos[0], 0.0, 0.0}
          )
      );
  wt2DViewZoomed.getViewUpCoordinate().setCoordinate(
      units_0, units_0, units_0, new DoubleVector(
          new double[] {0.0, -1.0, 0.0}
          )
      );
  wt2DViewZoomed.getPositionCoordinate().setCoordinate(
      units_0, units_0, units_0, new DoubleVector(
          new double[] {wt2DCameraPos[0], 0.0, wt2DCameraPos[2]}
        )
      );
  tmpCFDScene.getCurrentView().setView(wt2DViewZoomed);
  tmpCFDScene.printAndWait(outputDir + "BLP_MeshScene_" + fileName + ".png",
      1, xyRes[0], xyRes[1], true, false);
}
private void outputBLPFlowStatusScene(
    Simulation tmpSim,String outputDir, String fileName){
  SystemTool.touchDirectoryChain(tmpSim, outputDir);
  // Prism Layer Resolution Graphic
  Scene tmpCFDScene;
  PartDisplayer tmpPD;
  ScalarDisplayer tmpSD;
  ArrayList<Part> failBLPParts = new ArrayList();
  ArrayList<Part> marginalBLPParts = new ArrayList();
  ArrayList<Part> passBLPParts = new ArrayList();
  ArrayList<Part> highYPlusBLPParts = new ArrayList();
  ArrayList<Part> unknownBLPParts = new ArrayList();
  String wtTagName = this.getPrimaryDomainTag().getPresentationName();

  for(BoundaryLayerProbe tmpProbe:allBoundaryLayerProbes){
    if(tmpProbe.getResolutionStatus().equals("FAIL")){
      failBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("HIGH Y+")){
      highYPlusBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("PASS")){
      passBLPParts.add(tmpProbe.getProbePart());
    }else if(tmpProbe.getResolutionStatus().equals("MARGINAL")){
      marginalBLPParts.add(tmpProbe.getProbePart());
    }else{
      unknownBLPParts.add(tmpProbe.getProbePart());
    }
  }

  tmpCFDScene = SceneTool.getScene(tmpSim,
      wtTagName + "." + "CFD Boundary Layer Probe Flow Status");

  // Gather parts
  ArrayList<Domain> allDomainObjects = new ArrayList();
  allDomainObjects.add(this);
  allDomainObjects.addAll(allOversetDomains);
  ArrayList<Region> regionObjects = new ArrayList();
  for(Domain tmpDomain:allDomainObjects){
    regionObjects.add(tmpDomain.getRegion());
  }
  ArrayList<Boundary> wallBoundaries = new ArrayList();
  for(Domain tmpDomain : allDomainObjects){
    for(Boundary tmpBndy : tmpDomain.getRegion()
        .getBoundaryManager().getBoundaries()){
      if(tmpBndy.getBoundaryType() instanceof WallBoundary){
        wallBoundaries.add(tmpBndy);
      }
    }
  }
  tmpSD = SceneTool.getScalarDisplayer(tmpCFDScene, "Flow", regionObjects);
  tmpSD.getInputParts().addParts(wallBoundaries);
  tmpSD.setMeshColor(new DoubleVector(PlotTool.getColor("black")));

  // Velocity magnitude
  PrimitiveFieldFunction pFF =
      ((PrimitiveFieldFunction) tmpSim
          .getFieldFunctionManager().getFunction("Velocity")
      );
  VectorMagnitudeFieldFunction vMFF=
      ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
  SceneTool.setScalarDisplayerField(
      tmpSD, vMFF.getInternalName(),"Off","Off",0.0,refVel,proxyRep
      );
  tmpSD.setDisplayMesh(1);
  tmpSD.setFillMode(ScalarFillMode.CELL_FILLED);
  // white water
  Legend scalarLegend = tmpSD.getLegend();
  scalarLegend.setLookupTable(
      tmpSim.get(LookupTableManager.class)
          .getObject(ColorMapTool.getWhiteWaterString(tmpSim))
      );
  scalarLegend.setReverse(true);
  scalarLegend.setNumberOfLabels(5);

  // FAIL = red
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "FAIL", failBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("red")));

  // MARGINAL = yellow
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "MARGINAL", marginalBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("yellow")));
  // PASS = bright green
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "PASS", passBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);      
  tmpPD.setDisplayerColor(
      new DoubleVector(PlotTool.getColor("bright green"))
      );
  // HIGH Y+ = magenta
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "HIGHYPLUS", highYPlusBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);     
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("magenta")));
  // UNKNWON STATUS = black
  tmpPD = SceneTool.getPartDisplayer(
      tmpCFDScene, "UNKNOWN", unknownBLPParts, proxyRep
      );
  tmpPD.setLineWidth(10.0);
  tmpPD.setSurface(true);
  tmpPD.setColorMode(PartColorMode.CONSTANT);     
  tmpPD.setDisplayerColor(new DoubleVector(PlotTool.getColor("black")));
  //standard annotations
  SceneTool.removeDefaultLogo(tmpCFDScene);
  for(Annotation tmpAnn:standardAnnotations){
    SceneTool.addAnnotation(tmpCFDScene,tmpAnn);
  }
  //generic save out name
  tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("blp_status");

  // print out
  double[] wt2DfocalPt={wt2DCameraPos[0], 0., 0.};
  VisView wt2DViewZoomed=SceneTool.getView(
      tmpSim, wtTagName + "." + "WT 2D Auto View Zoomed",
      bodyCsys, wt2DfocalPt, wt2DCameraPos,
      yVector, true);
  wt2DViewZoomed.getParallelScale().setValue(0.5 * chordLength);
  Units units_0 = 
    ((Units) tmpSim.getUnitsManager().getObject("m"));
  wt2DViewZoomed.getFocalPointCoordinate().setCoordinate(
      units_0, units_0, units_0, new DoubleVector(
          new double[] {wt2DCameraPos[0], 0.0, 0.0}
          )
      );
  wt2DViewZoomed.getViewUpCoordinate().setCoordinate(
      units_0, units_0, units_0, 
      new DoubleVector(new double[] {0.0, -1.0, 0.0})
      );
  wt2DViewZoomed.getPositionCoordinate().setCoordinate(
      units_0, units_0, units_0, 
      new DoubleVector(
          new double[] {wt2DCameraPos[0], 0.0, wt2DCameraPos[2]}
          )
      );
  tmpCFDScene.getCurrentView().setView(wt2DViewZoomed);
  tmpCFDScene.printAndWait(
      outputDir + "BLP_FlowScene_"+fileName+".png", 1,
      xyRes[0], xyRes[1], true, false
      );
}
private void postProcessBoundaryLayerProbeResolution(
    Simulation tmpSim, String csvFileDir,String csvFileName){
  SystemTool.touchDirectoryChain(tmpSim, csvFileDir);

  // Method to output data to the CFD Master Results File
  Writer writer = null;

  int numBLProbes = allBoundaryLayerProbes.size();

  if(numBLProbes>0){
    try {
      writer = new BufferedWriter(new OutputStreamWriter(
          new FileOutputStream(csvFileDir+csvFileName,true), "utf-8"));
      String lineStr = "";
      for(int i=0; i < numBLProbes; i++){
        BoundaryLayerProbe tmpProbe = allBoundaryLayerProbes.get(i);
        if(i == 0){
          lineStr = lineStr + "BLP Name, First y+ value, Highest Prism Y+,"
              +" NCells in SubLayer, SubLayer Status, NCells in BufferLayer,"
              +" BufferLayer Status, NCells in LogLayer, LogLayer Status \n";
        }
        lineStr=lineStr+tmpProbe.prismLayerResolutionToCSV(tmpSim)+"\n";
        writer.write(lineStr);
        lineStr="";
      }
      ((BufferedWriter) writer).newLine();
    } catch (IOException ex) {
      // report
    } finally {
      try {writer.close();} catch (Exception ex) {/*ignore*/}
    }
  }
}
public void addBoundaryLayerProbe(Simulation tmpSim, BoundaryLayerProbe thisProbe){
  if(!allBoundaryLayerProbes.contains(thisProbe)){
    allBoundaryLayerProbes.add(thisProbe);  
  }
}
public void postProcessAeroTools(Simulation tmpSim, String folderPath){
  // Alpha.
  NumberFormat formatsmall = new DecimalFormat( "0.##" );
  String alphaStr= "_A";
  if (this.alphaAngle < (0.0 - SMALL_EPS)){
    alphaStr += "-";
  }
  if (Math.abs(this.alphaAngle) < (10.0 - SMALL_EPS)){
    alphaStr += "0";
  }
  alphaStr = alphaStr + formatsmall.format(Math.abs(this.alphaAngle));
  
  String userPath = folderPath + "USER";
  String cfdPath = folderPath + "CFD";

  SystemTool.touchDirectoryChain(tmpSim,userPath);
  SystemTool.touchDirectoryChain(tmpSim,cfdPath);
  
  //USER IMAGE PATH
  String userPlots  = userPath + File.separator + "PLOTS" + File.separator;
  String userScenes = userPath + File.separator + "SCENES" + File.separator;
  String userCSV    = userPath + File.separator + "CSV" + File.separator;

  SystemTool.touchDirectoryChain(tmpSim,userPlots);
  SystemTool.touchDirectoryChain(tmpSim,userCSV);
  SystemTool.touchDirectoryChain(tmpSim,userScenes);

  //CFD IMAGE PATH
  String cfdPlots   = cfdPath + File.separator + "PLOTS"  + File.separator;
  String cfdScenes  = cfdPath + File.separator + "SCENES" + File.separator;
  String cfdCSV     = cfdPath + File.separator + "CSV"    + File.separator;

  SystemTool.touchDirectoryChain(tmpSim, cfdPlots);
  SystemTool.touchDirectoryChain(tmpSim, cfdCSV);
  SystemTool.touchDirectoryChain(tmpSim, cfdScenes);

  // Prism Layer Resolution Info to Single CSV sheet
  String blpResolutionFile = "BoundaryLayerProbe_Resolution" + alphaStr + ".csv";
  postProcessBoundaryLayerProbeResolution(tmpSim, cfdCSV, blpResolutionFile);

  int lastIndx = folderPath.lastIndexOf(File.separator);
  String probeFileName = folderPath.substring(0, lastIndx);
  int firstIndx = probeFileName.lastIndexOf(File.separator);
  probeFileName = folderPath.substring(firstIndx + 1, lastIndx);

  outputBLPMeshStatusScene(
      tmpSim, folderPath + ".." + File.separator, probeFileName + alphaStr
      );

  outputBLPFlowStatusScene(
      tmpSim,folderPath+".."+File.separator, probeFileName + alphaStr
      );
}

//PHYSICS
public double getFSTVR(){
  return fsTVR;
}
public double getFSTi(){
  return fsTi;
}
public void setFSTVR(double newVal){
  fsTVR = newVal;
}
public void setFSTi(double newVal){
  fsTi = newVal;
}
public void setInitialConditions(Simulation tmpSim){
  CFD_Physics.set_VelocityIC(wtPhysics.getContinuum(), inletCsys, refVel,
      chordLength * .01, chordLength * 0.10);
  double refPress = wtPhysics.getContinuum().getReferenceValues()
          .get(ReferencePressure.class).getInternalValue();
  CFD_Physics.set_PressureIC(
      wtPhysics.getContinuum(), refVel, refPress, chordLength * 0.10
      );
  CFD_Physics.set_TemperatureIC(wtPhysics.getContinuum(),refPress, refRho);

  boolean isCoupled = CFD_Physics.isCoupledSolverUsed(wtPhysics.getContinuum());

  //Segregated Solver Special initialization techniques
  if(!isCoupled){
      SolverDriver.useLinearRampOnSegPressureSolver(tmpSim, 100, 200);
  }else{
    double fsTkeValue = 1.0E-6 * refVel * refVel;
    double fsOmegaValue = 5.0 * refVel / chordLength;
    // Initial Condition Modifications
    CFD_Physics.setTurbulenceToKandOmega(
        wtPhysics.getContinuum(), fsTkeValue, fsOmegaValue);
  }
}
public void instantiateExistingPhysics(Simulation tmpSim, String physicsName){
  if(wtPhysics==null){
    wtPhysics=new CFD_Physics(tmpSim,physicsName);
  }
}

//rotate the entire assembly
public void modifyOversetAngleOfAttack(Simulation tmpSim,double newAngle){
  //Rotate any overset domain region mesh about the body Csys
  tmpSim.println("I think I have: " + allOversetDomains.size() +" OS Domains");
  for(OversetDomain tmpDomain : allOversetDomains){
    tmpSim.println("WT MSG: Rotating " + tmpDomain.getName()
        + " to " + newAngle + " degrees.");
    double oldOSAngleInDeg = tmpDomain.setDomainAngleFromZero(newAngle, zVector);
    if(tmpDomain.getName().startsWith(globalNames.getOversetPreFix() + "1")){
      oldTurnTableAngleInDeg = oldOSAngleInDeg;
    }
  }
}
public static String getOversetFoilName(PartSurface tmpSurf){
  String tmpPartName=tmpSurf.getPresentationName();
  char sepChar1='_';
  char sepChar2=' ';
  int firstIndx=0;
  int counter=0;
  for (int i = 0; i<tmpPartName.length(); i++){
    if(tmpPartName.charAt(i) == sepChar1 ||
        tmpPartName.charAt(i) == sepChar2){
      counter+=1;
      if(counter==1){
          firstIndx=i;
      }
    }
  }

  if(tmpPartName.endsWith("TE")){
    //removes _TE
    return tmpPartName.substring(firstIndx+1, tmpPartName.length() - 3);
  }else{
    // to the end
    return tmpPartName.substring(firstIndx+1);
  }
}
public void modifyMeshAngleOfAttack(Simulation tmpSim, double newAngle){
  //If there is a turn table angle of the assembly that is being rotated
  // we also need to rotate BodyCsys at the same angle
  boolean thereIsAMeshTurnTable=true;
  try{
      String assemblyOpName = "Rotate Assembly";
      MeshOpTool.setRotationAngle(tmpSim, assemblyOpName, newAngle);
      MeshOpTool.setRotationAxis(tmpSim, assemblyOpName,
          new double[] {0.0, 0.0, -1.0}
          );
  }catch(NeoException e){
      tmpSim.println("WT MSG: No geometric mesh angle of attack found.");
      thereIsAMeshTurnTable = false;
  }

  //Get CAD-zero value for basis, rotate about that
  CartesianCoordinateSystem cadZeroBodyCsys =
      SimTool.getLabBasedCoordinate(
          tmpSim,"CAD Zero "+bodyCsys.getPresentationName()
          );
  bodyCsys.setBasis0(cadZeroBodyCsys.getBasis0());
  bodyCsys.setBasis1(cadZeroBodyCsys.getBasis1());

  //opposite direction of body, same y vector
  double oldxValue = bodyCsys.getBasis0().toDoubleArray()[0];
  double newXMultiplier = 1.0;
  if(oldxValue<0.0) newXMultiplier = -1.0;
  double oldyValue = bodyCsys.getBasis1().toDoubleArray()[1];
  double newYMultiplier = 1.0;
  if(oldyValue<0.0) newYMultiplier = -1.0;
  bodyCsys.setBasis0(new DoubleVector(new double[] {newXMultiplier,0.0,0.0}));
  bodyCsys.setBasis1(new DoubleVector(new double[] {0.0,newYMultiplier,0.0}));

  //set wind opposite of body, opposite y vector is ok because it's Aero
  // only in the case where this is a freestream case, otherwise, the inlet
  // should be set opposite of the body rotation vector.
  inletCsys.setBasis0(new DoubleVector(new double[] {-1.0,0.0,0.0}));
  inletCsys.setBasis1(new DoubleVector(new double[] {0.0,-1.0,0.0}));



  // In non-freestream cases, we rotate the body coordinate under 
  // the following conditions:
  //    1) There is a Mesh Assembly rotation operation
  //    2) Overset mesh rotates along with body
  // In freestream cases, we change the inlet velocity vector only as
  // this is more computationally efficient.
  if(!isFS){
    tmpSim.println("This is not a free stream case, rotating Body_CSys");
    tmpSim.println("Body Csys Name: "+bodyCsys.getPresentationName());
    SimTool.rotateCoordinateSystem(
        tmpSim, -newAngle, zVector, bodyCsys, bodyCsys
        );
  }

  //In non-freestream cases, we always counter rotate the Velocity Inlet from
  // the **CAD ZERO** body Csys using standard Euler rotations to keep the
  // direction of the wind aligned with the lab X-axis.
  // This was true only if the inletCsys was a nested coordinate system. It is
  // no longer nested in the Body, but is instead nested at the root level. For
  // a non-FS case, we do not rotate the coordinate system.
  if(isFS){
    SimTool.rotateCoordinateSystem(tmpSim,newAngle,zVector,bodyCsys,inletCsys);
  }
}

// BOUNDARY LAYER PROBES
public void setAllBLProbes(
    Simulation tmpSim, ArrayList<BoundaryLayerProbe> newProbeList){
  allBoundaryLayerProbes = newProbeList;
}
public void updateBLProbes(Simulation tmpSim, double refRho, double refMu){
  for(BoundaryLayerProbe tmpProbe : allBoundaryLayerProbes){
    tmpProbe.updateBLPValues(tmpSim, refRho, refMu/refRho);
  }
}
public void relaxBLPProbes(Simulation tmpSim){
  for(BoundaryLayerProbe tmpProbe : allBoundaryLayerProbes){
    tmpProbe.relaxToMeshSurface(tmpSim);
  }
}
public void rotateBoundaryLayerProbeCADParts(
    Simulation tmpSim, double newAngleInDeg){
  // because all low speed wind tunnel cases are overset and all high speed are
  // fixed with a remesh, we only rotate in the presence of overset domains.
  if(allOversetDomains.size() > 0){
    ArrayList<GeometryPart> allBLPCADParts = new ArrayList();
    for(BoundaryLayerProbe tmpProbe : allBoundaryLayerProbes){
      allBLPCADParts.add(tmpProbe.getGeometryPart());
    }

    // Because there is no history in the simulation file, we use the primary overset
    // region's old angle before rotation to counter-rotate the CAD parts
    if(Math.abs(oldTurnTableAngleInDeg) > SMALL_EPS){
      Units units_0 = SimTool.getUnitsVec(tmpSim);
      tmpSim.get(SimulationPartManager.class)
        .rotateParts(new NeoObjectVector(allBLPCADParts.toArray()),
          new DoubleVector(new double[] {0.0, 0.0, 1.0}),
          new NeoObjectVector(new Object[] {units_0, units_0, units_0}),
          oldTurnTableAngleInDeg*Math.PI/180.0, this.bodyCsys);
    }

    //only rotate the Parts if |angle| > 0 degrees, otherwise just accept the reset
    // because X points forward and y downward, a negative rotation is applied
    if(Math.abs(newAngleInDeg) > SMALL_EPS){
      Units units_0 = SimTool.getUnitsVec(tmpSim);
      tmpSim.get(SimulationPartManager.class)
        .rotateParts(new NeoObjectVector(allBLPCADParts.toArray()),
          new DoubleVector(new double[] {0.0, 0.0, 1.0}),
          new NeoObjectVector(new Object[] {units_0, units_0, units_0}),
          -newAngleInDeg*Math.PI/180.0, this.bodyCsys);
    }
  }

}

// REPORTS
public void createUnsteadyObjectReporting(Simulation tmpSim){
  int nPlotSamples=(int) 1.e4;
  int tsMonFreq = 1;
  int tsMonStart= 1;
  //AIRFOILS
  if(allAirfoilObjects.size()>0){
    ArrayList<Report> airfoilTotalReportsList =
        setupTotalAirfoilReports(tmpSim, "Aero", inletCsys);
    for(Report tmpReport : airfoilTotalReportsList){
      Monitor tmpMon = MonitorTool.reportUnsteadyMonitor(
          tmpSim, tmpReport, nPlotSamples, tsMonFreq, tsMonStart
          );
      TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
    }
  }
  for(AerodynamicSurface tmpFoil:allAirfoilObjects){
    tmpFoil.makeUnsteadyMonitors(tmpSim,nPlotSamples,tsMonFreq,tsMonStart);
  }
}
public AerodynamicSurface getAirfoilElement(int elementNumber){
  AerodynamicSurface correctElement = null;
  String tmpFoilPrefixStr=airfoilStrList.get(elementNumber);

  //shopping a prefix
  for(AerodynamicSurface tmp : allAirfoilObjects){
    for(PartSurface tmpPartSurface : tmp.getPartSurfaces()){
      String tmpPartSurfaceName=tmpPartSurface.getPresentationName();
      if(tmpPartSurfaceName.contains("."+tmpFoilPrefixStr) ||
          tmpPartSurfaceName.startsWith(tmpFoilPrefixStr)){
        correctElement = tmp;
        break;
      }
    }
    if(correctElement != null){
     break;
    }
  }
  return correctElement;
}
public TransformPartsOperation createAirfoilPartMeshOpCADAngle(
    Simulation tmpSim, GeometryPart tmpFoil, CartesianCoordinateSystem tmpCsys){ 
  /* createAirfoilPartMeshOpCADAngle allows CAD based airfoils to rotate becore
     a MeshOperation to subtract from the main tunnel domain.
  */
  String foilName = tmpFoil.getPresentationName();
  return MeshOpTool.rotationTransform(
      tmpSim, "Rotate " + foilName, tmpFoil, tmpCsys, wtAxisRot
      );
}

// POST PROCESSING
public String makeSheetsCSVData(Simulation tmpSim, String studyName,
    String cfdVer, String javaVer){
  String retString;

  //CFD Simulation Stats
  tmpSim.println("WT MSG: Post processing CFD header...");
  retString=studyName+","+cfdVer+","+javaVer;

  tmpSim.println("WT MSG: Post processing simulation state...");
  //Number of Iterations
  int currentItLvl=SolverDriver.getCurrentIterationLevel(tmpSim);
  retString = retString+","+currentItLvl;

  //Number of Steps
  int currentStepLvl=SolverDriver.getCurrentStepLevel(tmpSim);
  retString = retString+","+currentStepLvl;

  //Time step (if exists)
  if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
      retString=retString+","+SolverDriver.getTimeStep(tmpSim);
  }else{
      retString=retString+","+"N/A";
  }
  //Solution Time (if exists)
  if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
      retString=retString+","+SolverDriver.getCurrentTime(tmpSim);
  }else{
      retString=retString+","+"N/A";
  }


  //Stopping Criteria Satisfied
  retString=retString+","+"N/A";

  //Physics
  tmpSim.println("WT MSG: Post processing Physics...");
  //2d or 3d
  if(CFD_Physics.is2D(wtPhysics.getContinuum())){
      retString=retString+","+"2D";
  }else{
      retString=retString+","+"3D";
  }
  //fs or wt
  if(isFS){
      retString=retString+","+"Free Stream";
  }else{
      retString=retString+","+"Tunnel";
  }
  //steady or unsteady
  if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
      retString=retString+","+"Unsteady";
  }else{
      retString=retString+","+"Steady";
  }
  //turbulence model
  retString=retString+","+CFD_Physics.getTurbulenceModelName(wtPhysics.getContinuum());

  //  get primary boundary condition information
  //  --> dictates freestream or velocity inlet
  //  if inlet exists, it is a velocity inlet type of case
  //  else it is a freestream case
  // 
  tmpSim.println("Getting boundary from: " + this.getName());
  Boundary mainBndy = this.getAnInletBoundary();
  double bcVel;
  double bcTemp;
  double bcTVR;
  double bcTi;
  double bcRho;
  double bcMa;
  double bcRe;
  double bcMu;
  if(mainBndy.getBoundaryType() instanceof FreeStreamBoundary){
      //Mach at inlet
      MachNumberProfile machNP = 
        mainBndy.getValues().get(MachNumberProfile.class);
      bcMa= machNP.getMethod(ConstantScalarProfileMethod.class)
              .getQuantity().getRawValue();
      // Temperature
      StaticTemperatureProfile sTP = 
        mainBndy.getValues().get(StaticTemperatureProfile.class);
      bcTemp=sTP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();

      // K
      TurbulentKineticEnergyProfile turbulentKineticEnergyProfile_0 = 
        mainBndy.getValues().get(TurbulentKineticEnergyProfile.class);
      bcTi=turbulentKineticEnergyProfile_0.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();
      // Omega
      SpecificDissipationRateProfile specificDissipationRateProfile_0 = 
        mainBndy.getValues().get(SpecificDissipationRateProfile.class);
      bcTVR=specificDissipationRateProfile_0.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

      // Calculate Re, Mach
      double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
      double molAir=.0289645; // [kg/mol] molar gas constant of air
      double refPress=CFD_Physics.getReferencePressure(wtPhysics.getContinuum()); // [Pa] reference pressure for air

      // Get dynamic viscosity from WT Physics
      bcMu=CFD_Physics.getMu(wtPhysics.getContinuum());
      bcRho=refPress/bcTemp/(airR/molAir);
      bcVel = refVel;
      bcRe  =chordLength*bcVel*bcRho/bcMu;

  }else{
      // Velocity
      VelocityMagnitudeProfile vMP = 
          mainBndy.getValues().get(VelocityMagnitudeProfile.class);
      bcVel=vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
      // Temperature
      StaticTemperatureProfile sTP = 
        mainBndy.getValues().get(StaticTemperatureProfile.class);
      bcTemp=sTP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();

      // TuI
      TurbulenceIntensityProfile tIP = 
          mainBndy.getValues().get(TurbulenceIntensityProfile.class);
      bcTi=tIP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();

      // TVR
      TurbulentViscosityRatioProfile tVRP = 
          mainBndy.getValues().get(TurbulentViscosityRatioProfile.class);
      bcTVR=tVRP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();

      // Get dynamic viscosity from WT Physics
      bcMu=CFD_Physics.getMu(wtPhysics.getContinuum());

      // Calculate Re, Mach
      double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
      double molAir=.0289645; // [kg/mol] molar gas constant of air
      double refPress=CFD_Physics.getReferencePressure(wtPhysics.getContinuum()); // [Pa] reference pressure for air

      //Temp=refPress/refRho/(airR/molAir)
      bcRho= refPress/bcTemp/(airR/molAir);
      bcRe = chordLength*bcVel*bcRho/bcMu;
      bcMa = bcVel/Math.sqrt(1.4*(airR/molAir)*bcTemp);
  }

  retString=retString+","+bcRe+","+bcMa+","+chordLength
          +","+bcVel+","+bcTemp+","+bcRho+","+bcMu+","+bcTVR+","+bcTi;

  //Total Airfoil Performance (if it exists);
  double primaryLengthScale=0.0;
  if(allAirfoilObjects.size()>0){
      tmpSim.println("WT CSV: Post processing total airfoil assembly...");
      //chord length
      primaryLengthScale=chordLength;
      retString=retString+","+primaryLengthScale;
      //reference Area
      String referenceArea = Double.toString(refArea);
      retString=retString+","+referenceArea;
      //reference Radius not needed because it is always chord length
      //
      // turn table angle
      retString = retString + "," + alphaAngle;

        tmpSim.println("WT CSV: Post processing " 
      + this.getName()+ " Airfoil Total Performance CSV Values...");
      retString=retString+","+getTotalAirfoilReportCSVData(tmpSim, statSamples);
  }

  //Output global wind tunnel and mesh settings
  tmpSim.println("WT CSV: Post processing " 
      + this.getName()+ " Mesh Settings...");
  retString = retString + "," + getMainTunnelMeshString(tmpSim);


  //SPECIFIC OBJECT POST-PROCESSING INFORMATION
 // Individual Airfoil Element Information
  tmpSim.println("WT CSV: Post processing " 
      + this.getName()+ " Airfoil CSV Values...");
  for(AerodynamicSurface tmpFoil : allAirfoilObjects){
      retString = retString + "," + tmpFoil.csvData(tmpSim);
  }

  //fin
  return retString;
}
private String getMainTunnelMeshString(Simulation tmpSim){
  String retString="";

  // Publish name to help identify airfoils used
  retString += this.domainName;
  
  // get the volume mesh model name
  AutoMeshOperation wtSM = domainSurfMeshOp;
  AutoMeshOperation wtVM = domainVolumeMeshOp;
  try{
      wtVM.getMeshers().getObject("Trimmed Cell Mesher");
      retString = retString + "," + "Trim";
  }catch(NeoException e){
      retString = retString + "," + "Poly";
  }

  //Base size
  retString=retString + "," + wtVM
      .getDefaultValues().get(BaseSize.class).getRawValue();

  //Max size percentage
  PartsTargetSurfaceSize targetSurfaceSize = 
      wtSM.getDefaultValues().get(PartsTargetSurfaceSize.class);

  retString=retString+","+targetSurfaceSize.getRelativeSizeValue();

  //Min size percentage
  PartsMinimumSurfaceSize partsMinSurfSize=wtSM.getDefaultValues().get(PartsMinimumSurfaceSize.class);
  retString=retString+","+partsMinSurfSize.getRelativeSizeValue();

  //N Pts on Circle
  SurfaceCurvature surfaceCurvature = 
      wtSM.getDefaultValues().get(SurfaceCurvature.class);
  retString=retString+","+surfaceCurvature.getNumPointsAroundCircle();

  //Number of Prism Layers
  NumPrismLayers numPrismLayers = 
      wtVM.getDefaultValues().get(NumPrismLayers.class);
  retString=retString+","+numPrismLayers.getNumLayers();

  //Prism Thickness
  PrismThickness prismThickness = 
      wtVM.getDefaultValues().get(PrismThickness.class);
  retString=retString+","+prismThickness.getAbsoluteSizeValue().getInternalValue();

  //Prism First Cell Thickness
  retString=retString+","+wtVM.getDefaultValues().get(PrismWallThickness.class).getInternalValue();

  //List existing volume controls
  String vcString="";
  int vcCounter=0;
  for(CustomMeshControl tmpCntrl:wtVM.getCustomMeshControls().getObjects()){
      if(tmpCntrl instanceof VolumeCustomMeshControl){
          if(vcCounter==0){
              vcString=vcString+tmpCntrl.getPresentationName();
          }else{
              vcString=vcString+" & "+tmpCntrl.getPresentationName();
          }
          vcCounter+=1;
      }
  }
  if(vcString.equals("")){
      vcString="None";
  }
  retString=retString+","+vcString;

  return retString;
}
public void setWindTunnelView(Simulation tmpSim){
  if(is2D){
    wt2DCameraPos[0] = -0.25 * chordLength;
    wt2DCameraPos[2] =  1.0  * chordLength * viewAngleDistanceOffset;
  }else{
    tmpSim.println("WT MSG: Wind Tunnel is prescribed as 3D.");
  }
}

public void init2DPostProcessing(Simulation tmpSim){
  int nPlotSamples=(int) 1.e5;
  int itMonFreq = 1;
  int itMonStart= 1;
  //
  // Universal Field Functions
  //
  // pressure coefficient
  double refPress = wtPhysics.getContinuum().getReferenceValues()
          .get(ReferencePressure.class).getInternalValue(); //Pa
  SimTool.setPressureCoefficientFF(tmpSim, refRho, 0.0, refVel);
  // skin friction coefficient
  SimTool.setSkinFrictionCoefficientFF(tmpSim, refRho, refVel);

  // DERIVED PART SETUP
  ArrayList<Region> allWTRegs = new ArrayList();
  allWTRegs.add(this.getRegion());
  for(OversetDomain tmpDomain : allOversetDomains){
    allWTRegs.add(tmpDomain.getRegion());
  }

  // Derived Parts
  tmpSim.println("WT 2D MSG: Generating Derived Parts.");

  // mesh metrics
  tmpSim.println("WT 2D MSG: Setting Up Mesh Metrics");
  setupMeshMetrics(tmpSim, allWTRegs);

  //
  // REPORTS & MONITORS
  // 
  tmpSim.println("WT 2D MSG: Generating Reports & Monitors for Objects.");

  // Airfoil Specific
  if(allAirfoilObjects.size() > 0){
    tmpSim.println("WT 2D MSG: Total Airfoil Reports");
    // Total airfoil Reports
    ArrayList<Report> airfoilTotalReportsList =
        setupTotalAirfoilReports(tmpSim,
            this.getPrimaryDomainTag().getPresentationName() + "."+ "Aero",
            inletCsys
            );
    for(Report tmpReport : airfoilTotalReportsList){
      Monitor tmpMon = MonitorTool.reportIterationMonitor(
          tmpSim, tmpReport, nPlotSamples, itMonFreq, itMonStart);
      TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
    }
    // Center of pressure field function setup
    setup2DAirfoilTotalCenterOfPressure(tmpSim, airfoilTotalReportsList);
  }

    // For each airfoil element activate Reports.
    tmpSim.println("WT 2D MSG: Element Reports");
    for(AerodynamicSurface tmpFoil:allAirfoilObjects){
      tmpFoil.makeReports(tmpSim);
      tmpFoil.TagReports(
          tmpSim, Collections.singleton(this.getPrimaryDomainTag()));
      tmpFoil.makeIterationMonitors(
          tmpSim, nPlotSamples, itMonFreq, itMonStart);
      // Tag the foil monitors
      for(Monitor tmpMon : tmpFoil.getAllMonitors()){
        TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
      }
    }

    // Clean monitor output
    tmpSim.println("WT 2D MSG: Cleaning monitor output.");
    ArrayList<Monitor> monitorOutput = new ArrayList();
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Continuity"));
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("X-momentum"));
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Y-momentum"));
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Energy"));
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Tke"));
    monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Sdr"));
    tmpSim.getMonitorManager().setPrintedMonitors(monitorOutput);
    monitorOutput.clear();

    //
    // PLOTS
    //
    // Adjust residual plots
    tmpSim.println("WT 2D MSG: Adjusting residual plots.");
    PlotTool.adjustResidualPlot(tmpSim, is2D);

    // Airfoil specific
    // Skin Friction coefficient plot
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total Cf");
    setup2DAirfoilTotalCfPlot(tmpSim);

    // Pressure coefficient plot
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total Cp");
    setup2DAirfoilTotalCpPlot(tmpSim);

    // Total Lift
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total CL");
    tmpSim.println("making");
    MonitorPlot airfoilClPlot = PlotTool.getMonitorPlot(
        tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CL", "Airfoil Total CL");
    
    
    tmpSim.println("tagging");
    
    TagTool.addTags(tmpSim, airfoilClPlot, 
        Collections.singleton(this.getPrimaryDomainTag()));
    String tmpName = this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CL - It";

    tmpSim.println("data");
    MonitorDataSet tmpMonData =
        PlotTool.addDataSet(tmpSim, airfoilClPlot,tmpName, "CL");
    PlotTool.setDataLines(tmpMonData, "Solid", 2, PlotTool.getColor("green"));
    airfoilClPlot.close();

    // Total Drag
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total Cd");
    MonitorPlot airfoilCdPlot = PlotTool.getMonitorPlot(
        tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CD","Airfoil Total CD");
    TagTool.addTags(tmpSim, airfoilCdPlot, 
        Collections.singleton(this.getPrimaryDomainTag()));
    tmpName=this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CD - It";
    tmpMonData=PlotTool.addDataSet(tmpSim, airfoilCdPlot,tmpName,"CD");
    PlotTool.setDataLines(tmpMonData, "Solid", 2, PlotTool.getColor("red"));
    airfoilCdPlot.close();
    // Total Moment
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Total CmZ");
    MonitorPlot airfoilCmZPlot = PlotTool.getMonitorPlot(
        tmpSim, this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CmZ","Airfoil Total CmZ");
    TagTool.addTags(tmpSim, airfoilCmZPlot, 
        Collections.singleton(this.getPrimaryDomainTag()));
    
    tmpName = this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total - CmZ - It";
    tmpMonData=PlotTool.addDataSet(tmpSim, airfoilCmZPlot,tmpName,"CmZ");
    PlotTool.setDataLines(tmpMonData, "Solid", 2, PlotTool.getColor("green"));

    airfoilCmZPlot.close();
    // CFD Information
    tmpSim.println("WT 2D MSG: Setting up 2D Airfoil Wall y+");
    setup2DAirfoilTotalWallYPlus(tmpSim);

    //
    // SCENES
    //
    Scene tmpScene;
    Scene tmpCFDScene;
    ScalarDisplayer scalarDisp;
    PartDisplayer partDisp;
    VectorMagnitudeFieldFunction vMFF;
    PrimitiveFieldFunction pFF;
    String wtTagName = this.getPrimaryDomainTag().getPresentationName();

    // for 2D, we want to display outline on symmetry and wall boundaries
    ArrayList<Domain> allDomainObjects = new ArrayList();
    allDomainObjects.add(this);
    allDomainObjects.addAll(allOversetDomains);
    ArrayList<Boundary> outlineBoundaries = new ArrayList();
    for(Domain tmpDomain : allDomainObjects){
      for(Boundary tmpBndy : tmpDomain.getRegion()
          .getBoundaryManager().getBoundaries()){
        if(tmpBndy.getBoundaryType() instanceof SymmetryBoundary){
          outlineBoundaries.add(tmpBndy);
        }
      }
    }
    // for 2D, we want to display the fields from the all region boundaries
    ArrayList<Region> regionObjects = new ArrayList();
    for(Domain tmpDomain : allDomainObjects){
      regionObjects.add(tmpDomain.getRegion());
    }

    // for 2D, we want the camera to be centered on the Body Csys origin
    //   but remain in the Laboratory coordinate system
    double[] wt2DfocalPt = {wt2DCameraPos[0], 0.0, 0.0};
    VisView wt2DView =
        SceneTool.getView(
            tmpSim, "WT 2D Auto View", bodyCsys,
            wt2DfocalPt, wt2DCameraPos, yVector, true
            );
    wt2DView.getParallelScale().setValue(
        1.0 * this.getChordLengthFromParameters(tmpSim));
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    wt2DView.getFocalPointCoordinate().setCoordinate(units_0, units_0, units_0,
        new DoubleVector(new double[] {wt2DCameraPos[0], 0.0, 0.0}));
    wt2DView.getViewUpCoordinate().setCoordinate(units_0, units_0, units_0,
        new DoubleVector(new double[] {0.0, -1.0, 0.0}));
    wt2DView.getPositionCoordinate().setCoordinate(units_0, units_0, units_0,
        new DoubleVector(
            new double[] {wt2DCameraPos[0], 0.0, wt2DCameraPos[2]}
            )
        );

    // VELOCITY MAGNTIDUE SCENE
    tmpSim.println("WT 2D MSG: Setting Velocity Magnitude Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Velocity Magnitude");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "v_mag", regionObjects);
    pFF = (
        (PrimitiveFieldFunction) tmpSim.getFieldFunctionManager()
        .getFunction("Velocity")
        );
    vMFF= ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
    SceneTool.setScalarDisplayerField(
        scalarDisp, vMFF.getInternalName(), "Off", "Off", 0.0,
        refVel * 1.5, proxyRep
        );
    //better colormap
    Legend legend_0 = scalarDisp.getLegend();
    PredefinedLookupTable predefinedLookupTable_0 = 
      ((PredefinedLookupTable) tmpSim.get(LookupTableManager.class)
          .getObject("blue-red balanced"));
    legend_0.setLookupTable(predefinedLookupTable_0);
    legend_0.setNumberOfLabels(5);
    partDisp = SceneTool.getPartDisplayer(
        tmpScene, "objs",outlineBoundaries, proxyRep
        );
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getCurrentView()
        .setView(wt2DView);
    tmpScene.getSceneUpdate()
        .setAnimationFilenameBase("auto_vel_mag");
    tmpScene.getSceneUpdate()
        .getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate()
        .getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate()
        .getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Velocity Magnitude")){
      tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName + "." + "CFD Velocity Magnitude");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp =
          SceneTool.getScalarDisplayer(tmpCFDScene, "v_mag",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_vel_mag");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    //HIGH MACH NUMBER
    if(refMa > highMachNumber){
      tmpSim.println("WT 2D MSG: Setting Mach Scene");
      tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Mach");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "mach",regionObjects);
      pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("MachNumber"));
      SceneTool.setScalarDisplayerField(scalarDisp,pFF.getInternalName(),"Off","Off", highMachNumber, 1.2, proxyRep);
      partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
      SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
      //better colormap
      legend_0 = scalarDisp.getLegend();
      predefinedLookupTable_0 = 
        ((PredefinedLookupTable) tmpSim.get(LookupTableManager.class).getObject("blue-red balanced"));
      legend_0.setLookupTable(predefinedLookupTable_0);
      legend_0.setNumberOfLabels(5);
      //annotations
      for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
      }
      for(Annotation tmpAnn:wtAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
      }
      SceneTool.removeDefaultLogo(tmpScene);
      tmpScene.getCurrentView().setView(wt2DView);
      tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_mach");
      tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
      tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
      tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

      if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Mach")){
        tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName +"." + "CFD Mach");
        allWTScenes.add(tmpScene);
        tmpCFDScene.copyProperties(tmpScene);
        scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "mach",regionObjects);
        //turn on the mesh and disable smoothing
        scalarDisp.setDisplayMesh(1);
        scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
        tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_mach");
        tmpScene.getCurrentView().setView(wt2DView);
      }
    }

    //Schlieren Image
    if(refMa > highMachNumber){
      tmpSim.println("WT 2D MSG: Setting Schlieren Scene");
      tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Schlieren");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "schlieren",regionObjects);
      UserFieldFunction uFF = null;
      try{
        tmpSim.println("up");
        uFF = ((UserFieldFunction) tmpSim.getFieldFunctionManager().getObject("Schlieren"));
        tmpSim.println("here");
      }catch(NeoException e){
        tmpSim.println("down");
        uFF = (UserFieldFunction) tmpSim.getFieldFunctionManager().createFieldFunction();
        uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
        uFF.setPresentationName("Schlieren");
        uFF.setFunctionName("Schlieren");
        uFF.setDefinition("mag(grad($Density))");
        tmpSim.println("here");
      }
      SceneTool.setScalarDisplayerField(scalarDisp,uFF.getInternalName(),"Off","Off",0.0,100.0,proxyRep);
      partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
      SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
      //better colormap
      legend_0 = scalarDisp.getLegend();
      BlackWhiteLookupTable bwLookupTable_0 = 
        ((BlackWhiteLookupTable) tmpSim.get(LookupTableManager.class).getObject("grayscale"));
      legend_0.setLookupTable(bwLookupTable_0);
      legend_0.setReverse(true);
      //annotations
      for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
      }
      for(Annotation tmpAnn:wtAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
      }
      SceneTool.removeDefaultLogo(tmpScene);
      tmpScene.getCurrentView().setView(wt2DView);
      tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_schlieren");
      tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
      tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
      tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

      if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Schlieren")){
        tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName +"." + "CFD Schlieren");
        TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
        allWTScenes.add(tmpScene);
        tmpCFDScene.copyProperties(tmpScene);
        scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "schlieren",regionObjects);
        //turn on the mesh and disable smoothing
        scalarDisp.setDisplayMesh(1);
        scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
        tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_schlieren");
        tmpScene.getCurrentView().setView(wt2DView);
      }
    }

    // VORTICITY MAGNTIDUE SCENE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting Vorticity Magnitude Scene");
    tmpScene   = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Vorticity Magnitude");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "omega_mag", regionObjects);
    VorticityVectorFunction vortVF = 
      ((VorticityVectorFunction) tmpSim.getFieldFunctionManager().getFunction("VorticityVector"));
    vMFF= ((VectorMagnitudeFieldFunction) vortVF.getMagnitudeFunction());
    SceneTool.setScalarDisplayerField(scalarDisp,vMFF.getInternalName(),"Off","Off",0.0,refVel*refVel,proxyRep);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_omega_mag");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Vorticity Magnitude")){
      tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName + "." + "CFD Vorticity Magnitude");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "omega_mag",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_omega_mag");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    // VORTICITY_k SCENE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting Vorticity_k_Inlet Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO omega_k_inlet");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "omega_k_inlet", regionObjects);
    VectorComponentFieldFunction vectorCFF = 
      ((VectorComponentFieldFunction) vortVF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(2));
    SceneTool.setScalarDisplayerField(scalarDisp,vectorCFF.getInternalName(),"Off","Off",-refVel*refVel,refVel*refVel,proxyRep);
    //better colormap
    legend_0 = scalarDisp.getLegend();
    CoolWarmLookupTable coolwarmLookupTable_0 = 
      ((CoolWarmLookupTable) tmpSim.get(LookupTableManager.class).getObject("cool-warm"));
    legend_0.setLookupTable(coolwarmLookupTable_0);
    legend_0.setNumberOfLabels(5);
    partDisp = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_omega_k_inlet");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD omega_k_inlet")){
      tmpCFDScene = SceneTool.getScene(tmpSim,wtTagName + "." + "CFD omega_k_inlet");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "omega_k_inlet",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_omega_k_inlet");
      tmpScene.getCurrentView().setView(wt2DView);
    }


    // TKE MAGNTIDUE SCENE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting TKE Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName + "." + "AUTO TKE");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "tke", regionObjects);
    pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("TurbulentKineticEnergy"));
    SceneTool.setScalarDisplayerField(scalarDisp,pFF.getInternalName(),"Off","Off",0.0,10.0,proxyRep);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_tke");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD TKE Scene")){
      tmpCFDScene = SceneTool.getScene(tmpSim,wtTagName + "." + "CFD TKE");
      allWTScenes.add(tmpScene);
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "tke",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_tke");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    // TVR SCENE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting TVR Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO TVR");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "tvr", regionObjects);
    pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("TurbulentViscosityRatio"));
    SceneTool.setScalarDisplayerField(scalarDisp,pFF.getInternalName(),"Off","Off",0.0,1000.0,proxyRep);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_tvr");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD TVR")){
      tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName +"." + "CFD TVR");
      allWTScenes.add(tmpScene);
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "tvr",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_tvr");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    // PRESSURE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting Pressure Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Pressure");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "press", regionObjects);
    pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("Pressure"));
    double fsPressureHead = 0.5*refRho*refVel*refVel;
    SceneTool.setScalarDisplayerField(scalarDisp,pFF.getInternalName(),"Off","Off",-fsPressureHead,fsPressureHead,proxyRep);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    legend_0 = scalarDisp.getLegend();
    UserLookupTable userLookupTable_0 = 
      ((UserLookupTable) tmpSim.get(LookupTableManager.class).getObject(ColorMapTool.getBlueDkGnRedString(tmpSim)));
    legend_0.setLookupTable(userLookupTable_0);
    legend_0.setNumberOfLabels(5);

    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_press");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Pressure")){
      tmpCFDScene = SceneTool.getScene(tmpSim,wtTagName + "." + "CFD Pressure");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      allWTScenes.add(tmpScene);
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "press",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_press");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    // TEMPERATURE
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting Temperature Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "AUTO Temperature");
    TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
    allWTScenes.add(tmpScene);
    scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "temp", regionObjects);
    pFF = ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("Temperature"));
    SceneTool.setScalarDisplayerField(scalarDisp,pFF.getInternalName(),"Off","Off",refTemp,1.2*refTemp,proxyRep);
//      Units degreesC = ((Units) tmpSim.getUnitsManager().getObject("C"));
//      scalarDisp.getScalarDisplayQuantity().setUnits(degreesC);
    //better colormap
    legend_0 = scalarDisp.getLegend();
    predefinedLookupTable_0 = 
      ((PredefinedLookupTable) tmpSim.get(LookupTableManager.class).getObject("thermal"));
    legend_0.setLookupTable(predefinedLookupTable_0);
    legend_0.setNumberOfLabels(5);
    legend_0.setReverse(true);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_temp");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);

    if(!SceneTool.doesSceneExist(tmpSim, wtTagName +"." + "CFD Temperature")){
      tmpCFDScene = SceneTool.getScene(tmpSim, wtTagName +"." + "CFD Temperature");
      TagTool.addTag(tmpSim, tmpScene, this.getPrimaryDomainTag());
      tmpCFDScene.copyProperties(tmpScene);
      scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "temp",regionObjects);
      //turn on the mesh and disable smoothing
      scalarDisp.setDisplayMesh(1);
      scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
      tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_temp");
      tmpScene.getCurrentView().setView(wt2DView);
    }

    // CFD MESH
    // z=0 plane
    tmpSim.println("WT 2D MSG: Setting Mesh Scene");
    tmpScene = SceneTool.getScene(tmpSim, wtTagName +"." + "CFD Mesh");
    TagTool.addTag(tmpSim, tmpScene, primaryWTDomainTag);
    allWTScenes.add(tmpScene);
    partDisp   = SceneTool.getPartDisplayer(tmpScene, "mesh",regionObjects, proxyRep);
    SceneTool.setPartDisplayerField(partDisp, true, true, false, true, 0);

    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    for(Annotation tmpAnn:wtAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    tmpScene.getCurrentView().setView(wt2DView);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.getSceneUpdate().setAnimationFilenameBase("cfd_mesh");
    tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(xyRes[0]);
    tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(xyRes[1]);
  }
public void init3DPostProcessing(Simulation tmpSim){
    int nPlotSamples=(int) 1.e5;
    int itMonFreq = 1;
    int itMonStart= 1;

    // DERIVED PART SETUP
    ArrayList<Region> allWTRegs = new ArrayList();
    allWTRegs.add(this.getRegion());
    for(OversetDomain tmpDomain : allOversetDomains){
      allWTRegs.add(tmpDomain.getRegion());
    }
    
    // RELEVANT FIELD FUNCTIONS
    tmpSim.println("WT MSG: Making Pressure Coefficient Field Function");
    // pressure coefficient
    double refPress=wtPhysics.getContinuum().getReferenceValues()
            .get(ReferencePressure.class).getInternalValue(); //Pa
    SimTool.setPressureCoefficientFF(tmpSim,refRho,0.0,refVel);

    // skin friction coefficient
    tmpSim.println("WT MSG: Making Skin Friction Coefficient Field Function");
    SimTool.setSkinFrictionCoefficientFF(tmpSim,refRho,refVel);

    //
    //DERIVED PART SETUP
    //
    tmpSim.println("WT MSG: Generating 3d Derived Parts.");
    // mesh metrics
    tmpSim.println("WT MSG: Setting up Mesh Metrics");
    setupMeshMetrics(tmpSim, allWTRegs);

    // x-y plane for 3D
    tmpSim.println("WT MSG: Setting up XY-Plane for 3D Slice");
    double[] centerCutO = {0.,0.,chordLength*0.15};
    PlaneSection xyCutPlane = DerivedPartTool.singlePlane(
            tmpSim, allWTRegs, "Chord*0.15 Cut", bodyCsys, centerCutO, zVector
            );

    // default probe line for TVR/Ti
    tmpSim.println("WT MSG: Setting up Line Probe");
    LinePart lineProbe = DerivedPartTool.getLineProbe(
        tmpSim, allWTRegs, bodyCsys, chordLength, "Inlet Line", 100
        );

    // IsoSurface for reversed flow
    tmpSim.println("WT MSG: Setting up Reversed Flow IsoSurface");
    PrimitiveFieldFunction pFF = 
        ((PrimitiveFieldFunction) tmpSim
            .getFieldFunctionManager().getFunction("Velocity")
            );
    VectorComponentFieldFunction vCFF = 
        ((VectorComponentFieldFunction) pFF
            .getFunctionInCoordinateSystem(inletCsys).getComponentFunction(0));
    DerivedPartTool.getSingleIsoSurfPart(
        tmpSim, "Reversed Flow", allWTRegs, vCFF, -1.0
        );

    //REPORTS
    tmpSim.println("Making Total Airfoil");
    if(allAirfoilObjects.size()>0){
      ArrayList<Report> airfoilTotalReportsList =
          setupTotalAirfoilReports(tmpSim,
              this.getPrimaryDomainTag().getPresentationName() + "."+ "Aero",
              inletCsys
              );
      for(Report tmpReport : airfoilTotalReportsList){
        Monitor tmpMon = MonitorTool.reportIterationMonitor(
            tmpSim, tmpReport, nPlotSamples, itMonFreq, itMonStart
            );
        TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
      }
    }
    tmpSim.println("Making Deflectable CS");
    for(AerodynamicSurface tmpFoil:allAirfoilObjects){
      tmpSim.println(tmpFoil.getName());
      tmpFoil.makeReports(tmpSim);
      tmpFoil.TagReports(
          tmpSim, Collections.singleton(this.getPrimaryDomainTag()));
      tmpFoil.makeIterationMonitors(
          tmpSim, nPlotSamples, itMonFreq, itMonStart
          );

      // Tag the foil monitors
      for(Monitor tmpMon : tmpFoil.getAllMonitors()){
        TagTool.addTag(tmpSim, tmpMon, primaryWTDomainTag);
      }
    }
    
    
      //
      //PLOTS
      //
      //
      // MONITOR Plots
      MonitorPlot tmpPlot;

      // Airfoil CL, CD, CM plots
      String afString;
      String monAppend=" - It";

      //
      // XY Plots
      XYPlot xyPlot;
      Part tmpPart;
      double xMin;
      double xMax;
      double[] tmpColor=PlotTool.getColor("blue");

      // turbulent viscosity approaching airfoil
      tmpPart = lineProbe;
      xyPlot=PlotTool.getXYPlot(tmpSim,"Inlet Probe TVR","TVR");
      xyPlot.getParts().setObjects(tmpPart);
      xyPlot.setRepresentation(proxyRep);
      PlotTool.setXYPlotXVectorScalar(tmpSim, xyPlot, bodyCsys, "Position",0);
      PlotTool.setXYPlotYPrimitiveScalar(
          tmpSim, xyPlot, "TurbulentViscosityRatio"
          );
      //
      PlotTool.setPlotXAxis(xyPlot, "Position in Body",
          -10 * chordLength, 0.0, chordLength, 4);
      //
      PlotTool.setPlotYToLog(xyPlot, false);
      PlotTool.setPlotYAxis(xyPlot, "TVR", 0.1, refRe / 10.0, refRe / 100.0, 4);
      PlotTool.setYTypeLine(xyPlot, tmpPart, 2, tmpColor);
      PlotTool.setYTypeSymbol(xyPlot, tmpPart, tmpColor, 12, 2.0, 2);

      // wall y+ (c*0.15 section)
      tmpPart = xyCutPlane;
      xyPlot = PlotTool.getXYPlot(tmpSim, "All Airfoil Wall y+", "Wall +");
      TagTool.addTag(tmpSim, xyPlot, primaryWTDomainTag);
      xyPlot.getParts().setObjects(tmpPart);
      xyPlot.setRepresentation(proxyRep);
      PlotTool.setXYPlotXVectorScalar(tmpSim, xyPlot, bodyCsys, "Position", 0);
      PlotTool.setXYPlotYPrimitiveScalar(tmpSim, xyPlot, "WallYplus");
      //
      PlotTool.setPlotXAxis(
          xyPlot, "Position in Body", -0.5 * chordLength, chordLength, 0.1, 4
          );
      //
      PlotTool.setPlotYToLog(xyPlot, true);
      PlotTool.setPlotYAxis(xyPlot, "Wall Y+", 0.1, 500., 1.0, 4);
      PlotTool.setYTypeLine(xyPlot, tmpPart, 2, tmpColor);
      PlotTool.setYTypeSymbol(xyPlot, tmpPart, tmpColor, 12, 2.0, 2);
      //
      //SCENES
      tmpSim.println("WT MSG: Generating Scenes.");
      Scene tmpScene;
      PartDisplayer tmpPD;
      ScalarDisplayer tmpSD;
      double AR=2.; //x/y
      double yViewSize=chordLength;
      double xPos=-AR/6*yViewSize;
      double tanYViewAngle=Math.tan(15.0*Math.PI/180);
      double zPos=yViewSize/tanYViewAngle;

      //Mesh Cut
      tmpScene=SceneTool.getScene(tmpSim,"Mesh Cut");
      tmpPD = SceneTool.getPartDisplayer(tmpScene, "Volume Mesh",
              Collections.singleton(xyCutPlane), proxyRep);
      tmpPD.setMesh(true);
      tmpPD.setSurface(true);
      SceneTool.setSceneView(tmpScene,new double[]{-xPos,0.,0.},
              new double[] {-xPos,0.,zPos},
              yVector,
              labCsys,1./AR,1);

      //Velocity Magnitude
      tmpScene=SceneTool.getScene(tmpSim,"Velocity Magnitude");
      tmpSD = SceneTool.getScalarDisplayer(tmpScene,
              "Velocity",Collections.singleton(xyCutPlane));
      tmpSD.setRepresentation(proxyRep);
      SceneTool.setScalarVectorComponent(tmpSD,"Velocity",inletCsys,1,"Auto","Off",0.0,refVel,proxyRep);
      //SceneTool.setScalarDisplayerField(tmpSD,"Velocity","Off","Off",0.0,refVel,proxyRep);
      SceneTool.setSceneView(tmpScene,new double[]{-xPos,0.,0.},
              new double[] {-xPos,0.,zPos},
              yVector,
              labCsys,1./AR,1);
  }
public void outputAllScenes(Simulation tmpSim, String folderPath){
  // Folder paths should be absolute
  folderPath = SystemTool.getAbsoluteFolderPath(folderPath);
  SystemTool.touchDirectoryChain(tmpSim, folderPath);

  String userPath = folderPath + "USER";
  String cfdPath = folderPath + "CFD";
  SystemTool.touchDirectoryChain(tmpSim, userPath);
  SystemTool.touchDirectoryChain(tmpSim, cfdPath);

  // USER IMAGE PATH
  String userPlots  = userPath + File.separator + "PLOTS" + File.separator;
  String userScenes = userPath + File.separator + "SCENES" + File.separator;
  String userCSV    = userPath + File.separator + "CSV" + File.separator;

  SystemTool.touchDirectoryChain(tmpSim, userPlots);
  SystemTool.touchDirectoryChain(tmpSim, userCSV);
  SystemTool.touchDirectoryChain(tmpSim, userScenes);

  // CFD IMAGE PATH
  String cfdPlots  = cfdPath + File.separator + "PLOTS"  + File.separator;
  String cfdScenes = cfdPath + File.separator + "SCENES" + File.separator;
  String cfdCSV    = cfdPath + File.separator + "CSV"    + File.separator;

  SystemTool.touchDirectoryChain(tmpSim, cfdPlots);
  SystemTool.touchDirectoryChain(tmpSim, cfdCSV);
  SystemTool.touchDirectoryChain(tmpSim, cfdScenes);

  // Alpha.
  NumberFormat formatsmall = new DecimalFormat( "0.##" );
  String alphaStr= "_A";
  if (this.alphaAngle < (0.0 - SMALL_EPS)){
    alphaStr += "-";
  }
  if (Math.abs(this.alphaAngle) < (10.0 - SMALL_EPS)){
    alphaStr += "0";
  }
  alphaStr = alphaStr + formatsmall.format(Math.abs(this.alphaAngle));
  
  // PLOTS
  ArrayList<StarPlot> thisWTPlots = new ArrayList();
  for(StarPlot tmpPlot : tmpSim.getPlotManager().getObjects()){
    if(TagTool.getObjectTags(tmpPlot).contains(this.getPrimaryDomainTag())){
      thisWTPlots.add(tmpPlot);
    }
  }
  int wtTagLength = this.getPrimaryDomainTag().getPresentationName().length();
  //Airfoil
  // Cf and Cp charts
  for(StarPlot tmpPlot : thisWTPlots){
    String tmpPlotName = tmpPlot.getPresentationName();
    if(tmpPlotName.contains(".Airfoil")){
      String shortName = tmpPlotName.substring(wtTagLength + 9) + alphaStr;
      try{
        StarPlot.createParentDirs(new File(userPlots));
      }catch(IOException i){
        tmpSim.println("WTDomain StarPlot .Airfoil IOException createParentDirs for userPlots");
      }
      try{
        StarPlot.createParentDirs(new File(userCSV));
      }catch(IOException i){
        tmpSim.println("WTDomain StarPlot .Airfoil IOException createParentDirs for userCSV");
      }
      try{
        tmpPlot.encode(userPlots + shortName
            + ".png", "png", xyRes[0], xyRes[1]);
      }catch(NeoException e){
        tmpSim.println("WTDomain StarPlot .Airfoil to png ERR.");
      }
      try{
        tmpPlot.serverExportToSCE(userPlots + shortName
           + ".sce", false, false, tmpPlot.getPresentationName(), "Airfoil Performance");
      }catch(NeoException e){
        tmpSim.println("WTDomain StarPlot .Airfoil to sce ERR.");        
      }
      try{
        tmpPlot.export(userCSV + shortName.replaceAll(" ","_") + ".csv", ",");
      }catch(NeoException e){
        tmpSim.println("WTDomain StarPlot .Airfoil to csv ERR.");
      }
    }
  }

  for(StarPlot tmpPlot : thisWTPlots){
    String tmpPlotName = tmpPlot.getPresentationName();
    String shortName = tmpPlotName.substring(wtTagLength + 1) + alphaStr;
    try{
      StarPlot.createParentDirs(new File(cfdPlots));
    }catch(IOException i){
      tmpSim.println("WTDomain StarPlot .Airfoil IOException createParentDirs for cfdPlots");
    }
    try{
      StarPlot.createParentDirs(new File(cfdCSV));
    }catch(IOException i){
      tmpSim.println("WTDomain StarPlot .Airfoil IOException createParentDirs for cfdCSV");
    }
    try{
      tmpPlot.encode(cfdPlots + shortName + ".png", "png", xyRes[0], xyRes[1]);      
    }catch(NeoException e){
      tmpSim.println("WTDomain StarPlot encode png ERR");
    }
    try{
      tmpPlot.serverExportToSCE(cfdPlots + shortName
          + ".sce", false, false, tmpPlot.getPresentationName(), "Wind Tunnel Plot");
    }catch(NeoException e){
      tmpSim.println("WTDomain StarPlot encode sce ERR");
    }
    try{
      tmpPlot.export(cfdCSV + shortName.replaceAll(" ","_") + ".csv",",");
    }catch(NeoException e){
      tmpSim.println("WTDomain StarPlot encode csv ERR");
    }
  }

  //SCENES
  ArrayList<Scene> thisWTScenes = new ArrayList();
  for(Scene tmpScene : tmpSim.getSceneManager().getObjects()){
    if(TagTool.getObjectTags(tmpScene).contains(this.getPrimaryDomainTag())){
      thisWTScenes.add(tmpScene);
    }
  }
  wtTagLength = this.getPrimaryDomainTag().getPresentationName().length();
  for(Scene tmpScene : thisWTScenes){
    String tmpSceneName = tmpScene.getPresentationName();
    if(tmpSceneName.contains(".AUTO")){
      tmpScene.printAndWait(userScenes + tmpSceneName.substring(wtTagLength + 6) + alphaStr + ".png", 1,
          xyRes[0], xyRes[1], true, false
          );
    }
  }

  for(Scene tmpScene : thisWTScenes){
    String tmpSceneName = tmpScene.getPresentationName();
    if(tmpSceneName.contains(".CFD")){
      tmpScene.printAndWait(cfdScenes + tmpSceneName.substring(wtTagLength + 5) + alphaStr + ".png", 1,
          xyRes[0], xyRes[1], true, false);
    }
  }
}

public String getSheetsCSVData(Simulation tmpSim){
  String retString = "";

  //  get primary boundary condition information
  //  --> dictates freestream or velocity inlet
  //  if inlet exists, it is a velocity inlet type of case
  //  else it is a freestream case
  Boundary mainBndy = this.getAnInletBoundary();
  double bcVel;
  double bcTemp;
  double bcTVR;
  double bcTi;
  double bcRho;
  double bcMa;
  double bcRe;
  double bcMu;
  if(mainBndy.getBoundaryType() instanceof FreeStreamBoundary){
    //Mach at inlet
    MachNumberProfile machNP = 
        mainBndy.getValues().get(MachNumberProfile.class);
    bcMa = machNP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();
    // Temperature
    StaticTemperatureProfile sTP = 
        mainBndy.getValues().get(StaticTemperatureProfile.class);
    bcTemp = sTP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

    // K
    TurbulentKineticEnergyProfile turbulentKineticEnergyProfile_0 = 
      mainBndy.getValues().get(TurbulentKineticEnergyProfile.class);
    bcTi = turbulentKineticEnergyProfile_0
        .getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();
    // Omega
    SpecificDissipationRateProfile specificDissipationRateProfile_0 = 
      mainBndy.getValues().get(SpecificDissipationRateProfile.class);
    bcTVR=specificDissipationRateProfile_0
        .getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

    // Calculate Re, Mach
    double airR = 8.3144598;  // [kJ/kmol.K]  air gas constant
    double molAir = 0.0289645; // [kg/mol] molar gas constant of air
    double refPress = CFD_Physics
        .getReferencePressure(this.getPhysicsContinuum()); // [Pa] reference pressure for air

    // Get dynamic viscosity from WT Physics
    bcMu = CFD_Physics.getMu(this.getPhysicsContinuum());
    bcRho = refPress/bcTemp/(airR/molAir);
    bcVel = this.getRefVel();
    bcRe  = this.getChordLength()*bcVel*bcRho/bcMu;

  }else{
    // Velocity
    VelocityMagnitudeProfile vMP = 
        mainBndy.getValues().get(VelocityMagnitudeProfile.class);
    bcVel = vMP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();
    // Temperature
    StaticTemperatureProfile sTP = 
      mainBndy.getValues().get(StaticTemperatureProfile.class);
    bcTemp = sTP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

    // TuI
    TurbulenceIntensityProfile tIP = 
        mainBndy.getValues().get(TurbulenceIntensityProfile.class);
    bcTi = tIP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

    // TVR
    TurbulentViscosityRatioProfile tVRP = 
        mainBndy.getValues().get(TurbulentViscosityRatioProfile.class);
    bcTVR = tVRP.getMethod(ConstantScalarProfileMethod.class)
        .getQuantity().getRawValue();

    // Get dynamic viscosity from WT Physics
    bcMu=CFD_Physics.getMu(this.getPhysicsContinuum());

    // Calculate Re, Mach
    double airR = 8.3144598;  // [kJ/kmol.K]  air gas constant
    double molAir = 0.0289645; // [kg/mol] molar gas constant of air
    double refPress = CFD_Physics
        .getReferencePressure(this.getPhysicsContinuum()); // [Pa] reference pressure for air

    //Temp=refPress/refRho/(airR/molAir)
    bcRho = refPress/bcTemp / (airR / molAir);
    bcRe = this.getChordLength() * bcVel * bcRho / bcMu;
    bcMa = bcVel / Math.sqrt(1.4 * (airR / molAir) * bcTemp);
  }

  retString = retString + "," + bcRe + "," + bcMa + "," 
      + this.getChordLength() + "," + bcVel + "," + bcTemp + ","
      + bcRho+","+bcMu+","+bcTVR+","+bcTi;

  //Total Airfoil Performance (If it exists);
  double primaryLengthScale = 0.0;
  if(allAirfoilObjects.size() > 0){
    //chord length
    primaryLengthScale = chordLength;
    retString = retString + "," + primaryLengthScale;
    //reference Area
    String referenceArea=Double.toString(refArea);
    retString=retString+","+referenceArea;
    //reference Radius not needed because it is always chord length
    //
    // turn table angle
    retString = retString + "," + alphaAngle;
    retString = retString + "," + getTotalAirfoilReportCSVData(
        tmpSim, statSamples);
  }

  //Output global mesh settings
  tmpSim.println("WT MSG: Post processing WT Mesh Settings...");
  retString=retString + "," + getMainTunnelMeshString(tmpSim);

  //SPECIFIC OBJECT POST-PROCESSING INFORMATION
  // Individual Airfoil Element Information
  for(AerodynamicSurface tmpFoil : allAirfoilObjects){
      retString = retString + "," + tmpFoil.csvData(tmpSim);
  }
  
  return retString;
}
public void implement2DObjectsIntoScenes(Simulation tmpSim){
    // for 2D, we want to display the fields from the all region boundaries
    ArrayList<Domain> allDomainObjects = new ArrayList();
    allDomainObjects.add(this);
    allDomainObjects.addAll(allOversetDomains);

    ArrayList<Region> regionObjects = new ArrayList();
        
    for(Domain tmpDomain : allDomainObjects){
      regionObjects.add(tmpDomain.getRegion());
    }
    
    

    // Get all the wall boundaries in the domain to inject into the outline 
    // displayer.
    ArrayList<Boundary> injectedWallBoundaries = new ArrayList();
    for(Domain tmpDomain:allDomainObjects){
      for(Boundary tmpBndy:tmpDomain.getRegion().getBoundaryManager().getBoundaries()){
        if(tmpBndy.getBoundaryType() instanceof WallBoundary){
          injectedWallBoundaries.add(tmpBndy);
        }
      }
    }

    
    ArrayList<Scene> thisWTScenes = new ArrayList();
    for(Scene tmpScene : tmpSim.getSceneManager().getObjects()){
      if(TagTool.getObjectTags(tmpScene).contains(this.getPrimaryDomainTag())){
        thisWTScenes.add(tmpScene);
      }
    }

    for(Scene tmpScene : thisWTScenes){
      for(Displayer tmpDisp:tmpScene.getDisplayerManager().getObjects()){
        // Inject Regions into the Scalar Displayer
        if (tmpDisp instanceof ScalarDisplayer){
          tmpDisp.getInputParts().setObjects(regionObjects);
        // Inject Wall Boundaries into the Outline Displayer.
        } else if(tmpDisp instanceof PartDisplayer &&
            !tmpDisp.getPresentationName().contains("mesh")){
          tmpDisp.getInputParts().setObjects(injectedWallBoundaries);
        // Inject Regions into other Part Displayers
        } else if(tmpDisp instanceof PartDisplayer){
          tmpDisp.getInputParts().setObjects(regionObjects);
        }
      }
    }
  }
public void initAnnotations(Simulation tmpSim, Collection<Annotation> newAnnotations){
  this.standardAnnotations = newAnnotations;

  Annotation wtAnnotation = wtNameAnnotation(tmpSim);
  if(!this.wtAnnotations.contains(wtAnnotation)){
    this.wtAnnotations.add(wtAnnotation);
  }

}
private Annotation wtNameAnnotation(Simulation tmpSim){
  SimpleAnnotation caseAnnotation;
  String wtTagName = this.getPrimaryDomainTag().getPresentationName();
  try{
    caseAnnotation = (SimpleAnnotation) tmpSim.getAnnotationManager()
        .getObject(wtTagName + "." + "Tunnel Name");
  }catch(NeoException e){
    caseAnnotation = tmpSim.getAnnotationManager().createSimpleAnnotation();
    caseAnnotation.setPresentationName(wtTagName + "." + "Tunnel Name");
  }
  caseAnnotation.setText("WT: " + wtTagName);
  caseAnnotation.setDefaultHeight(0.0375);
  caseAnnotation.setDefaultPosition(
      new DoubleVector(new double[] {0.0075, 0.045, 0.0}));
  return caseAnnotation;
}
//  OPTIMIZATION
public void setUpAlphaSweepMetrics(
  Simulation tmpSim, double[] alphaSweepAngles){
  String preFix = 
      this.getPrimaryDomainTag().getPresentationName()+ "." +"Sweep Metric ";
  String metricName;
  String reportName;
  double initialValue = 99.0;
  String metricReportName;
  NumberFormat formatsmall = new DecimalFormat("0.0");

  //CL_MIN Metric
  metricName = "CL MIN";
  metricReportName = preFix + metricName;
  modifySweepMetricValue(tmpSim, metricReportName, initialValue);

  //CL_MAX Metric
  metricName = "CL MAX";
  metricReportName = preFix + metricName;
  modifySweepMetricValue(tmpSim, metricReportName, -1.0 * initialValue);

  for(double tmpAngle:alphaSweepAngles){
    String tmpAnglePostFix = " "+formatsmall.format(tmpAngle).replace('.', 'p');
    //CL Metric
    metricName = "CL";
    reportName = "Aero Airfoil Total - CL";
    metricReportName = preFix + metricName+tmpAnglePostFix;
    modifySweepMetricValue(tmpSim, metricReportName, -1.0 * initialValue);

    //CD Metric
    metricName = "CD";
    reportName = "Aero Airfoil Total - CD";
    metricReportName = preFix + metricName+tmpAnglePostFix;
    modifySweepMetricValue(tmpSim, metricReportName, -1.0 * initialValue);

    //LoverD Metric
    metricName = "LoverD";
    reportName = "Aero Airfoil Total - CD";
    metricReportName = preFix + metricName+tmpAnglePostFix;
    modifySweepMetricValue(tmpSim, metricReportName, -100. * initialValue);
  }
}
public void modifySweepMetricValue(
    Simulation tmpSim, String expressionReportName, double thisValue){
      ExpressionReport tmpReport;
      try{
        tmpReport = 
            (ExpressionReport) tmpSim
                .getReportManager().getObject(expressionReportName);
        tmpReport.setDefinition("" + thisValue);
      }catch(NeoException e){
        tmpReport = 
            (ExpressionReport) tmpSim
                .getReportManager().createReport(ExpressionReport.class);
        tmpReport.setPresentationName(expressionReportName);
        tmpReport.setDefinition("" + thisValue);
      }
      TagTool.addTags(tmpSim, tmpReport, 
          Collections.singleton(this.getPrimaryDomainTag()));
  }
public void completeSweepMetricValues(
    Simulation tmpSim, double thisAngle){
    NumberFormat formatsmall = new DecimalFormat("0.0");
    String preFix = this.getPrimaryDomainTag().getPresentationName()
        + "." + "Sweep Metric ";
    String metricName;
    String reportName;
    String metricReportName;
    String tmpAnglePostFix = " "+formatsmall.format(thisAngle).replace('.', 'p');
    //
    int numSamples = 500;

    // FOR EVERY POINT IN THE SWEEP WE INPUT CL, CD, and CL/CD
    //  REGARDLESS OF VALIDITY
    //
    // CL Metric
    //
    metricName = "CL";
    String rootName = this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total";
    reportName = rootName+" - "+metricName;
    metricReportName = preFix+metricName+tmpAnglePostFix;
    String appEnd=" - It";
    String tmpName;
    //
    double CL_aveVal;
    double CL_stdDev;
    double LoverD;
    Report tmpRep;
    tmpName=reportName;
    tmpRep = ReportTool.getReport(tmpSim,tmpName);
    CL_aveVal = SimTool.getMeanReportItVal(tmpRep,appEnd,numSamples);
    CL_stdDev = SimTool.getMeanReportStddevItVal(tmpRep,appEnd,numSamples,CL_aveVal);
    modifySweepMetricValue(tmpSim, metricReportName, CL_aveVal);
    LoverD = CL_aveVal; // special for LoverD metric

    //
    // CD Metric
    //
    double CD_aveVal;
//      double CD_stdDev;
    metricName = "CD";
    rootName = this.getPrimaryDomainTag().getPresentationName()
        + "." + "Aero Airfoil Total";
    reportName = rootName+" - "+metricName;
    metricReportName = preFix+metricName+tmpAnglePostFix;
    appEnd=" - It";
    //
    tmpName=reportName;
    tmpRep = ReportTool.getReport(tmpSim,tmpName);
    CD_aveVal = SimTool.getMeanReportItVal(tmpRep,appEnd,numSamples);
//      CD_stdDev = SimTool.getMeanReportStddevItVal(tmpRep,appEnd,numSamples,CD_aveVal);
    modifySweepMetricValue(tmpSim, metricReportName,CD_aveVal);
    LoverD=LoverD/CD_aveVal;

    //
    // L/D Metric
    //
    metricName = "LoverD";
    metricReportName = preFix+metricName+tmpAnglePostFix;
    modifySweepMetricValue(tmpSim, metricReportName,LoverD);

    //=================================
    // GLOBAL SWEEP CHARACTERISTICS
    //=================================
    // Cases only take CLMax or CLMin if they meet validity criterion
    double convergenceTolerance = 0.05;
    //CL_MIN Metric
    metricName = "CL MIN";
    metricReportName = preFix+metricName;
    ExpressionReport clMinValue = (ExpressionReport) tmpSim.getReportManager().getReport(metricReportName);
    double clMinOld = Double.parseDouble(clMinValue.getDefinition());
    if( CL_aveVal < clMinOld && Math.abs( CL_stdDev / CL_aveVal ) < convergenceTolerance ){
      modifySweepMetricValue(tmpSim, metricReportName, CL_aveVal);
    }

    //CL_MAX Metric
    metricName = "CL MAX";
    metricReportName = preFix+metricName;
    ExpressionReport clMaxValue =
        (ExpressionReport) tmpSim.getReportManager()
            .getReport(metricReportName);
    double clMaxOld = Double.parseDouble(clMaxValue.getDefinition());
    if( CL_aveVal > clMaxOld && Math.abs( CL_stdDev / CL_aveVal ) 
        < convergenceTolerance ){
      modifySweepMetricValue(tmpSim, metricReportName, CL_aveVal);
    }
}

///
}// End Class WindTunnelDomain
