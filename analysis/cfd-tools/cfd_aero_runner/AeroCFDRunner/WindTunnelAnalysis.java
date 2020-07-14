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
package AeroCFDRunner;

import java.util.*;
import java.io.*;

import star.common.*;
import star.base.neo.*;
import star.meshing.*;
import star.flow.*;
import star.base.query.*;

import AeroToolKit.*;
import GeometricObject.*;
import MeshTools.*;
import PhysicsTools.*;
import Domain.*;
import Naming.*;
import Tools.*;
import Solvers.*;
import java.awt.Color;
import java.io.File;
import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import star.base.report.*;
import star.coupledflow.CoupledImplicitSolver;
import star.prismmesher.*;
import star.energy.StaticTemperatureProfile;
import star.turbulence.SpecificDissipationRateProfile;
import star.turbulence.TurbulenceIntensityProfile;
import star.turbulence.TurbulentKineticEnergyProfile;
import star.turbulence.TurbulentViscosityRatioProfile;
import star.vis.*;

public class WindTunnelAnalysis {
// constants
final double SMALL_EPS = 1.0E-6;
final double TINY_VALUE = 1.0E-20;

// Debugging feature
Simulation tmpSim;
final int verbosity = 1; // default verbosity is 0, minimal window output

//Simulation
Representation proxyRep;

// Type of windtunnel simulation
String wtType;
boolean is2D  = false; // 2D flag
boolean isWT  = false; // Windtunnel flag
boolean isFS  = false; // Freestream flag
boolean isUns = false; // Unsteady flag

//Geometry
Collection<GeometryPart> allGeomParts;
NamingConvention globalNames;

//String identifier List
String   inStr;
String   outStr;
String   fsStr;
String   symmStr;
String   wallStr;
String[] allDomStr = {inStr, outStr, fsStr, symmStr, wallStr};

ArrayList<String> airfoilStrList;

// geometric configuration
double chordLength;
double oldTurnTableAngleInDeg = 0.0;

double[] wtAxisRot = {0., 0., 1.}; //windtunnel geometries always about Z axis

// Domain objects
ArrayList<WindTunnelDomain> allWTDomains;
ArrayList<String> allWTDomainNames;

// It is OK for the analysis to understand what Overset Domains exist.
ArrayList<OversetDomain>      allOversetDomains = new ArrayList();

ArrayList<Tag> allWindTunnelTags;

//Meshing
double baseSize;

// Physics
double highMachNumber = 0.4;

double refRho;
double refVel;
double refTemp;
double refMu;
double refMa;
double refArea;
double refRe;

//Default Freestream values
double fsTVR = 10.0;
double fsTi  = 0.01;

// Regions
ArrayList<Region> allWTRegs = new ArrayList();
SolverDriver solverDriver   = new SolverDriver();

//Annotations
String caseAnnotationName = "";
ArrayList<Annotation> standardAnnotations = new ArrayList();
ArrayList<Annotation> highMachAnnotations = new ArrayList();

// Coordinate Systems
LabCoordinateSystem labCsys;
CartesianCoordinateSystem bodyCsys;
CartesianCoordinateSystem inletCsys;
ArrayList<CartesianCoordinateSystem> airfoilCoordSystems;
ArrayList<CartesianCoordinateSystem> oversetCoordSystems;

// Aerodynamicist Tools
double commonAlphaValue = 0.0;
ArrayList<BoundaryLayerProbe> allBoundaryLayerProbes = new ArrayList();

//Visualization tools
int[]    xyRes = {2000, 1000};
double   viewAngleDistanceOffset = Math.atan(15.0 * Math.PI / 180.0);

// Tags
ArrayList<Tag> existingWindTunnelTags;

//Basic Stuff
double[] xVector    = { 1.,  0.,  0.};
double[] negativeX  = {-1.,  0.,  0.};
double[] yVector    = { 0.,  1.,  0.};
double[] negativeY  = { 0., -1.,  0.};
double[] zVector    = { 0.,  0.,  1.};
double[] negativeZ  = { 0.,  0., -1.};
double[] zeroVector = { 0.,  0.,  0.};

//results statistics
int statSamples = 500;

public WindTunnelAnalysis(Simulation tmpSim, double chordLength){

  //Simulation controls
  this.proxyRep = SimTool.getSimProxy(tmpSim, "Proxy");

  // Get all geometry parts in Tree.
  this.allGeomParts = tmpSim.getGeometryPartManager().getObjects();

  // Get coordinate systems
  this.labCsys = tmpSim.getCoordinateSystemManager()
          .getLabCoordinateSystem();

  // Meshing values
  this.chordLength = chordLength;
  this.baseSize    = chordLength/10.0;
  
  this.globalNames = new NamingConvention();
  
  this.existingWindTunnelTags = TagTool.getSimulationTagsByPrefix(
      tmpSim, globalNames.getWindTunnelTagID()
      );
  if(existingWindTunnelTags.isEmpty()){
    // Do not allow a WindTunnel Analysis to proceed without WindTunnel
    // Domains being appropriately tagged. Backwards compatibility should be
    // obtained w/ v1 by manually adding a WindTunnel object tag.
    throw new NeoException("WT ERR: There are no Wind Tunnel Tags!");
  }
  
  // Track all the WindTunnelDomain Objects in this analysis.
  this.allWTDomains = new ArrayList();
}

// WIND TUNNEL ANALYSIS PROCESSES
public void setTmpSim(Simulation tmpSim){
  this.tmpSim = tmpSim;
}

public void setIs2D(boolean isAnalysis2D){
  is2D = isAnalysis2D;
}
public boolean getIs2D(){
  return is2D;
}
public void runPreProc(Simulation tmpSim, boolean justGetNames){
  tmpSim.println("WT PRE: Running preprocessor.");
  /* Method runPreProc is responsible for setting up a simulation file from
     scratch, or re-preprocessing a case that was set up previously.

     Generally, objects are first realized, mesh operations are created.
     From there, the remaining objects that would be desired in a wind tunnel 
     simulation are instantiated. The necessary meshing and region setup 
     are performed afterward. 
     Tags that indicate which parts belong in which WindTunnel Domain are
     required in these raw simulation file locations:
      1) GeometryPart
      2) CoordinateSystem
     These tags will proliferate across a pre-processed sim file, most notably
     to Regions associated with a given WindTunnel Domain.
  */

  // Filter out GeometryParts that are not appropriate for the wind tunnel
  // workflow. In this case, MeshOperationParts are not acceptable Leaf level
  // GeometryParts.
  allGeomParts = MeshOpTool.removeMeshOpParts(allGeomParts);

  // Get all volume controls in the simulation.
  // It is ok for the total analysis to understand what volume controls exist
  // inside the simulation. The analysis is responsible for dolling them out
  // to each wind tunnel domain via tags.
  // The Wind Tunnel Domains are responsible for actually making use of them.
  ArrayList<GeometryPart> allVolumeControlParts = getVolumeControlParts();

  // Instantiate any Domain objects.
  // This will allow the pre-processor to understand what objects exist in
  // the simulation and make decisions about how to set them up accordingly.
  tmpSim.println("Getting OversetDomains");
  ArrayList<OversetDomain> oversetDomains = 
      instantiateExistingOversetDomainObjects(tmpSim);

  tmpSim.println("Getting WindTunnelDomains");
  ArrayList<WindTunnelDomain> windTunnelDomains =
      instantiateExistingWindTunnelDomainObjects(tmpSim);

  if(existingWindTunnelTags.size() != windTunnelDomains.size()){
    tmpSim.println("WT");
    throw new NeoException("WT ERR: number of Wind Tunnel Tags and"
        + " WindTunnelDomains do not match!");
  }

  // WindTunnel Domains are responsible for manipulating their own Overset
  // Domains. Assign the correct OversetDomains to the correct
  // WindTunnel Domains by tag identification.
  for(WindTunnelDomain wtDomain : windTunnelDomains){
    Tag wtDomainTag = wtDomain.getPrimaryDomainTag();
    for(OversetDomain osDomain : oversetDomains){
      if(osDomain.getPrimaryDomainTag() == wtDomainTag){
        wtDomain.addOversetDomain(osDomain);
      }
    }
  }

  if(oversetDomains.size()>0){
    if(verbosity > 0){
      tmpSim.println("WT MSG: OversetDomain(s) detected." +
          "Setting all SimpleBlockParts to CAD Zero Body coordinates.");
    }
    // If pre-processing, the assumption is that the case file will be
    // remeshed. So, each Overset CAD coordinate angle must be returned to
    // the correct 0 degree orientation.
    for(OversetDomain tmpDom:oversetDomains){
        tmpDom.zeroOutDomainCsys();
        tmpDom.determineDomainAngle();
    }

    // Any volume control part that exists in the sim file that is a
    // SimplePartforce must be set to Lab. 
    // It is a requirement that the STAR-CCM+ Lab and all Objects' CAD x-axis,
    // e.g. chord line at zero alpha, be aligned in the same x-axis 
    // orientation.
    //
    // Because there are an arbitrary number of domains, only the STAR-CCM+
    // Lab axis remains unique and immuteable.
    for(GeometryPart tmpPart : allVolumeControlParts){
      if(tmpPart instanceof SimpleBlockPart){
        ((SimpleBlockPart) tmpPart).setCoordinateSystem(labCsys);
          }
    }
  }

  // There are no raw airfoil GeometryParts allowed anymore in this workflow.
  // TODO: incorporate raw GeometryParts and mesh operations to
  // accomodate non-overset workflows. Although, this was an old workflow and
  // not particularly useful.

  // The first attempt will make multiple mesh operations per wind tunnel
  // domain. This is not particularly optimal, but wil allow multiple geometry
  // perturbations, i.e. flap settings, of a single airfoil.
  //
  // TODO: Use tags to control the splitting of mesh operations. It is
  //   possible that volume controls across all of space will cause
  //   performane problems.

  //   The reason for this is as follows: If there are 100 GeometryPart mesh
  //   operations to complete as in a single kite with multiple airfoil
  //   perturbations, a single mesh operation can be run in parallel
  //   concurrently. This will reduce the total meshing time on a cluster.
  //
  //   The downside is that a single bad GeometryPart will tank the entire
  //   process. The likelyhood of a bad CAD part increases with number of 
  //   GeometryParts present in the simulation.

  // Each WindTunnelDomain will control the mesh operation used for itself
  // and any other objects under it control. The order of operations in the
  // meshing tree is important.
  ArrayList<MeshOperation> orderedMeshOperations = new ArrayList();

  for(WindTunnelDomain mainDomain : windTunnelDomains){
    Tag primaryTag = mainDomain.getPrimaryDomainTag();
    String primaryTagName = primaryTag.getPresentationName();

    // Set an initial primary chord length
    double thisWTBaseSize;
    if(!mainDomain.checkIfWTChordLengthExists(tmpSim)){
      mainDomain.setChordLength(tmpSim, chordLength);
      thisWTBaseSize = chordLength / 10.0;
    }else{
      thisWTBaseSize = mainDomain.getChordLengthFromParameters(tmpSim) / 10.0;
    }
    
    // Set an initial primary reference area
    if(!mainDomain.checkIfWTReferenceAreaExists(tmpSim)){
      mainDomain.setReferenceArea(tmpSim, refArea);
    }else{
    }

    // Set an initial primary reference area
    if(!mainDomain.checkIfWTMomentReferenceVectorExists(tmpSim)){
      mainDomain.setMomentReferenceVector(tmpSim, 
          new double[] {mainDomain.getChordLength(),
              mainDomain.getChordLength(), mainDomain.getChordLength()
              }
          );
    }else{
    }

    // Set surface meshing operation up.
    String surfaceMeshOpName = primaryTagName + ".SM";
    AutoMeshOperation wtSurfaceOp = MeshOpTool.surfMeshOp(
        tmpSim, surfaceMeshOpName, thisWTBaseSize,
        100.0, 10.0, 32.0, 1.3, false
        );
    wtSurfaceOp.setMeshPartByPart(true);
    mainDomain.setDomainSurfMeshOp(wtSurfaceOp);
    wtSurfaceOp.getInputGeometryObjects().setQuery(null);
    wtSurfaceOp.getInputGeometryObjects().setObjects(
    mainDomain.getGeometryPartsOfControlledObjects());

    // custom settings for far field boundaries
    double domainTargetPct = 400.0;
    SurfaceCustomMeshControl domainControl =
            MeshOpTool.surfControl(
                mainDomain.getDomainSurfMeshOp(), "00X Domain"
                );
    MeshOpTool.surfFilter(domainControl,2);
    MeshOpTool.surfCustTargSize(domainControl,"Relative", domainTargetPct);
    MeshOpTool.surfEdgeProx(domainControl, 5.0);
    
    // All Symmetry Planes
    SurfaceCustomMeshControl allSymmetryControl = 
        MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(),
        ""+globalNames.getSymmStr()+ " Symm"
        );
    MeshOpTool.surfFilter(allSymmetryControl, 
        globalNames.getSymmStr().length());
    MeshOpTool.surfEdgeProx(allSymmetryControl, 5.0);
    
    // All Overset Surfaces
    SurfaceCustomMeshControl allOversetControl = 
        MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(),
        ""+globalNames.getOversetStr()+ " Overset"
        );
    MeshOpTool.surfFilter(allOversetControl, 
        globalNames.getOversetStr().length());
    MeshOpTool.surfEdgeProx(allOversetControl, 3.0);

    // Set volume meshing operation up.
    // TODO: Investigate implementing the polyhedral mesher when the
    // case is a 2D case. It will be much faster.
    String volumeMeshOpName = primaryTagName + ".VM";
    CFD_TrimmerModel wtVolumeMeshOp = new CFD_TrimmerModel(tmpSim,
        volumeMeshOpName, labCsys, thisWTBaseSize);
    wtVolumeMeshOp.getVMOp().setMeshPartByPart(true);
    wtVolumeMeshOp.getVMOp().getDefaultValues()
      .get(PartsTargetSurfaceSize.class).setRelativeSize(400.0);
    mainDomain.setDomainVolumeMeshOp(wtVolumeMeshOp.getVMOp());
    wtVolumeMeshOp.addParts(mainDomain.getGeometryPartsOfControlledObjects());
    mainDomain.setDomainVolumeMeshOp(wtVolumeMeshOp.getVMOp());

    // Apply Volume Controls related to this meshing operation
    ArrayList<NamedObject> tmpArrayList =
        new ArrayList(allVolumeControlParts);
    ArrayList<GeometryPart> relatedVolumeControlParts =
        new ArrayList(TagTool.filterObjectsByTag(
            tmpSim, tmpArrayList, primaryTag
            )
        );
    for(GeometryPart tmpPart : relatedVolumeControlParts){
      String groupID = getVCGroupID(tmpPart);
      String vcPctStr = getVCPctStr(tmpPart, groupID);
      double vcPct=Double.valueOf(vcPctStr);
      VolumeCustomMeshControl groupVC =
          wtVolumeMeshOp.getVolumeControl(groupID+" "+vcPctStr);
      groupVC.getGeometryObjects().add(tmpPart);
      MeshOpTool.custVCIsoSize(groupVC, "Relative", vcPct);
    }

    for(OversetDomain tmpDomain : mainDomain.getAllOversetDomains()){
      GeometryPart osPart = tmpDomain.getDomainPart();
      String osName = osPart.getPresentationName();
      VolumeCustomMeshControl groupVC =
        wtVolumeMeshOp.getVolumeControl(osName);
      groupVC.getGeometryObjects().add(osPart);
      if(!is2D){
        MeshOpTool.custVCIsoSize(groupVC, "Relative", 100.0);
      }else{
        MeshOpTool.custVCIsoSize(groupVC, "Relative", 50.0);
      }
  }

    // 2D Windtunnel Mesh Operation Modifications
    /* This is where we have the possibility of re-ordering the mesh operation
       tree. So, we need to track the previous mesh operations in the tree and
       the mesh operations that have been introduced.

       This modification is pretty straightforward - we just inject the
       2D badge for meshing up into the top of the wind tunnel meshing tree
       pipeline.
    */
    orderedMeshOperations.add(mainDomain.getDomainSurfMeshOp());
    orderedMeshOperations.add(mainDomain.getDomainVolumeMeshOp());
    if(is2D){
      // check for a 2D badging operation
      PrepareFor2dOperation badge2DMeshOp;
      String badgeName = primaryTagName + "."+ "Badge WT Objects";
      try{
        badge2DMeshOp = 
            (PrepareFor2dOperation) tmpSim.get(
                MeshOperationManager.class).getObject(badgeName);
      }catch(NeoException e){
          badge2DMeshOp = 
            (PrepareFor2dOperation) tmpSim.get(MeshOperationManager.class)
                    .createPrepareFor2dOperation(
                            new NeoObjectVector(
                              new Object[] {}));
          badge2DMeshOp.setPresentationName(badgeName);
      }
      orderedMeshOperations.add(orderedMeshOperations.size()-2,
          badge2DMeshOp);
      tmpSim.get(MeshOperationManager.class).reorderMeshOperations(
              new NeoObjectVector(orderedMeshOperations.toArray()));        

      // Fill Badge Operation with the relevant AutoMeshOperation
      // GeometryParts from the WindTunnelDomain.
      badge2DMeshOp.getInputGeometryObjects().setObjects(
          mainDomain.getGeometryPartsOfControlledObjects());

      // Meshing on the surface needs to be fixed
      // Currently there is a bug for the .CONCURRENT option
      // TODO: 14.02 check concurrent option functions
      AutoMeshOperation autoOp = mainDomain.getDomainVolumeMeshOp();
      autoOp.getMesherParallelModeOption().setSelected(
          MesherParallelModeOption.Type.SERIAL);
      
      // Turn off proximity in the farfield 00X values and change the edge
      // proximity to a value of 1.0. No need for refinement in the far reaches.
      MeshOpTool.surfEdgeProx(domainControl, 1.0);
      domainControl.getCustomConditions().get(PartsSurfaceProximityOption.class)
          .setSelected(PartsSurfaceProximityOption.Type.DISABLE);
      
      // Turn off proximity on the symmetry planes. Otherwise, it will cause the
      // farfield mesh to be overly too fine.
      MeshOpTool.surfEdgeProx(domainControl, 1.0);
      allSymmetryControl.getCustomConditions().get(PartsSurfaceProximityOption.class)
          .setSelected(PartsSurfaceProximityOption.Type.DISABLE);

    } // End of 2D badging reordering.
  } // End of Wind Tunnel Domains mesh operations setup.

  //
  // AIRFOILS
  //
  for(WindTunnelDomain mainDomain : windTunnelDomains){
    mainDomain.instantiateAirfoilObjects(tmpSim);
    mainDomain.applyAirfoilStandardMeshSettings(tmpSim);
  }

  
  // BOUNDARY LAYER PROBES
  // TODO: Get boundary layer probes on a tagged basis.
  for(WindTunnelDomain mainDomain : windTunnelDomains){
    mainDomain.setAllBLProbes(tmpSim, new ArrayList());
  }

  for(WindTunnelDomain mainDomain: windTunnelDomains){
    mainDomain.cleanUpDefaultBoundaries();
  }
  for(OversetDomain osDomain: oversetDomains){
    osDomain.cleanUpDefaultBoundaries();
  }

  if(justGetNames){
    // If the preprocessor is being used to get names of objects in the
    // simulation, wipe out the MeshOperations and Regions created by the
    // pre-processor.
    tmpSim.get(MeshOperationManager.class).removeObjects((tmpSim.get(MeshOperationManager.class).getObjects()));
    tmpSim.getRegionManager().deleteChildren(tmpSim.getRegionManager().getObjects());
  }
}

public void setAirfoilRelativeChangeStoppingCriterion(Simulation tmpSim,
    double pctCLChange, double pctCDChange, double pctCMChange){
  
  ArrayList<SolverStoppingCriterion> allFoilPercentCriteria = new ArrayList();
  
  for(WindTunnelDomain wtDomain : allWTDomains){
    String wtTagName  = wtDomain.getPrimaryDomainTag().getPresentationName();
    Collection<Report> tmpColl = wtDomain.getAeroTotalAirfoilReports(tmpSim);
    tmpColl.forEach( tmpReport -> {
      if(tmpReport.getPresentationName().contains("CL")){
        Monitor tmpMon = MonitorTool.getReportMonitor(
            tmpSim, tmpReport.getPresentationName() + globalNames.getItPostFix());
        allFoilPercentCriteria.add(
            CriterionTool.setItMonStdDevCriteria(tmpSim, tmpMon, 0.01, statSamples));
      }else if(tmpReport.getPresentationName().contains("CD")){
        Monitor tmpMon = MonitorTool.getReportMonitor(
            tmpSim, tmpReport.getPresentationName() + globalNames.getItPostFix());
        allFoilPercentCriteria.add(
            CriterionTool.setItMonStdDevCriteria(tmpSim, tmpMon, 0.002, statSamples));
      }else if(tmpReport.getPresentationName().contains("CmZ")){
        Monitor tmpMon = MonitorTool.getReportMonitor(
            tmpSim, tmpReport.getPresentationName() + globalNames.getItPostFix());
        allFoilPercentCriteria.add(
            CriterionTool.setItMonStdDevCriteria(tmpSim, tmpMon, 0.005, statSamples));
      }
    });
  }

  allFoilPercentCriteria.forEach( tmpCrit -> {
    tmpCrit.setIsUsed(true);
    tmpCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
  });
}

public void runSolver(Simulation tmpSim, int solverSteps){
  /* Runs the Windtunnel under the current settings and configuration.
     Even though multiple WindTunnel Domains may exist in the simulation,
     there is only one solver allowed per simulation.
  */
  tmpSim.getSolution().initializeSolution();
  for(WindTunnelDomain mainDomain : allWTDomains){
    mainDomain.relaxBLPProbes(tmpSim);
  }
  
  // TODO: Check to see if there is a way to get all solvers.
  SolverDriver.stepSimulation(tmpSim, allWTDomains.get(0).getPhysicsContinuum(),
      solverSteps, solverDriver.useAutoTimeStep());
}

// WIND TUNNEL ANALYSIS MAIN OBJECTS
public void instantiateExistingObjects(Simulation tmpSim){
  // Overset Domain instantiate.
  ArrayList<OversetDomain> existingOversetDomains = new ArrayList();
  existingOversetDomains.addAll(instantiateExistingOversetDomainObjects(tmpSim));

  // Wind Tunnel Domain instantiate.
  // If allWTDomains is not empty, the code has already run through instantiate.
  if(allWTDomains.size()<1){
    instantiateExistingWindTunnelDomainObjects(tmpSim);
  }
  
  // Overset Domain Communications Required after these two interacting objects
  // exist.
  // Make sure that main Wind Tunnel Domains are aware of associated Overset
  // Domain objects.
  for(OversetDomain osDomain : existingOversetDomains){
    Tag thisPrimaryOSTag = osDomain.getPrimaryDomainTag();
    for(WindTunnelDomain wtDomain : allWTDomains){
      if(thisPrimaryOSTag == wtDomain.getPrimaryDomainTag() 
          && wtDomain.getAllOversetDomains().size()<1){
        wtDomain.addOversetDomain(osDomain);
      }
    }
  }
  for(OversetDomain osDomain : existingOversetDomains){
    Tag thisPrimaryOSTag = osDomain.getPrimaryDomainTag();
    for(WindTunnelDomain wtDomain : allWTDomains){
      if(thisPrimaryOSTag == wtDomain.getPrimaryDomainTag()){
        osDomain.setDomainSurfMeshOp(wtDomain.getDomainSurfMeshOp());
        osDomain.setDomainVolumeMeshOp(wtDomain.getDomainVolumeMeshOp());
      }
    }
  }

  // Airfoil Objects
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.instantiateAirfoils(tmpSim);
  }

  // Boundary Layer Probe Objects
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.instantiateBoundaryLayerProbes(tmpSim);
  }
  
}

public ArrayList<WindTunnelDomain> instantiateExistingWindTunnelDomainObjects(
    Simulation tmpSim){

  ArrayList<WindTunnelDomain> returnWTDomains = new ArrayList();
  //===============================
  // Instantiate WindTunnelDomains.
  //===============================
  if(verbosity > 0){
    tmpSim.println("\nWT MSG: Instantiating WT Domain Objects...");
    tmpSim.println("WT MSG: ... from Regions in the simulation.\n");
  }

  returnWTDomains.addAll(instantiateWTDomainsFromRegions(tmpSim));
  if(returnWTDomains.isEmpty()){
    if(verbosity > 1){
      tmpSim.println("WT MSG: ... from GeometryParts in the simulation.");
    }
    returnWTDomains.addAll(instantiateWTDomainsFromGeometryParts(tmpSim));
  }

  // Make domains aware of 2D cases
  returnWTDomains.forEach(tmpWTDomain -> {
    tmpWTDomain.setIs2D(is2D);
  });
  
  for(WindTunnelDomain wtDomain : returnWTDomains){
    String primaryTagName = wtDomain.getPrimaryDomainTag().getPresentationName();
    String surfaceMeshOpName = primaryTagName + ".SM";
    String volumeMeshOpName = primaryTagName + ".VM";
    try{
      wtDomain.setDomainSurfMeshOp(
          MeshOpTool.getAutoMeshOp(tmpSim, surfaceMeshOpName));
      wtDomain.setDomainVolumeMeshOp(
          MeshOpTool.getAutoMeshOp(tmpSim, volumeMeshOpName));
    }catch(NeoException e){
      tmpSim.println("Did not find SM or VM operations.");
    }
  }

  // Make domains aware of proxy representation.
  returnWTDomains.forEach(tmpWTDomain -> {
    tmpWTDomain.setProxyRepresentations(proxyRep);
  });
  
  // Make sure analysis knows of any new domains created.
  tmpSim.println("New domains: "+returnWTDomains.size());
  for(WindTunnelDomain wtDomain : returnWTDomains){
    if(! allWTDomains.contains(wtDomain)){
      allWTDomains.add(wtDomain);
    }
  }

  tmpSim.println("Size of allWTDomains" + allWTDomains.size());
  return returnWTDomains;
}
public ArrayList<OversetDomain> instantiateExistingOversetDomainObjects(
    Simulation tmpSim){
  //===============================
  // Instantiate OversetDomains.
  //===============================
  // Check that the simulation file does not contain a previously set
  // up case that is being run for the first time and just needs to rebuild
  // its objects. If the case is ready to go, there should be regions.
  ArrayList<OversetDomain> returnOversetDomains = new ArrayList();
  if(verbosity > 0){
    tmpSim.println("WT MSG: Instantiating OversetDomain Objects...");
    tmpSim.println("WT MSG: ... checking for Overset Regions.");
  }
  returnOversetDomains.addAll(instantiateOversetDomainsFromRegions(tmpSim));
  // First, check that the simulation file does not contain an already set
  // up case that is being run for the first time and just needs to rebuild
  // its objects. If the case is ready to go, there should be regions.
  if(returnOversetDomains.isEmpty()){
    if(verbosity > 1){
    tmpSim.println("WT MSG: ... checking for Overset GeometryParts.");
    }
    returnOversetDomains.addAll(
        instantiateOversetDomainsFromGeometryParts(tmpSim)
        );
  }
  return returnOversetDomains;
}

// WIND TUNNEL REFERENCE VALUES
// REFERENCE VALUES
private void setRefRho(double refVal){
    refRho=refVal;
}
private void setRefVel(double refVal){
    refVel=refVal;
}
private void setRefTemp(double refVal){
    refTemp=refVal;
}
private void setRefMu(double refVal){
    refMu=refVal;
}
private void setRefMach(double refVal){
    refMa=refVal;
}
private void setRefRe(double refVal){
    refRe=refVal;
}
public void setGlobalReferenceAreaVariable(double refVal){
  this.refArea = refVal;
}
public void setGlobalReferenceLengthVariable(double refVal){
  this.chordLength = refVal;
}
public void updateAllReferenceValues(
    Simulation tmpSim, double refRho,double refVel, double refTemp,
    double refMu, double refMa,double refRe,
    ArrayList<String> wtTagNames,
    ArrayList<Double> wtCustReferenceAreaArray,
    ArrayList<double[]> wtCustReferenceMomRDoubleArray){

  if(verbosity > 0){
    tmpSim.println("WT MSG: All reference values are being updated.");
  }
  
  // Wind Tunnel Analysis will also keep track of updated reference values.
  setRefRho(  refRho);
  setRefVel(  refVel);
  setRefTemp(refTemp);
  setRefMu(    refMu);
  setRefMach(  refMa);
  setRefRe(    refRe);
  
  for(WindTunnelDomain wtDomain : allWTDomains){
    int index = wtTagNames.indexOf(
        wtDomain.getPrimaryDomainTag().getPresentationName());
    double localRefArea;
    double[] localRefMomR;
    if(index < 0){
      localRefArea = refArea;
      localRefMomR = new double[] {chordLength, chordLength, chordLength};
    }else{
      localRefArea = wtCustReferenceAreaArray.get(index);
      localRefMomR = wtCustReferenceMomRDoubleArray.get(index);
    }

    if(verbosity > 0){
      tmpSim.println("Updating reference values.");
    }
    wtDomain.setRefRho( refRho);
    wtDomain.setRefVel( refVel);
    wtDomain.setRefTemp(refTemp);
    wtDomain.setRefMu(  refMu);
    wtDomain.setRefMa(  refMa);
    wtDomain.setRefRe(  refRe);
    wtDomain.getChordLengthFromParameters(tmpSim);
    wtDomain.setReferenceArea(tmpSim, localRefArea);
    wtDomain.setMomentReferenceVector(tmpSim, localRefMomR);

    // Update Physics
    if(verbosity > 0){
      tmpSim.println("Updating physics.");
    }
    CFD_Physics.updateMu(wtDomain.getPhysicsContinuum(), refMu);
    
    // Each wind tunnel tracks its information about angle of attack.
    if(verbosity > 0){
      tmpSim.println("WT Alpha.");
    }
    wtDomain.setWTAlphaAngle(tmpSim);

    // TODO: Create and update field functions for each wind tunnel
    // domain after the characteristic base length can change per wind tunnel.
    // pressure coefficient
    if(verbosity > 0){
      tmpSim.println("Pressure coef.");
    }
    SimTool.setPressureCoefficientFF(tmpSim,refRho,0.0,refVel);

    // skin friction coefficient
    if(verbosity > 0){
      tmpSim.println("Skin fric coef.");
    }
    SimTool.setSkinFrictionCoefficientFF(tmpSim,refRho,refVel);
    
    //====================================
    // Object specific reference updates.
    //====================================
    if(verbosity > 0){
      tmpSim.println("Updating airfoil reference values.");
    }
    // Airfoils
    wtDomain.updateAirfoilReferenceValues(tmpSim);

    // Boundary Layer Probes
    if(verbosity > 0){
      tmpSim.println("Updating BLP values.");
    }
    wtDomain.updateBLProbes(tmpSim, refRho, refMu / refRho);

    if(verbosity > 0){
      tmpSim.println("WT MSG: All reference values were updated.");
    }
  }
}
// MESHING
public void updateWTCustomChordLengths(Simulation tmpSim, 
  ArrayList<String> wtTagNames, ArrayList<Double> newChordLengthsSizes){
  if(wtTagNames.size() != allWTDomains.size() || 
      newChordLengthsSizes.size() != allWTDomains.size()){
    throw new NeoException("WT ERR: New chord length input arraylist size"
        + "does not match number of wind tunnel domains!\n"
        + "Size of WT Domains: " + allWTDomains.size() + "\n"
        + "Size of wtTagNames: " + wtTagNames.size() + "\n"
        + "Size of newChordLengthsSizes: "+newChordLengthsSizes.size()+"\n");
  }
  for(WindTunnelDomain wtDomain: allWTDomains){
    int index = wtTagNames.indexOf(
        wtDomain.getPrimaryDomainTag().getPresentationName());
    if(index > -1){
      wtDomain.setChordLength(tmpSim, newChordLengthsSizes.get(index));
    }
  }
}

public void updateWTCustomMeshBaseSizes(Simulation tmpSim, 
    ArrayList<String> wtTagNames, ArrayList<Double> newChordLengthsSizes){
  if(wtTagNames.size() != allWTDomains.size() || 
      newChordLengthsSizes.size() != allWTDomains.size()){
    throw new NeoException("WT ERR: New base size input arraylist size"
        + "does not match number of wind tunnel domains!");
  }
  for(WindTunnelDomain wtDomain: allWTDomains){
    int index = wtTagNames.indexOf(
        wtDomain.getPrimaryDomainTag().getPresentationName());
    if(index > -1){
      double newBaseSize = newChordLengthsSizes.get(index);
      wtDomain.getDomainSurfMeshOp().getDefaultValues().get(BaseSize.class)
          .setValue(newBaseSize / 10.0);
      wtDomain.getDomainVolumeMeshOp().getDefaultValues().get(BaseSize.class)
          .setValue(newBaseSize / 10.0);
    }
  }
  
}
public boolean checkMeshOperationsUpToDate(Simulation tmpSim){
  boolean upToDate=true;
  for(MeshOperation tmpOp:tmpSim.get(MeshOperationManager.class).getObjects()){
    if(tmpOp instanceof AutoMeshOperation){
      upToDate = checkVolumeMeshUpToDate(tmpSim,tmpOp.getPresentationName());
      return upToDate;
    }
  }
  return upToDate;
}
private boolean checkVolumeMeshUpToDate(Simulation tmpSim, String opName){
  boolean retVal = !((AutoMeshOperation) tmpSim.get(
          MeshOperationManager.class).getObject(opName)).isDirty();
  return retVal;
}
private boolean checkMeshOpUpToDate(Simulation tmpSim, String opName){
  boolean retVal = !((AutoMeshOperation) tmpSim.get(
          MeshOperationManager.class).getObject(opName)).isDirty();
  return retVal;
}
public void generateSurfaceMesh(Simulation tmpSim){
  // Only remesh the Surface Mesh Operations that are out of date.
  for(WindTunnelDomain wtDomain : allWTDomains){
    AutoMeshOperation tmpOp = wtDomain.getDomainSurfMeshOp();
    if(!checkMeshOpUpToDate(tmpSim, tmpOp.getPresentationName())){
      tmpSim.println(
              "domain " +wtDomain.getName()+" surf op not up to date");
      tmpOp.execute();
    }
    for(OversetDomain osDomain : wtDomain.getAllOversetDomains()){
      AutoMeshOperation osTmpOp = wtDomain.getDomainSurfMeshOp();
      if(!checkMeshOpUpToDate(tmpSim, osTmpOp.getPresentationName())){
        tmpSim.println(
                "OS domain " +osDomain.getName()+" surf op not up to date");
        tmpOp.execute();
      }
    }
  }
}
public void generateVolumeMesh(Simulation tmpSim){
  // Only remesh the Volume Mesh if it is out of date.
  for(WindTunnelDomain wtDomain : allWTDomains){
    AutoMeshOperation tmpOp = wtDomain.getDomainVolumeMeshOp();
    if(!checkMeshOpUpToDate(tmpSim, tmpOp.getPresentationName())){
      tmpSim.println(
              "domain " +wtDomain.getName()+" vol mesh not up to date");
      tmpOp.execute();
    }
    for(OversetDomain osDomain : wtDomain.getAllOversetDomains()){
      AutoMeshOperation osTmpOp = wtDomain.getDomainVolumeMeshOp();
      if(!checkMeshOpUpToDate(tmpSim, osTmpOp.getPresentationName())){
        tmpSim.println(
                "OS domain " +osDomain.getName()+" surf op not up to date");
        tmpOp.execute();
      }
    }
  }
  

  Representation volMesh = 
          tmpSim.getRepresentationManager().getObject("Volume Mesh");
  SimTool.getSimProxy(
          tmpSim,globalNames.getProxyName()).setRepresentation(volMesh);

}
public void executeAllMeshModels(Simulation tmpSim){
  tmpSim.getMeshPipelineController().generateVolumeMesh();
  Representation volMesh =
      tmpSim.getRepresentationManager().getObject("Volume Mesh");
  SimTool.getSimProxy(tmpSim, globalNames.getProxyName())
      .setRepresentation(volMesh);
}

// REGIONS
public void applyBoundaryConditions(double refVel,double refTemp,
                                    double refMa,double fsPress){

  for(WindTunnelDomain mainDomain : allWTDomains){
    if(!mainDomain.isFreeStreamDomain()){
      mainDomain.setAllInletBCs(refVel, refTemp, refMa, fsTVR, fsTi);
      mainDomain.setAllOutletBCs(fsPress, refTemp, fsTVR, fsTi);
    }else{
      double fsTkeValue = 1.0E-6 * refVel * refVel;
      double fsOmegaValue = 5.0 * refVel / chordLength;
      //
      mainDomain.setFSBoundariesToKAndOmega(fsTkeValue, fsOmegaValue);
       // Boundary condition changes to fsTkeValue OmegaValue.
      mainDomain.setAllInletBCs(refVel, refTemp, refMa, fsTkeValue, fsOmegaValue);
    }
  }
}

// OBJECTS
public void rotateBoundaryLayerProbeCADParts(Simulation tmpSim,double newAngleInDeg){
  // because all low speed wind tunnel cases are overset and all high speed are 
  // fixed with a remesh, we only rotate in the presence of overset domains.
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.rotateBoundaryLayerProbeCADParts(tmpSim, newAngleInDeg);
  }
}
public void implement2DObjectsIntoScenes(Simulation tmpSim){
  // for 2D, we want to display the fields from the all region boundaries
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.implement2DObjectsIntoScenes(tmpSim);
  }
}
// PHYSICS
public void initPhysics(Simulation tmpSim, String physName,String physModel){
  tmpSim.println("WT MSG: Initializing physics continuum.");

  for(WindTunnelDomain wtDomain : allWTDomains){
    Tag wtTag = wtDomain.getPrimaryDomainTag();
    String wtTagName = wtTag.getPresentationName();

    // Figure out if there is a CFD_Physics associated with this wind tunnel.
    if( !wtDomain.isPhysicsSet() ){
      wtDomain.setPhysics(new CFD_Physics(tmpSim, wtTagName + "." + physName));
    }
    CFD_Physics wtCFDPhysics = wtDomain.getPhysics();
    PhysicsContinuum wtPhysicsContinuum = wtDomain.getPhysicsContinuum();
    TagTool.addTags(tmpSim, wtPhysicsContinuum, Collections.singleton(wtTag));

    // Detect whether this is a high Mach number wind tunnel.
    boolean useSegregated=true;
    if(refMa > highMachNumber){
      useSegregated=false;
    }

    // Set best practice Reference Values.
    double minWallDistance = 1.0E-7;
    // This is a default best practice. 
    wtCFDPhysics.setWallDistanceToFreeStream(0.05*chordLength);
    double wallDistToFS = wtCFDPhysics.getWallDistanceToFreeStream();

    //Physics model selection
    if(physModel.equals("kwSST")){
        tmpSim.println("WT MSG: ... kwSST Model Selected.");
        isUns=false;
        CFD_Physics.set_RANS_KwSST(wtPhysicsContinuum, !is2D, useSegregated,
                                   true, false, false, wallDistToFS);

        //Physics Solver Best Practice Settings
        CFD_Physics.setKWSSTURF(tmpSim,0.6);
        CFD_Physics.setKWSSTViscosity(tmpSim,0.8,1.0E20);
        CFD_Physics.setMinimumReferenceWallDistance(
            wtPhysicsContinuum, minWallDistance);

    }else if(physModel.equals("kwSSTGRT")){
        tmpSim.println("WT MSG: ... kwSST GRT Model Selected.");
        isUns=false;
        CFD_Physics.set_RANS_KwSST(wtPhysicsContinuum, !is2D, useSegregated,
                                   true, false, true, wallDistToFS);

        // This is unique situation since 2D airfoils may not have the same
        // chord length.
        if(wtDomain.is2D()){
          String grtID = wtDomain.getPrimaryDomainTag().getPresentationName();
          CFD_Physics.set_GRT_FF(tmpSim, wtPhysicsContinuum,
              "GammaReTheta Freestream Function - " + grtID, "grt_" + grtID,
              0.05 * wtDomain.getChordLengthFromParameters(tmpSim));
        }

        //Physics Solver Best Practice Settings
        CFD_Physics.setKWSSTURF(tmpSim, 0.6);
        CFD_Physics.setKWSSTViscosity(tmpSim, 0.8, 1.0E20);
        CFD_Physics.setMinimumReferenceWallDistance(
            wtPhysicsContinuum, minWallDistance);

    }else if(physModel.equals("kwSSTGRTDES")){
        tmpSim.println("WT MSG: ... kwSST GRT DES Model Selected.");
        isUns=true;
        CFD_Physics.set_DES_KwSST(wtPhysicsContinuum, !is2D, useSegregated,
                                  true, false, true, wallDistToFS);
        //set up time step information
        SolverDriver.set2ndOrderTimeDisc(tmpSim);
        CFD_Physics.setMinimumReferenceWallDistance(
            wtPhysicsContinuum, minWallDistance);

    }else if(physModel.equals("kwSST-3EqSGamma")){
        tmpSim.println("WT MSG: ... kwSST 3 Eqn. S Gamma Model Selected.");
        isUns=false;
        CFD_Physics.set_RANS_KwSST_SGamma(wtPhysicsContinuum, !is2D,
                                          useSegregated, true, false);
        //Physics Solver Best Practice Settings
        CFD_Physics.setKWSSTURF(tmpSim,0.6);
        CFD_Physics.setKWSSTViscosity(tmpSim,0.8,1.0E20);
        CFD_Physics.setMinimumReferenceWallDistance(
            wtPhysicsContinuum, minWallDistance);

    }else if(physModel.equals("kwSST-3EqSGamma-DES")){
        tmpSim.println("WT MSG: ... kwSST 3 Eqn. S Gamma DES Model Selected.");
        isUns=true;
        CFD_Physics.set_DES_KwSST_SGamma(wtPhysicsContinuum, !is2D,
                                         useSegregated, true, false);
        //set up time step information
        SolverDriver.set2ndOrderTimeDisc(tmpSim);
        CFD_Physics.setMinimumReferenceWallDistance(
            wtPhysicsContinuum, minWallDistance);

    }else{
      tmpSim.print("WT MSG: No matching physics model found in initPhysics!");
    }

    //Apply to all applicable regions
    for(Region tmpReg : wtDomain.getRegionsOfControlledObjects()){
      tmpReg.setPhysicsContinuum(wtPhysicsContinuum);
    }
  }
}
public void setHighMachNumber(double newHighMachNumber){
    highMachNumber = newHighMachNumber;
  }
public void highMachAdjustment(){
  double cflNumber = 1000.0;
  double ccaURF = 0.1;
  int ccaFreq = 10;

  int startRampIts = 100;
  int endRampIts = 500;

  double turbSolverURF=0.75;
  double turbViscositySolverURF=1.0;

  // High Mach Turbulence settings
  double newA1 = 0.31;

  for(WindTunnelDomain wtDomain : allWTDomains){
    PhysicsContinuum tmpPhys = wtDomain.getPhysicsContinuum();
    Simulation tmpSim = tmpPhys.getSimulation();

    // Turn on Coupled Solver w/ AUSM+
    CoupledImplicitSolver coupledSolver =
        CFD_Physics.switchToCoupledSolver(tmpPhys,cflNumber);
    CFD_Physics.useAUSMPlus(tmpPhys);

    // Turbulence solver modifications
    CFD_Physics.setKWTurbSolverURF(tmpSim, turbSolverURF);
    CFD_Physics.setBLTurbulenceInitialization(tmpSim,true);
    CFD_Physics.setKWTurbViscositySolverURF(tmpSim, turbViscositySolverURF);

    // Expert Driver Settings
    ExpertDriverCoupledSolver expertDriver =
            CFD_Physics.getExpertDriver(coupledSolver);
    expertDriver.setEndIteration(endRampIts);
    expertDriver.setAmgNrCyclesForCflLimiting(8);

    //Swap all mainWTdomain boundaries to be freestream boundaries
    wtDomain.switchInAndOutBCToFreeStream(refMa,refTemp,fsTVR,fsTi);
    isFS=wtDomain.hasFSBoundaries();

    //Modify kwSST turbulence A1 value
    CFD_Physics.setKWSSTa1Coef(tmpPhys,newA1);
  }
  

}
public void createOversetInterfaces(Simulation tmpSim){
  /* Method createOversetInterfaces causes all WindTunnelDomains to create their
     required OversetDomains. Note that this assumes that Physics continuua
     already exists.
  */
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.createOversetInterfaces(tmpSim);
  }
}
public void setInitialConditions(Simulation tmpSim){
  /* Method setInitialConditions causes all WindTunnelDomains to set their
     required physics continuum initial conditions.
  */
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.setInitialConditions(tmpSim);
  }  
}
public void instantiateExistingPhysics(Simulation tmpSim, String physicsName){
  for(WindTunnelDomain wtDomain : allWTDomains){
    if(wtDomain.getPhysics() == null){
      wtDomain.setPhysics(new CFD_Physics(tmpSim, 
          wtDomain.getPrimaryDomainTag().
                  getPresentationName()+"."+physicsName));
    }
  }
}
public void modifyAirfoilSetAngles(Simulation tmpSim, double[] newAngles){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.modifyAirfoilSetAngles(tmpSim, newAngles);

  }
}
public void setAirfoilSurfaceMeshSettings(ArrayList<Boolean> af_CustomMesh,
  ArrayList<Double>  af_TargetSize, ArrayList<Double> af_MinSize,
  ArrayList<Double> af_NPtsOnCircle){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.setAirfoilSurfaceMeshSettings(af_CustomMesh,
        af_TargetSize, af_MinSize, af_NPtsOnCircle);
  }
}
public void setAirfoilVolumeMeshSettings(Simulation tmpSim,
  ArrayList<Boolean> af_CustomMesh,
  ArrayList<Boolean> af_LowYPlus,ArrayList<Boolean> af_HighYPlus,
  ArrayList<Double>  af_PrismAbsThick, ArrayList<Double> af_1stCellThick,
  ArrayList<Integer> af_NPrisms){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.setAirfoilVolumeMeshSettings(tmpSim, af_CustomMesh, af_LowYPlus,
        af_HighYPlus, af_PrismAbsThick, af_1stCellThick, af_NPrisms);
  }
}
public void modifyMeshAngleOfAttack(Simulation tmpSim, double newAngle){
  //If there is a turn table angle of the assembly that is being rotated
  // we also need to rotate BodyCsys at the same angle
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.modifyMeshAngleOfAttack(tmpSim, newAngle);
  }
}
public double getWindTunnelAngleOfAttack(){
  double alphaOfMesh = 0.0;
  for(WindTunnelDomain wtDomain : allWTDomains){
    alphaOfMesh = wtDomain.getAlphaAngle();
  }
  return alphaOfMesh;
}
public void modifyOversetAngleOfAttack(Simulation tmpSim, double newAngle){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.modifyOversetAngleOfAttack(tmpSim, newAngle);
  }
}
public void createUnsteadyObjectReporting(Simulation tmpSim){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.createUnsteadyObjectReporting(tmpSim);
  }
}

// SOLVER
public void clearSolution(Simulation tmpSim){
  tmpSim.clearSolution();
}

// Iteration Based
public static void setInnerIterations(Simulation tmpSim, int newInnerIterations){
    SolverDriver.setInnerIts(tmpSim, newInnerIterations);
}
public static void deactivateMaxStepsCriteria(Simulation tmpSim){
  SolverDriver.deactivateMaxStepsCriteria(tmpSim);
}
// Unsteady
public boolean isUnsteady(){
    return isUns;
}
public static double setTimeStep(Simulation tmpSim, double newTimeStep){
    SolverDriver.setTimeStep(tmpSim,newTimeStep);
    return newTimeStep;
}
public static void setFinalSolutionTime(Simulation tmpSim, double newValue){
  SolverDriver.setFinalSolutionTime(tmpSim, newValue);
}
public double getRecommendedTimeStep(double manualTimeStep,
                                     double nDiscretizations){
  double minPeriod = 1.0E37;
  double airfoilTimeScale = 1.0E37; //[s]
  double minTimeStep = manualTimeStep;
  for(WindTunnelDomain wtDomain : allWTDomains){
    // Gather all airfoil time scales.
    if(wtDomain.getAllAirfoilObjects().size() > 0){
      airfoilTimeScale = wtDomain.getChordLength()
          / (wtDomain.getRefVel() + TINY_VALUE);
      if(airfoilTimeScale < minPeriod){
        minPeriod = airfoilTimeScale;
      }
    }
  }
  NumberFormat formattedDouble = new DecimalFormat("0.#E0");
  String truncatedTimeStep=formattedDouble.format(minPeriod/nDiscretizations);
  return Double.parseDouble(truncatedTimeStep);
}
public double getRecommendedTotalRunTime(){
  /* Returns the recommended final solution time based on 10 periodic
     flow overs, i.e.: Lengthscale/Reference Velocity
  */
  double recommendedTime = 0.0;
  for(WindTunnelDomain wtDomain : allWTDomains){
    //AIRFOILS
    if(wtDomain.getAllAirfoilObjects().size() > 0){
      recommendedTime = 10.0 * wtDomain.getChordLength()
          / (wtDomain.getRefVel() + TINY_VALUE);
    }
  }
  NumberFormat formattedDouble = new DecimalFormat("0.#E0");
  String truncatedTimeStep=formattedDouble.format(recommendedTime);
  return Double.parseDouble(truncatedTimeStep);
}
public void turnOnAutoTimeStep(){
  solverDriver.turnOnAutoTimeStep();
}

//POST PROCESSING
public void setPostProc(Simulation tmpSim){
  //========================
  // Universal Coefficients
  //========================
  // pressure coefficient
  SimTool.setPressureCoefficientFF(tmpSim, refRho, 0.0, refVel);
  // skin friction coefficient
  SimTool.setSkinFrictionCoefficientFF(tmpSim, refRho, refVel);

  //====================
  // ANNOTATIONS
  //====================
  initAnnotations(tmpSim);
  
  //==========================
  // SPECIFIC POST PROCESSING
  //==========================
  
  for(WindTunnelDomain wtDomain : allWTDomains){
    if(wtDomain.is2D()){
      tmpSim.println("Setting 2D view positions");
      wtDomain.set2DWTViewPositions(tmpSim);
      wtDomain.init2DPostProcessing(tmpSim);
    }else{
      wtDomain.init3DPostProcessing(tmpSim);
    }
  }

  //====================
  // MONITORS
  //====================
  // Clean monitor output
  ArrayList<Monitor> monitorOutput = new ArrayList();
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Continuity"));
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("X-momentum"));
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Y-momentum"));
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Energy"));
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Tke"));
  monitorOutput.add(tmpSim.getMonitorManager().getMonitor("Sdr"));
  tmpSim.getMonitorManager().setPrintedMonitors(monitorOutput);
  monitorOutput.clear();

  //====================
  // GLOBAL PLOTS
  //====================
  // Adjust residual plots
  PlotTool.adjustResidualPlot(tmpSim, is2D);

}

public String makeSheetsCSVData(Simulation tmpSim, String studyName,
                                String cfdVer, String javaVer){
  /* Each WindTunnelDomain will output its own CSV row.
  
  */
  if(verbosity > 0){
    tmpSim.println("WT MSG: Post processing CFD data to CSV values...");
    tmpSim.println("  Number of Wind Tunnel Domains to process: "
        +allWTDomains.size());
  }

  String retString = "";
  for(WindTunnelDomain wtDomain : allWTDomains){
    if(verbosity > 1){
      tmpSim.println("... simulation stats.");      
    }

    //CFD Simulation Stats
    // Note the wind tunnel domain tag name is prepended to the study name
    // for sorting later.
    if(verbosity > 1){
      tmpSim.println("WT MSG: Post processing simulation state...");
    }
    String wtTagName = wtDomain.getPrimaryDomainTag().getPresentationName();
    retString = retString + wtTagName + "." 
            + studyName + "," + cfdVer + "," + javaVer;
    
    // Number of iterations.
    int currentItLvl = SolverDriver.getCurrentIterationLevel(tmpSim);
    retString = retString + "," + currentItLvl;

    // Number of steps.
    int currentStepLvl = SolverDriver.getCurrentStepLevel(tmpSim);
    retString = retString + "," + currentStepLvl;
    
    // Time step (if applicable).
    if(CFD_Physics.isUnsteady(wtDomain.getPhysicsContinuum())){
      retString = retString + "," + SolverDriver.getTimeStep(tmpSim);
    }else{
      retString = retString + "," + "N/A";
    }
    // Solution Time (if applicable).
    if(CFD_Physics.isUnsteady(wtDomain.getPhysicsContinuum())){
      retString = retString + "," + SolverDriver.getCurrentTime(tmpSim);
    }else{
      retString = retString + "," + "N/A";
    }

    // Stopping Criteria Satisfied.
    retString = retString + "," + "N/A";

    // Physics
    // 2d or 3d
    if(CFD_Physics.is2D(wtDomain.getPhysicsContinuum())){
        retString = retString + "," + "2D";
    }else{
        retString = retString + "," + "3D";
    }
    //fs or wt
    if(wtDomain.isFreeStreamDomain()){
        retString = retString + "," + "Free Stream";
    }else{
        retString = retString + "," + "Tunnel";
    }
    //steady or unsteady
    if(CFD_Physics.isUnsteady(wtDomain.getPhysicsContinuum())){
        retString = retString + "," + "Unsteady";
    }else{
        retString = retString + "," + "Steady";
    }
    //turbulence model
    retString = retString + "," + CFD_Physics
        .getTurbulenceModelName(wtDomain.getPhysicsContinuum());

    // Nominal reference chord length Reynolds number and Mach number
    retString = retString + "," + chordLength;
    retString = retString + "," + refRe;
    retString = retString + "," + refMa;

    retString = retString + wtDomain.getSheetsCSVData(tmpSim);
    retString = retString + "\n";
  }
  return retString;
}

public String getFileString(Simulation tmpSim){
  /* There are two workflows for getting the file string. 
     Workflow 1: Only one wind tunnel exists in the simulation. The file
                 string is going to be strictly related to the alpha values of
                 the local wind tunnel case, the turbulence model,
                 the name of the Study Folder, the nominal Reynolds number,
                 and the angle of attack.
     Workflow 2: Multiple wind tunnels exist in the simulation. The file
                 string is going to be the name of the StudyFolder, the
                 indicator of "Database", the database *speed*, and the common
                 alpha value of all wind tunnel articles.
  */
    // Formatting
    NumberFormat formatbig   = new DecimalFormat( "0.##E0" );
    NumberFormat formatsmall = new DecimalFormat( "0.##" );
    
    String retStr = "";

    // Physics
    // Assumption is that all WT regions maintain the same physics per alpha
    retStr=retStr + CFD_Physics
        .getTurbulenceModelName(allWTDomains.get(0).
                                getPhysics().getContinuum());

    // Nominal Reynolds number for one windtunnel. Also include speed.
    retStr += "_Re";
    retStr += formatbig.format(refRe);
    retStr=retStr + "_Speed";
    retStr=retStr + formatbig.format(refVel);

    //Alpha
    retStr=retStr + "_A";
    retStr=retStr + formatsmall.format( getWindTunnelAngleOfAttack() );      

    return retStr;
  }
public void setCaseAnnotationName(String studyName,String caseName){
  caseAnnotationName=studyName+": "+caseName;
}
public void initAnnotations(Simulation tmpSim){
  Annotation tmpAnn;
  //annotations
  String fileSep =File.separator;

  //Keep the order of the deck correct
  standardAnnotations.clear();

  IterationAnnotation iterationAnnotation = 
      ((IterationAnnotation) tmpSim.getAnnotationManager().
              getObject("Iteration"));
  iterationAnnotation.setShowTimeStep(true);
  iterationAnnotation.setShowPhysicalTime(true);
  iterationAnnotation.setDefaultHeight(0.15);
  iterationAnnotation.setDefaultPosition(
          new DoubleVector(new double[] {0.660, 0.825, 0.}));
  standardAnnotations.add(iterationAnnotation);
  tmpAnn = caseNameAnnotation(tmpSim,caseAnnotationName);
  tmpAnn.setBackground(false);
  standardAnnotations.add(tmpAnn);

  //Special one-offs
  whiteBackgroundCaseNameAnnotation(tmpSim,caseAnnotationName);
  
  // Wind Tunnel Domains are responsible for their own annotations.
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.initAnnotations(tmpSim, standardAnnotations);
  }
}

// METHODS FOR OPTIMIZER SWEEPS
public void setUpAlphaSweepMetrics(
  Simulation tmpSim, double[] alphaSweepAngles){
    for(WindTunnelDomain tmpWTDomain : allWTDomains){
      tmpWTDomain.setUpAlphaSweepMetrics(tmpSim, alphaSweepAngles);
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
}
public void completeSweepMetricValues(
  Simulation tmpSim, double thisAngle){
  for(WindTunnelDomain tmpWTDomain : allWTDomains){
    tmpWTDomain.completeSweepMetricValues(tmpSim, thisAngle);
  }
}

//=======================
// PRIVATE METHODS
//=======================
// WIND TUNNEL ANALYSIS DOMAIN INSTANTIATION
private OversetDomain createOversetDomain(Simulation tmpSim,
    GeometryPart osPart){
    OversetDomain osDomain = new OversetDomain(tmpSim,osPart);
    tmpSim.println("WT MSG: Setting up overset region from Domain "
        + osPart.getPresentationName());

    // Get the appropriate wind tunnel tag for this domain
    ArrayList<Tag> allGeomPartTags = TagTool.getObjectTagsByPrefix(
        osPart,globalNames.getWindTunnelTagID()
        );
    if(allGeomPartTags.size()> 1){
      throw new NeoException("OS ERR: Too Many WT Tags on this Part.");
    }else{
      osDomain.setPrimaryDomainTag(allGeomPartTags.get(0));
    }
    String primaryWTTagName = 
        osDomain.getPrimaryDomainTag().getPresentationName();

    osDomain.setUpRegion(tmpSim,
        primaryWTTagName + "." + osPart.getPresentationName());
    tmpSim.println("WT MSG: Setting up overset region boundaries.");
    osDomain.initPartBoundaries();
    Region osRegion = osDomain.getRegion();
    allWTRegs.add(osRegion);
    return osDomain;
}
private ArrayList<WindTunnelDomain> instantiateWTDomainsFromRegions(
    Simulation tmpSim){
  /* Figures out what existing WindTunnelDomain objects may already exist
     in a previously preprocessed simulation and instantiates their objects if
     applicable.
  */
  ArrayList<Region> mainWTRegions;
  ArrayList<WindTunnelDomain> returnDomains = new ArrayList();

  // Get Regions in the simulation and search for any wind tunnels and
  // get each respective part to build a WindTunnelDomain. We build the
  // WindTunnelDomain objects off of a Region's GeometryPart.
  Collection<Region> allRegions = getRegions(tmpSim);
  mainWTRegions = getMainWTRegions(allRegions);
  for(Region wtRegion : mainWTRegions){
    // WindTunnel Domains are build through GeometryPart associated with the
    // Region. If there is no GeometryPart, we cannot build the WindTunnel
    // Domain object.
    if(!getRegionPart(wtRegion).isEmpty()){
      GeometryPart wtPart = getRegionPart(wtRegion);

      // Figure out how many Wind tunnel domain tags are tagged on the
      // specific GeometryPart. If there is more than one, this is likely a
      // user input error.
      ArrayList<Tag> wtTags = TagTool.getObjectTagsByPrefix(wtPart,
          globalNames.getWindTunnelTagID()
          );
      if(wtTags.size() > 1){
        throw new NeoException("WT ERR: Cannot have more than one wind" +
           " tunnel tag per GeometryPart. On Part: " + wtPart + ".");
      }else if(wtTags.isEmpty()){
        // Do not account for backward compatibility. If it is a CAD based
        // part, force the user to fix the original CAD model and retransfer
        // into the simulation. Otherwise, force the user to go back and
        // manually tag the part.
        throw new NeoException("WT ERR: Must have at least one wind tunnel" +
            " tag. On Part: " + wtPart + ".");
      }
      // There had better be only one wind tunnel tag in this array at this
      // point in the process.
      Tag thisWTPartTag = wtTags.get(0);

      // Instantiate Wind Tunnel Domain object itself.
      WindTunnelDomain wtDomain = new WindTunnelDomain(tmpSim, wtPart);
      wtDomain.setPrimaryDomainTag(thisWTPartTag);

      // Set up wind tunnel coordinate systems.
      String domainTagName = 
          wtDomain.getPrimaryDomainTag().getPresentationName();
      CartesianCoordinateSystem wtCsys = 
          SimTool.getLabBasedCoordinate(tmpSim, domainTagName);
      wtDomain.setWTCsys(wtCsys);
      wtDomain.setBodyCsys(
          SimTool.getNestedCoordinate(
              wtCsys, globalNames.getBodyCsysName()
              )
          );

      CartesianCoordinateSystem wtInletCsys = SimTool.getNestedCoordinate(
              wtCsys, globalNames.getInletCsysName());
      TagTool.addTag(tmpSim, wtInletCsys, thisWTPartTag);
      wtDomain.setInletCsys(wtInletCsys);

      // Fill up the region object
      wtDomain.setUpRegion(tmpSim,
          domainTagName + "." + wtPart.getPresentationName());
      wtDomain.getExistingDomainBoundaries();

      // Figure out type of domain
      wtDomain.setIsFS(wtDomain.hasFSBoundaries());

      if(verbosity > 0){
        tmpSim.println("WT MSG: Domain " + wtDomain.getName()+" instantiated.");
        tmpSim.println("        Region: " 
                + wtDomain.getRegion().getPresentationName());
      }


      returnDomains.add(wtDomain);
    }else{
      throw new NeoException("WT MSG: Regions do exist but"
          + " no compatable WindTunnel Domain Region detected.");
    }
  }
  return returnDomains;
}
public int getNumberOfWTDomains(){
  return allWTDomains.size();
}
private ArrayList<WindTunnelDomain> instantiateWTDomainsFromGeometryParts(
    Simulation tmpSim){
  /* Figures out what existing WindTunnelDomain objects may exist from 
     the GeometryParts that exist in the simulation.
  */
  ArrayList<WindTunnelDomain> returnedWTDomains = new ArrayList();
  if(verbosity > 0){
    tmpSim.println("WT MSG: Looking for WTDomains from GeometryParts.");
  }

  // Find candidate GeometryParts for WindTunnelDomainParts.
  ArrayList<String> domainIdent = new ArrayList();
  domainIdent.add(globalNames.getInletStr()); // inlet
  domainIdent.add(globalNames.getFreeStr()); // freestream

  ArrayList<GeometryPart> wtDomainPartList =
      MeshOpTool.findGeometryParts(tmpSim, allGeomParts, domainIdent);

  for(GeometryPart domainPart : wtDomainPartList){
    String partName = domainPart.getPresentationName();
    if(verbosity > 1){
      tmpSim.println("WT MSG: Looking at GeometryPart: " + partName);
    }
    WindTunnelDomain tmpWTDomain = new WindTunnelDomain(tmpSim, domainPart);

    // Get the appropriate wind tunnel tag for this domain
    ArrayList<Tag> allGeomPartTags = TagTool.getObjectTagsByPrefix(
        domainPart, globalNames.getWindTunnelTagID()
        );
    if(allGeomPartTags.size()> 1){
      throw new NeoException("WT ERR: Too Many WT Tags on this Part.");
    }else{
      tmpWTDomain.setPrimaryDomainTag(allGeomPartTags.get(0));
    }

    // Get the WindTunnelDomain's primary coordinate system, i.e.
    // the one it is supposed to rotate about for wind tunnel angles.
    CartesianCoordinateSystem wtCsys = 
        SimTool.getLabBasedCoordinate(
            tmpSim, tmpWTDomain.getPrimaryDomainTag().getPresentationName()
            );
    tmpWTDomain.setWTCsys(wtCsys);
    tmpWTDomain.setBodyCsys(
        SimTool.getNestedCoordinate(
            wtCsys, globalNames.getBodyCsysName()
            )
        );
    tmpWTDomain.setInletCsys(
        SimTool.getNestedCoordinate(
            wtCsys, globalNames.getInletCsysName()
            )
        );
    tmpWTDomain.setUpRegion(tmpSim,
        tmpWTDomain.getPrimaryDomainTag().getPresentationName() + "." 
            + domainPart.getPresentationName());

    Region wtRegion = tmpWTDomain.getRegion();
    TagTool.addTags(tmpSim, wtRegion, 
        Collections.singleton(tmpWTDomain.getPrimaryDomainTag())
        );


    if(verbosity > 1){
      tmpSim.println("WT MSG: Setting up windtunnel boundaries.");
    }
    tmpWTDomain.initPartBoundaries();
    tmpWTDomain.getExistingDomainBoundaries();
    returnedWTDomains.add(tmpWTDomain);
  }
  return returnedWTDomains;
}

private ArrayList<OversetDomain> instantiateOversetDomainsFromRegions(
    Simulation tmpSim){
  // This method collects or creates all the OversetDomains from an existing
  // pre-processed simulation.
  //
  // TODO: Extend the identification away from name based to tags.

  // If Regions exist, see if these regions contain existing OversetDomain
  // objects that were already set up by a previous preprocessor use.
  ArrayList<OversetDomain> returnOversetDomains = new ArrayList();
  for(Region tmpReg : tmpSim.getRegionManager().getObjects()){
    String regName = tmpReg.getPresentationName();
    if(regName.startsWith(globalNames.getWindTunnelTagID()) &&
        regName.contains("." + "OS")){
      tmpSim.println("WT MSG: Region " + regName
          + " should be an OversetDomain...");
      tmpSim.println("WT MSG: ... creating OversetDomain from Region.");

      // Create the overset domain.
      GeometryPart osPart = getRegionPart(tmpReg);
      OversetDomain osDomain = createOversetDomain(tmpSim, osPart);

      // Get the appropriate wind tunnel tag for this domain
      ArrayList<Tag> allGeomPartTags = TagTool.getObjectTagsByPrefix(
          osPart,globalNames.getWindTunnelTagID()
          );
      if(allGeomPartTags.size()> 1){
        throw new NeoException("OS ERR: Too Many WT Tags on this Part.");
      }else{
        osDomain.setPrimaryDomainTag(allGeomPartTags.get(0));
      }
      String domainTagName = 
          osDomain.getPrimaryDomainTag().getPresentationName();
      //Get the OversetDomain local coordinate system & set angle.
      CartesianCoordinateSystem wtBodyCsys = 
          SimTool.getLabBasedCoordinate(tmpSim, domainTagName);

      osDomain.setBodyCoordinateSystem(
          SimTool.getNestedCoordinate(wtBodyCsys,
              globalNames.getBodyCsysName()
              )
          );
      osDomain.setOversetCoordinateSystem(
          SimTool.getNestedCoordinate(wtBodyCsys,
              osPart.getPresentationName()
              )
          );
      osDomain.setInletCoordinateSystem(
          SimTool.getNestedCoordinate(wtBodyCsys,
              globalNames.getInletCsysName()
              )
          );
      osDomain.determineDomainAngle();

      // make sure WT analysis knows this Overset domain exists!
      returnOversetDomains.add(osDomain);

      // Indicate success output window
      tmpSim.println("WT MSG: Overset domain " + osDomain.getName()
          + " was found.");
      tmpSim.println("       Region: "
              + osDomain.getRegion().getPresentationName());
    }
  }
  return returnOversetDomains;
}
private ArrayList<OversetDomain> instantiateOversetDomainsFromGeometryParts(
        Simulation tmpSim){
  // This method collects or creates OversetDomains from GeometryPart objects.
  // The GeometryPart is expected to have a primary WindTunnelDomain tag.
  ArrayList<OversetDomain> returnOversetDomains = new ArrayList();
  
  // We are probably setting up from scratch or there are no Overset
  // Domains.
  for(GeometryPart tmpOSPart:getRawOversetParts(tmpSim)){
    if(verbosity > 0){
      tmpSim.println("WT MSG: instantiateOversetDomainsFromGeometryParts");
      tmpSim.println("WT MSG: GeometryPart " + tmpOSPart.getPresentationName()
          + " should be OversetDomain...");
    }
    //Create domain
    OversetDomain osDomain = createOversetDomain(tmpSim, tmpOSPart);
    
    // Get the appropriate wind tunnel tag for this domain
    ArrayList<Tag> allGeomPartTags = TagTool.getObjectTagsByPrefix(
        tmpOSPart,globalNames.getWindTunnelTagID()
        );
    if(allGeomPartTags.size()> 1){
      throw new NeoException("OS ERR: Too Many WT Tags on this Part.");
    }else{
      osDomain.setPrimaryDomainTag(allGeomPartTags.get(0));
    }

    // Get the OversetDomain's primary coordinate system, i.e.
    // the one it is supposed to rotate about for wind tunnel angles.
    CartesianCoordinateSystem wtBodyCsys = 
        SimTool.getLabBasedCoordinate(
            tmpSim, osDomain.getPrimaryDomainTag().getPresentationName()
            );
    osDomain.setBodyCoordinateSystem(
        SimTool.getNestedCoordinate(wtBodyCsys,
            globalNames.getBodyCsysName()
            )
        );
    osDomain.setOversetCoordinateSystem(
        SimTool.getNestedCoordinate(wtBodyCsys,
            tmpOSPart.getPresentationName()
            )
        );
    osDomain.setInletCoordinateSystem(
        SimTool.getNestedCoordinate(wtBodyCsys,
            globalNames.getInletCsysName()
            )
        );

    Region osRegion = osDomain.getRegion();
    TagTool.addTags(tmpSim, osRegion, 
        Collections.singleton(osDomain.getPrimaryDomainTag())
        );

    // make sure WT analysis knows this Overset domain exists!
    returnOversetDomains.add(osDomain);
  }
  return returnOversetDomains;
}

// REFERENCE VALUES
private double getGlobalAlphaValue(Simulation tmpSim,
    Collection<WindTunnelDomain> windTunnels){
  double oldAlpha = 0.;
  double retAlpha = 0.;
  int counter = 0;
  for(WindTunnelDomain tmpWT: windTunnels){
    if(counter != 0){
      oldAlpha=retAlpha;
    }
    retAlpha = tmpWT.getAlphaAngle();
    if(counter == 0){
      oldAlpha=retAlpha;
    }
    if(Math.abs(retAlpha - oldAlpha) > SMALL_EPS){
      tmpSim.println("WTA WARNING: " + tmpWT.getName() 
              + "alpha is inconsitent.");
    }
    counter++;
  }
  return retAlpha;
}

// GEOMETRY
private ArrayList<GeometryPart> getRawOversetParts(Simulation tmpSim){
  Collection<GeometryPart> rawGeomList = allGeomParts;
  ArrayList<String> oversetIdent = new ArrayList();
  oversetIdent.add(globalNames.getOversetStr()); // oversetprefix
  return MeshOpTool.findGeometryParts(tmpSim, rawGeomList, oversetIdent);
}

// REGIONS
private GeometryPart getRegionPart(Region tmpReg){
    Collection<GeometryPart> allRegPart = tmpReg.getPartGroup().getObjects();
    return(GeometryPart) allRegPart.toArray()[0];
}
private ArrayList<Region> getRegions(Simulation tmpSim){
    return new ArrayList<>(tmpSim.getRegionManager().getRegions());
}
private ArrayList<Region> getMainWTRegions(Collection<Region> allRegs){
  ArrayList<Region> retRegionList = new ArrayList();
  for(Region tmpReg:allRegs){
    Collection<Boundary> allBndy = 
        tmpReg.getBoundaryManager().getBoundaries();

    //Main WT Region will contain Inlet or FS
    for(Boundary tmpBndy:allBndy){
      if((tmpBndy.getBoundaryType() instanceof InletBoundary) ||
        (tmpBndy.getBoundaryType() instanceof FreeStreamBoundary)){
          retRegionList.add(tmpReg);
      }
    }
  }
  return retRegionList;
}
private Collection<Region> getOversetRegions(Collection<Region> allRegs){
    Collection<Region> oversetRegs=new ArrayList();
    for(Region tmpReg:allRegs){
        Collection<Boundary> allBndy=
                tmpReg.getBoundaryManager().getBoundaries();
        //Main WT Region will contain Inlet or FS
        for(Boundary tmpBndy:allBndy){
            if(tmpBndy.getBoundaryType() instanceof OversetMeshBoundary){
                oversetRegs.add(tmpReg);
                break; //next Region
            }
        }
    }
    return oversetRegs;
}

// MESHING
private ArrayList<GeometryPart> getVolumeControlParts(){
    /* Figures out what Objects are actually in the windtunnel */
    /* Method to determine which part contains the domain */
    ArrayList<GeometryPart> retArr= new ArrayList();
    for(GeometryPart tmpPart:allGeomParts){
      if(tmpPart instanceof CompositePart){
        for(GeometryPart childPart:((CompositePart) tmpPart).
                getChildParts().getObjects()){
          String childPartName=childPart.getPresentationName();
          if(childPartName.startsWith(globalNames.getVCPreFix())){
            retArr.add(childPart);
          }
        }
      }else{
        String tmpPartName=tmpPart.getPresentationName();
        if(tmpPartName.startsWith(globalNames.getVCPreFix())){
            retArr.add(tmpPart);
        }
      }
    }
    return retArr;
}
private ArrayList<GeometryPart> getVolumeControlGroup(
        ArrayList<GeometryPart> vcParts, String groupID){
    ArrayList<GeometryPart>  retList=new ArrayList();
    for(GeometryPart tmpPart:vcParts){
        String tmpPartName=tmpPart.getPresentationName();
        if(tmpPartName.startsWith(globalNames.getVCPreFix())&&
                tmpPartName.contains(groupID)){
                retList.add(tmpPart);
        }
    }
    return retList;
}
private static String getVCGroupID(GeometryPart tmpPart){
    String tmpPartName=tmpPart.getPresentationName();

    char sepChar='_';
    int firstIndx=0;
    int secondIndx=0;
    int counter=0;
    for (int i = 0;i<tmpPartName.length();i++){
        if(tmpPartName.charAt(i)==sepChar){
            counter+=1;
            if(counter==1){
                firstIndx=i;
            }else if(counter==2){
                secondIndx=i;
                break;
            }
        }
    }
    return tmpPartName.substring(firstIndx+1,secondIndx);//removes _
}
private String getVCPctStr(GeometryPart tmpPart,String groupID){
    char sepChar='_';
    int subIndx;       //start of PCT portion of string
    String tmpPctStr; //PCT and possible description part of string
    String tmpPartName=tmpPart.getPresentationName();
    String tmpStr = globalNames.getVCPreFix()+sepChar+groupID+sepChar;
    tmpPctStr=tmpPartName.substring(tmpStr.length(),tmpPartName.length());
    //expect tmpPctStr=PCT{_DESCRIPTOR}
    //remove optional _DESCRIPTOR string if it exists
    int firstIndx=0;
    int counter=0;
    for (int i = 0;i<tmpPctStr.length();i++){
        if(tmpPctStr.charAt(i)==sepChar){
            counter+=1;
            if(counter==1){
                firstIndx=i;
                tmpPctStr=tmpPctStr.substring(0,firstIndx);
                break;
            }
        }
    }

    //convert tmpPctStr to a percentage value
    return tmpPctStr;
}
private void disableProximityOnSymm(ArrayList<Domain> allDomains){
  for(Domain tmp:allDomains){
    AutoMeshOperation autoOp = tmp.getDomainSurfMeshOp();
    String symmName = globalNames.getSymmStr();
    SurfaceCustomMeshControl tmpCntrl = MeshOpTool.surfControl(autoOp, ""
            +symmName+" No Proximity");
    tmpCntrl.getGeometryObjects().setQuery(
      new Query(new NamePredicate(NameOperator.StartsWith, symmName),
              Query.STANDARD_MODIFIERS));
    tmpCntrl.getCustomConditions().get(PartsSurfaceProximityOption.class)
     .setSelected(PartsSurfaceProximityOption.Type.DISABLE);
  }
}
private void enableProximityForWT(ArrayList<Domain> allDomains){
  for(Domain tmp:allDomains){
    AutoMeshOperation autoOp = tmp.getDomainSurfMeshOp();
    String symmName = globalNames.getSymmStr();
    SurfaceCustomMeshControl tmpCntrl = MeshOpTool.surfControl(autoOp, ""
            +symmName+" WT Proximity");
    tmpCntrl.getGeometryObjects().setQuery(
      new Query(new NamePredicate(NameOperator.StartsWith, symmName),
              Query.STANDARD_MODIFIERS));
    tmpCntrl.getCustomConditions().get(PartsSurfaceProximityOption.class)
     .setSelected(PartsSurfaceProximityOption.Type.CUSTOM_VALUES);
    SurfaceProximity custSurfaceProx = tmpCntrl.getCustomValues().get(
            SurfaceProximity.class);
    custSurfaceProx.setNumPointsInGap(5.0);
  }
}
public void updateBLProbes(Simulation tmpSim, double refRho, double refMu){
  for(WindTunnelDomain wtDomain : allWTDomains){
    wtDomain.updateBLProbes(tmpSim, refRho, refMu);
  }
}
private ArrayList<GeometryPart> getBoundaryLayerProbeParts(){
  /* Figures out what Objects are actually in the windtunnel */
  /* Method to determine which part contains the domain */
  ArrayList<GeometryPart> retArr= new ArrayList();
  for(GeometryPart tmpPart:allGeomParts){
    if(tmpPart instanceof CompositePart){
      for(GeometryPart childPart:((CompositePart) tmpPart).
              getChildParts().getObjects()){
        String childPartName=childPart.getPresentationName();
        if(childPartName.startsWith(globalNames.getBLPPreFix())){
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

// DERIVED PARTS
private static ThresholdPart highValueCells(Simulation tmpSim,
        Collection<Region> bodyParts,String partName, 
        String functionName, double highValue){
    ThresholdPart myPart;
    try{
        myPart = (ThresholdPart) tmpSim.getPartManager().getObject(partName);
    }catch(NeoException e){
        Units units_0 = 
          ((Units) tmpSim.getUnitsManager().getObject("m"));
        NullFieldFunction nullFieldFunction_0 = 
          ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                  getFunction("NullFieldFunction"));
        myPart = 
          (ThresholdPart) tmpSim.getPartManager().createThresholdPart(
                  new NeoObjectVector(new Object[] {}), new DoubleVector(
                          new double[] {0.0, 1.0}), units_0, 
                          nullFieldFunction_0, 0);
        myPart.setPresentationName(partName);
    }
    myPart.getInputParts().setQuery(null);
    myPart.getInputParts().setObjects(bodyParts);
    myPart.setFieldFunction(tmpSim.getFieldFunctionManager().
            getFunction(functionName));
    myPart.setMode(ThresholdMode.ABOVE_TAG);
    myPart.getRangeQuantities().setArray(new DoubleVector(
            new double[] {highValue, highValue}));
    return myPart;
}
private static ThresholdPart lowValueCells(Simulation tmpSim,
        Collection<Region> bodyParts,String partName, String functionName,
        double lowValue){
    ThresholdPart myPart;
    try{
        myPart = (ThresholdPart) tmpSim.getPartManager().getObject(partName);
    }catch(NeoException e){
        Units units_0 = 
          ((Units) tmpSim.getUnitsManager().getObject("m"));
        NullFieldFunction nullFieldFunction_0 = 
          ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                  getFunction("NullFieldFunction"));
        myPart = 
          (ThresholdPart) tmpSim.getPartManager().createThresholdPart(
                  new NeoObjectVector(new Object[] {}), new DoubleVector(
                          new double[] {0.0, 1.0}), units_0,
                          nullFieldFunction_0, 0);
        myPart.setPresentationName(partName);
    }
    myPart.getInputParts().setQuery(null);
    myPart.getInputParts().setObjects(bodyParts);
    myPart.setFieldFunction(tmpSim.getFieldFunctionManager().
            getFunction(functionName));
    myPart.setMode(ThresholdMode.BELOW_TAG);
    myPart.getRangeQuantities().setArray(new DoubleVector(
            new double[] {lowValue, lowValue}));
    return myPart;
}
private static PlaneSection getPlane(Simulation tmpSim,String planeName){
    return ((PlaneSection) tmpSim.getPartManager().getObject(planeName));
}
private static IsoPart getIso(Simulation tmpSim,String planeName){
    return ((IsoPart) tmpSim.getPartManager().getObject(planeName));
}

private PlaneSection singlePlane(Simulation tmpSim, Collection<Region> myRegs,
        String planeName,CoordinateSystem myCsys, double[] newOrigin,
        double[] newNormal){
    PlaneSection myPlane;
    try{
        myPlane=(PlaneSection) tmpSim.getPartManager().getObject(planeName);
    }catch(NeoException e){
        myPlane=
          (PlaneSection) tmpSim.getPartManager().createImplicitPart(
                  new NeoObjectVector(new Object[] {}), new DoubleVector(
                          new double[] {0.0, 0.0, 1.0}), new DoubleVector(
                                  new double[] {0.0, 0.0, 0.0}), 0, 1, 
                                  new DoubleVector(new double[] {0.0}));
    }
    myPlane.setCoordinateSystem(myCsys);

    Coordinate myOrientation = 
      myPlane.getOrientationCoordinate();
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    myOrientation.setCoordinate(units_0, units_0, units_0, 
            new DoubleVector(newNormal));
    Coordinate myOrigin = myPlane.getOriginCoordinate();
    myOrigin.setCoordinate(units_0, units_0, units_0, 
            new DoubleVector(newOrigin));
    myPlane.setPresentationName(planeName);
    myPlane.getInputParts().setObjects(myRegs);
    return myPlane;
}
private PlaneSection multiPlane(Simulation tmpSim,Collection<Region> myRegs,
        String planeName,CoordinateSystem myCsys,double[] newOrigin, 
        double[] newNormal, int nSec, double minVal,double maxVal){
    PlaneSection myPlane;
    try{
        myPlane=(PlaneSection) tmpSim.getPartManager().getObject(planeName);
    }catch(NeoException e){
        myPlane=
          (PlaneSection) tmpSim.getPartManager().createImplicitPart(
                  new NeoObjectVector(new Object[] {}), new DoubleVector(
                          new double[] {0.0, 0.0, 1.0}), new DoubleVector(
                                  new double[] {0.0, 0.0, 0.0}), 0, 1, 
                                  new DoubleVector(new double[] {0.0}));
    }
    myPlane.setCoordinateSystem(myCsys);

    Coordinate myOrientation = 
      myPlane.getOrientationCoordinate();
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    myOrientation.setCoordinate(units_0, units_0, units_0, 
            new DoubleVector(newNormal));
    Coordinate myOrigin =myPlane.getOriginCoordinate();
    myOrigin.setCoordinate(units_0, units_0, units_0, 
            new DoubleVector(newOrigin));
    myPlane.setPresentationName(planeName);
    myPlane.setValueMode(ValueMode.RANGE);
    RangeMultiValue rangeMultiValue_0 = 
      myPlane.getRangeMultiValue();
    rangeMultiValue_0.setNValues(nSec);
    rangeMultiValue_0.getStartQuantity().setValue(minVal);
    rangeMultiValue_0.getStartQuantity().setUnits(units_0);
    rangeMultiValue_0.getEndQuantity().setValue(maxVal);
    rangeMultiValue_0.getEndQuantity().setUnits(units_0);
    myPlane.getInputParts().setObjects(myRegs);
    return myPlane;
}
private IsoPart getSingleIsoSurfPart(Simulation tmpSim,
        Collection<Region> myRegs, String isoName, FieldFunction tmpFF,
        double newValue){
    IsoPart retPart;
    try{
        retPart = (IsoPart) tmpSim.getPartManager().getObject(isoName);
    }catch(NeoException e){
        NullFieldFunction nullFieldFunction_0 = 
          ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                  getFunction("NullFieldFunction"));
        retPart = 
          tmpSim.getPartManager().createIsoPart(new NeoObjectVector(
                  new Object[] {}), nullFieldFunction_0);
    }
    retPart.setPresentationName(isoName);
    retPart.setMode(retPart.getModeEnum().ISOVALUE_SINGLE);
    SingleIsoValue singleIsoValue_0 = 
      retPart.getSingleIsoValue();
    singleIsoValue_0.getValueQuantity().setValue(newValue);
    retPart.getInputParts().setQuery(null);

    retPart.getInputParts().setObjects(myRegs);
    Collection<NamedObject> tmpObj = retPart.getInputParts().getObjects();
    for(NamedObject tmp:tmpObj){
        if(tmp instanceof Boundary){
            tmpObj.remove(tmp);
        }
    }
    retPart.getInputParts().setObjects(tmpObj);
    retPart.setFieldFunction(tmpFF);
    return retPart;
}

// POST PROCESSING
private ImageAnnotation2D getImgAnnotation(Simulation tmpSim, String tmpName,
    String imgPath,double defH, double[] newPosition){
  ImageAnnotation2D tmpAnn;
  try{
    tmpAnn = (ImageAnnotation2D) tmpSim.getAnnotationManager().
            getObject(tmpName);
  }catch(NeoException e){
    tmpAnn = tmpSim.getAnnotationManager().createImageAnnotation2D();
    tmpAnn.setPresentationName(tmpName);
  }
  tmpAnn.setFilePath(imgPath);
  tmpAnn.setDefaultHeight(defH);
  tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
  return tmpAnn;
}
private Annotation caseNameAnnotation(Simulation tmpSim,String caseName) {
  SimpleAnnotation caseAnnotation;
  try{
    caseAnnotation = (SimpleAnnotation) tmpSim.getAnnotationManager().
            getObject("Case Name");
  }catch(NeoException e){
    caseAnnotation = tmpSim.getAnnotationManager().createSimpleAnnotation();
    caseAnnotation.setPresentationName("Case Name");
  }
  caseAnnotation.setText(caseName);
  caseAnnotation.setDefaultHeight(0.0375);
  caseAnnotation.setDefaultPosition(
      new DoubleVector(new double[] {0.0075, 0.0075, 0.0}));
  return caseAnnotation;
}
private Annotation whiteBackgroundCaseNameAnnotation(
    Simulation tmpSim,String caseName){
  SimpleAnnotation caseAnnotation;
  try{
      caseAnnotation = (SimpleAnnotation) tmpSim.getAnnotationManager().
              getObject("White Background Case Name");
  }catch(NeoException e){
      caseAnnotation = 
          tmpSim.getAnnotationManager().createSimpleAnnotation();
      caseAnnotation.setPresentationName("White Background Case Name");
  }
  caseAnnotation.setText(caseName);
  caseAnnotation.setDefaultHeight(0.0375);
  caseAnnotation.setDefaultPosition(new DoubleVector(
          new double[] {0.0075, 0.0075, 0.0}));
  caseAnnotation.setBackgroundColor(Color.white);
  return caseAnnotation;
}

// MONITORS
private static void setResidualsToAbsolute(Simulation tmpSim,
        boolean is2DCase){
      ((ResidualMonitor) tmpSim.getMonitorManager().
              getMonitor("Continuity")).getNormalizeOption().
              setSelected(MonitorNormalizeOption.Type.OFF);
      ((ResidualMonitor) tmpSim.getMonitorManager().
              getMonitor("X-momentum")).getNormalizeOption().
              setSelected(MonitorNormalizeOption.Type.OFF);
      ((ResidualMonitor) tmpSim.getMonitorManager().
              getMonitor("Y-momentum")).getNormalizeOption().
              setSelected(MonitorNormalizeOption.Type.OFF);
      if(!is2DCase){
        ((ResidualMonitor) tmpSim.getMonitorManager().
                getMonitor("Z-momentum")).getNormalizeOption().
                setSelected(MonitorNormalizeOption.Type.OFF);
      }
      ((ResidualMonitor) tmpSim.getMonitorManager().getMonitor("Energy")).
              getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
      ((ResidualMonitor) tmpSim.getMonitorManager().getMonitor("Sdr")).
              getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
      ((ResidualMonitor) tmpSim.getMonitorManager().getMonitor("Tke")).
              getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
  }

// TOOLS
// Coordinate systems
private ArrayList<CartesianCoordinateSystem> getPartCoordinateSystems(
        CoordinateSystem tmpCsys,ArrayList<GeometryPart> listParts){
    ArrayList<CartesianCoordinateSystem> retArrList=new ArrayList();
    for(GeometryPart tmpPart:listParts){
        String crdName=tmpPart.getPresentationName();
        try{
            retArrList.add((CartesianCoordinateSystem) tmpCsys.
                    getLocalCoordinateSystemManager().getObject(crdName));
        }catch(NeoException e){
            tmpPart.getSimulation().println("WT MSG: getPartCoordinate "
                    +crdName+" not found. Will likely fail!");
        }
    }
    return retArrList;
}
private ArrayList<CartesianCoordinateSystem> getAirfoilCoordinateSystems(
        CoordinateSystem tmpCsys,ArrayList<GeometryPart> airfoilParts){
    ArrayList<CartesianCoordinateSystem> retArrList=new ArrayList();
    for(GeometryPart tmpPart:airfoilParts){
        String crdName=tmpPart.getPresentationName();
        try{
            retArrList.add((CartesianCoordinateSystem) tmpCsys.
                    getLocalCoordinateSystemManager().getObject(crdName));
        }catch(NeoException e){
            tmpPart.getSimulation().println("WT MSG: getAirfoilCoordinate "
                    +crdName+" not found. Will likely fail!");
        }
    }
    return retArrList;
}
private CoordinateSystem getPartCADCoordinateSystem(Simulation tmpSim,
        GeometryPart tmpPart){
    String cadCSysName=tmpPart.getPresentationName();
    return SimTool.getLabBasedCoordinate(tmpSim,cadCSysName);
}

// SCENES
public void outputAllScenes(Simulation tmpSim, String folderPath){
  
  folderPath = SystemTool.getAbsoluteFolderPath(folderPath);
  SystemTool.touchDirectoryChain(tmpSim,folderPath);
  
  // USER IMAGE PATH
  String userPlots  = folderPath + "USER" + File.separator + "PLOTS" + File.separator;
  String userScenes = folderPath + "USER" + File.separator + "SCENES" + File.separator;
  String userCSV    = folderPath + "USER" + File.separator + "CSV" + File.separator;

  SystemTool.touchDirectoryChain(tmpSim, userPlots);
  SystemTool.touchDirectoryChain(tmpSim, userCSV);
  SystemTool.touchDirectoryChain(tmpSim, userScenes);

  // CFD IMAGE PATH
  String cfdPlots   = folderPath + "CFD" + File.separator + "PLOTS"  + File.separator;
  String cfdScenes  = folderPath + "CFD" + File.separator + "SCENES" + File.separator;
  String cfdCSV     = folderPath + "CFD" + File.separator + "CSV"    + File.separator;
  SystemTool.touchDirectoryChain(tmpSim, cfdPlots);
  SystemTool.touchDirectoryChain(tmpSim, cfdCSV);
  SystemTool.touchDirectoryChain(tmpSim, cfdScenes);
  
  // RESIDUAL PLOTS
  for(StarPlot tmpPlot : tmpSim.getPlotManager().getObjects()){
    String tmpPlotName = tmpPlot.getPresentationName();
    if(tmpPlotName.startsWith("Residuals")){
      tmpPlot.encode(userPlots + tmpPlotName + ".png", "png", xyRes[0],xyRes[1]);
    }
    try{
      tmpPlot.serverExportToSCE(userPlots + tmpPlotName
         + ".sce", false, false, tmpPlot.getPresentationName(), "Residuals");
    }catch(NeoException e){
      tmpSim.println("WTAnalysis StarPlot Residuals to sce ERR.");        
    }
  }
  for(StarPlot tmpPlot : tmpSim.getPlotManager().getObjects()){
    String tmpPlotName = tmpPlot.getPresentationName();
    if(tmpPlotName.startsWith("Residuals")){
      tmpPlot.encode(cfdPlots+tmpPlotName+".png", "png", xyRes[0],xyRes[1]);
    }
    try{
      tmpPlot.serverExportToSCE(cfdPlots + tmpPlotName
         + ".sce", false, false, tmpPlot.getPresentationName(), "Residuals");
    }catch(NeoException e){
      tmpSim.println("WTAnalysis StarPlot Residuals to sce ERR.");        
    }
  }

  // PLOTS AND SCENES
  tmpSim.println("Printing Scenes");
  for(WindTunnelDomain wtDomain : allWTDomains){
    String wtTagName = wtDomain.getPrimaryDomainTag().getPresentationName();
    String wtOutputPath = folderPath + wtTagName + File.separator;
    wtDomain.outputAllScenes(tmpSim, wtOutputPath);
  }
}
public void postProcessAeroTools(Simulation tmpSim, String folderPath){
  // Folder paths should be absolute
  folderPath = SystemTool.getAbsoluteFolderPath(folderPath);

  //USER IMAGE PATH
  String userPlots  = folderPath+"USER"+File.separator+"PLOTS"+File.separator;
  String userScenes = folderPath+"USER"+File.separator+"SCENES"+File.separator;
  String userCSV    = folderPath+"USER"+File.separator+"CSV"+File.separator;
  SystemTool.touchDirectoryChain(tmpSim, userPlots);
  SystemTool.touchDirectoryChain(tmpSim, userCSV);
  SystemTool.touchDirectoryChain(tmpSim, userScenes);

  //CFD IMAGE PATH
  String cfdPlots   = folderPath + "CFD" + File.separator + "PLOTS"  + File.separator;
  String cfdScenes  = folderPath + "CFD" + File.separator + "SCENES" + File.separator;
  String cfdCSV     = folderPath + "CFD" + File.separator + "CSV"    + File.separator;
  SystemTool.touchDirectoryChain(tmpSim, cfdPlots);
  SystemTool.touchDirectoryChain(tmpSim, cfdCSV);
  SystemTool.touchDirectoryChain(tmpSim, cfdScenes);

  // Post process aero tools with wind tunnel domain.
  for(WindTunnelDomain wtDomain : allWTDomains){
    String wtTagName = wtDomain.getPrimaryDomainTag().getPresentationName();
    SystemTool.touchDirectoryChain(tmpSim, folderPath + File.separator 
    + wtTagName + File.separator);

    wtDomain.postProcessAeroTools(tmpSim, folderPath + File.separator
        + wtTagName + File.separator);
  }
}
////
} // END WindTunnelAnalysis.java
