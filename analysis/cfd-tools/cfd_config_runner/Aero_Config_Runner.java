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

Note:
  This code automates aerodynamic analyses in STAR-CCM+ release:
  STAR-CCM+ 13.06.010

Documentation:

    Framework Design Philosophy: 
        1) Help and Respect the CFD Engineer:
            *Explicitly indicate when existing simulation settings are being
             overridden. 
            *Provide the option to turn off the override in the USER EDIT
             field. No hunting and pecking for magic buttons.
        2) Adhere to existing STAR-CCM+ functionality wherever possible

    Framework Benefits:
        1) Save engineering time: automate common, repeated, tedious tasks
            *Automatically adhere to Best-Known-Practices
        2) Future-Proof Simulation Results:
            *Remove user setup errors or "oh I forgots"
            *Adapt mindset of only running a case once

    Framework Scope:
        *Automate the mesh setup of Makani M600 geometries 

    Framework Features:
        *Automated setup assumes that the mesh Operaitons tree is empty

*/

import java.util.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.io.*;
import java.math.*;

// Geometry and Meshing
import star.common.*;
import star.base.neo.*;
import star.base.query.*;
import star.resurfacer.*;
import star.meshing.*;
import star.prismmesher.*;
import star.trimmer.*;
import star.solidmesher.*;
import star.sweptmesher.*;
import star.twodmesher.*;

// Physics
import star.flow.*;
import star.energy.*;
import star.segregatedflow.*;
import star.segregatedenergy.*;
import star.material.*;
import star.vdm.*;

// Turbulence modeling
import star.kwturb.*;
import star.walldistance.*;

// Motion
import star.motion.*;

// Reports
import star.base.report.*;

// Post processing
import star.vis.*;

// Aero runner Libary
import AeroToolKit.*;
import Tools.*;
import Solvers.*;
import MeshTools.*;
import PhysicsTools.*;
import MeshTools.VolumeMeshTool;


public class Aero_Config_Runner extends StarMacro {
  // Simulation and file information
  private Simulation simu;
  private String postSubFolder;
  private String caseName;
  private String simPathName;

  // Version Control
  private String ccmVersion;
  private String javaScriptVersion;

  // Custom Output Locations
  private String m600PostProcFolder="M600_POST";

  // Constants
  private final double SMALL_EPS = 1.e-6;

  // Geometry
  double charBaseSize=0.1;
  double kiteAlpha_o;
  double kiteBeta_o;
  double kitePhi0;
  double defaultRotAngle=0.0;

  // Default flight condition
  double kiteAlpha = 0.0;
  double kiteBeta = 0.0;
  double kitePhi = 20.0;
  double airSpeed = 0.0;
  double airDensity = 1.176;
  double airViscosity = 1.85508E-5;
  double refPressure = 101325.0;

  // Physics continua
  private double wallDistanceToFS = 0.1;
  private Collection<PhysicsContinuum> allUsedFluidPhysics;
  
  // Initial conditions
  private boolean needHighViscosityInit = false;
  private int numHighViscItSteps = 500;
  
  // Solver
  private boolean autoSimStep = true;
  private int maxSteadySteps = 4000;
  private double unSteadyTimeStep = 1.e-3;
  private int unSteadyInnterIterations = 20;
  private double propellerDegreesPerTimeStep = 2.0;
  private double propellerDiscretizationsPerTimeStep;
  // Axis shorthand
  double[] zeroOrigin = {0.0,0.,0.};
  double[] xOnlyAxis  = {1.0,0.,0.};
  double[] yOnlyAxis  = {0.0,1.,0.};
  double[] zOnlyAxis  = {0.0,0.,1.};

  private Units prefUVec ; // preferred units vector

  //Geometry
  ArrayList<Part> kiteParts = new ArrayList();
  ArrayList<GeometryPart> rotatableParts = new ArrayList();

  // Mesh Operations
  ArrayList<MeshOperation> rotorOperations = new ArrayList();

  // Region variables
  private Collection<Region> allBodyRegs; 
  private Collection<Region> allRotorRegs;
  ArrayList<Region> allRegs = new ArrayList();

  // Coordinate systems
  LabCoordinateSystem labCsys;
  CartesianCoordinateSystem bodyCsys;
  LocalCoordinateSystemManager bodyCsysManager;
  CartesianCoordinateSystem groundCsys;
  CartesianCoordinateSystem zeroCADAngleBodyCsys;
  CartesianCoordinateSystem meshAngleCsys;
  CartesianCoordinateSystem inletCsys;

  String bodyCsysName="Body Csys";
  String zeroCADAngleBodyCsysName = "CAD Zero "+bodyCsysName;
  String meshAngleCsysName="Mesh Angle Csys";
  String inletCsysName="Velocity Inlet";

  ArrayList<String> controlsCsysList = new ArrayList();
  ArrayList<String> rotorsCsysList = new ArrayList();
  ArrayList<String> radiatorCsysList = new ArrayList();

  //Proxy
  ProxyRepresentation simProxy;

  //Post
  ArrayList<VisView> fieldViews = new ArrayList();
  ArrayList<Annotation> standardAnnotations = new ArrayList();
  String studyPostDir;
  
  String userPlots;
  String userScenes;
  String userCSV;
  
  String cfdPlots;
  String cfdScenes;
  String cfdCSV;

  //Reference Lengths/Areas
  double nominalChord = 1.2831;
  double wingSpanNorm = 25.6626;
  double referenceArea  = 32.9285;

  // Performance metric
  double systemCD0 = 0.18;
  int nDataSamples = 500;
  int[] outputRes ={2000,1000};
  double  viewAngleDistanceOffset = Math.atan(15.0 * Math.PI / 180.0);

  private String studyName="Local";
  ArrayList<PitotProbe> allPitots = new ArrayList();

  //Set all ENV presets to private variables
  private String newKitePartFileName;
  private boolean needRunPreProc     =true;
  private boolean needRunMesh        =false;
  private boolean needRunSolver      =false;
  private boolean needClearSolution  =false;
  private boolean needRunUnsteady    =false;
  private boolean needRunPost        =false;
  private boolean needQuickPost      =false;
  private boolean needRunRotors      =false;
  private boolean needRunBEM         =false;
  private boolean needStarView       =false;
  
  //special solvers
  private boolean needChangeMaxSteadySteps =false;
  
  //flap angles
  private double flapA1_angle = 0.0;
  private double flapA2_angle = 0.0;
  private double flapA3_angle = 0.0;
  private double flapA4_angle = 0.0;
  private double flapA5_angle = 0.0;
  private double flapA6_angle = 0.0;
  private double flapA7_angle = 0.0;
  private double flapA8_angle = 0.0;
  private double htail_angle  = 0.0;
  private double rudder_angle = 0.0;
  
  private double omega_hat_x = 0.0;
  private double omega_hat_y = 0.0;
  private double omega_hat_z = 0.0;
  
  private double om_origin_rx = 0.0;
  private double om_origin_ry = 0.0;
  private double om_origin_rz = 0.0;
  
  private double rtr_1_low = 0.0;
  private double rtr_2_low = 0.0;
  private double rtr_3_low = 0.0;
  private double rtr_4_low = 0.0;
  private double rtr_1_upp = 0.0;
  private double rtr_2_upp = 0.0;
  private double rtr_3_upp = 0.0;
  private double rtr_4_upp = 0.0;
  
  // Special MRF Crosswind Flags
  private boolean isCrossWindMRFCase = false;

  public void execute(){
    //===================================
    // Familiarize with simulation
    //===================================
    this.simu=getActiveSimulation(); 
    this.caseName = simu.getPresentationName();
    this.simPathName = simu.getSessionDir();
    this.prefUVec = simu.getUnitsManager().getPreferredUnits(
      new IntVector(new int[] {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    this.labCsys = simu.getCoordinateSystemManager().getLabCoordinateSystem();
    this.allUsedFluidPhysics = CFD_Physics.getAllFluidPhysics(simu);
    this.simProxy=getSimProxy("Proxy");

    // M600 Runner Additions
    this.studyName = getSubFolderName();

    this.javaScriptVersion=Aero_Config_Runner.class.getSimpleName()+"_v1p04e";
    this.ccmVersion=simu.getStarVersion().getString("ReleaseNumber");

    // Customized Post-processing folders to /<Study_Folder>/M600_POST
    this.m600PostProcFolder=simPathName+File.separator+m600PostProcFolder;
    this.postSubFolder=caseName;

    // Coordinate systems
    simu.println("M600 RUN: Setting up primary coordinate systems.");
    setupMainCoordinateSystems();
    this.bodyCsysManager = bodyCsys.getLocalCoordinateSystemManager();

    // Auto time step calculation
    this.propellerDiscretizationsPerTimeStep = 360./propellerDegreesPerTimeStep;
    simu.println("M600 RUN: Gathering Rotating and Stationary Regions");
    this.allRotorRegs = gatherRotatingBodyRegions();
    this.allBodyRegs = gatherBodyRegions();
    simu.println("=============Gather body regions complete.");
    simu.println("========= allBodyRegs size = "+allBodyRegs.size());
    
    // Just makes sense to have knowledge of pi
    ScalarGlobalParameter pi_param =
            SimTool.getScalarSimulationParameter(simu,"pi");
    pi_param.getQuantity().setValue(Math.PI);

    //==================================
    // READ CASE INPUTS
    //==================================
    simu.println("M600 RUN: Reading input file.");
    readInputFile();
    
    // Adjustments to global objects based on input file
    adjustGroundLabCoordinateSystem();
    
    // Setup XWind Usable Parameters
    ScalarGlobalParameter kitePhi_param =
            SimTool.getScalarSimulationParameter(simu,"phi");
    kitePhi_param.getQuantity().setValue(kitePhi);

    // Parameters to Control the Run
    if(needChangeMaxSteadySteps){
      this.maxSteadySteps = maxSteadySteps;
    }
    if(needHighViscosityInit){
      this.numHighViscItSteps = numHighViscItSteps;
    }
    
    //======================================
    // Begin Java Macro
    //======================================
    //Forces hard numbers into coordinate systems
    simu.println("M600 RUN: Enforcing all default M600 coordinate systems.");
    forceDefaultBodyCoordinateSystems();

    // Ability to replace Kite Part in batch
    simu.println("M600 RUN: Check on requested replacement kite CAD parts.");
    if( !newKitePartFileName.isEmpty() ){
      simu.println("M600 RUN: New Kite File Detected: Replacing Kite Part");
      replaceKitePart("CFD Kite Parts",newKitePartFileName);
      needRunPreProc=true;
    }

    //=======================================
    //      PRE-PROCESSING OPTIONS
    //=======================================
    if(needRunPreProc){
      createNeededSurfVCs();

      // Check to see if there are MRF values for volume controls
      this.isCrossWindMRFCase = needRunXWind(omega_hat_x,omega_hat_y,omega_hat_z);
      createMeshSensitiveVCs();
      simu.println("    ");
      simu.println("M600 PreProc: Initiating case parameters...");
      simu.println("    ");

      setupCaseFreeStreamParameters();

      //==============================
      // BEGIN USER PART DEFINITIONS
      //==============================
      simu.println("M600 PreProc: Defining parts.");

      String strKiteAssembly="CFD Kite Parts";
      String strRadiatorAssembly ="CFD Kite Parts";
      String strControlsAssembly ="CFD Control Parts";
      String strRotorBladeAssembly = "CFD Rotor Parts";

      //
      String strRotorIntAssembly ="Rotor Interface Parts";

      // Define Rotor CAD Interface
      ArrayList<GeometryObject> rotorIntGeom = new ArrayList();
      CompositePart rotorIntAssembly = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRotorIntAssembly));
      for(int i=1;i<=4;i++){
        rotorIntGeom.add((SimpleCylinderPart) rotorIntAssembly.getChildParts().
                getPart("Rotor "+i+" Lower Interface"));
        rotorIntGeom.add((SimpleCylinderPart) rotorIntAssembly.getChildParts().
                getPart("Rotor "+i+" Upper Interface"));
      }

      // Define Rotor CAD Blades
      ArrayList<GeometryPart> rotorBladeGeom = new ArrayList();
      CompositePart bladesAssembly = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRotorBladeAssembly));
      for(int i=1;i<=4;i++){
        rotorBladeGeom.add(bladesAssembly.getChildParts().getPart("Rotor "+i+" Lower"));
        rotorBladeGeom.add(bladesAssembly.getChildParts().getPart("Rotor "+i+" Upper"));
      }

      // Define Radiator CAD Bodies
      ArrayList<GeometryPart> radiatorGeom = new ArrayList();
      CompositePart radiatorAssembly = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRadiatorAssembly));
      for(int i=1;i<=4;i++){
        radiatorGeom.add(radiatorAssembly.getChildParts().getPart("Radiator "+i+" Lower"));
        radiatorGeom.add(radiatorAssembly.getChildParts().getPart("Radiator "+i+" Upper"));
      }

      // Define Control Surfaces CAD geometry
      ArrayList<GeometryObject> controlsGeom = new ArrayList();
      ArrayList<GeometryObject> flapGeom = new ArrayList();
      CompositePart controlsAssmb = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strControlsAssembly));
      controlsGeom.add(controlsAssmb.getChildParts().getPart("H Tail"));
      controlsGeom.add(controlsAssmb.getChildParts().getPart("Rudder"));
      for(int i=1;i<=8;i++){
          flapGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Flap A"+i));
          controlsGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Flap A"+i));
      }
      // slat
      try{
          flapGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Slat"));
          controlsGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Slat"));
      }catch(NeoException e){
      }

      //===================================
      // AUTOMATION W/ BEST PRACTICES
      //===================================
      // Mesh Operation Tree
      simu.println("M600 PreProc: Setting up mesh operation tree.");
      setupMeshOperationTree(charBaseSize, needRunRotors);

      // Physics
      simu.println("M600 PreProc: Making physics...");
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
        CFD_Physics.set_RANS_KwSST(tmpPhys,true,true,true,false,false,wallDistanceToFS);
      }

      // Regions
      simu.println("M600 PreProc: Creating kite Region...");
      Region kiteRegion = setupKiteRegion(simu.get(SimulationPartManager.class).getPart("Kite Domain"));

      simu.println("M600 PreProc: Creating radiator regions...");
      ArrayList<Region> allRadReg = setupRadiatorRegions(radiatorGeom);

      simu.println("M600 PreProc: Setting up radiator interfaces...");
      setupRadiatorInterfaces(kiteRegion,allRadReg);

      ArrayList<Region> allRotorReg=null;
      if(needRunRotors){
        simu.println("M600 PreProc: Creating rotor regions...");
        allRotorReg = setupRotorRegions(rotatableParts);
        setupRotorInterfaces(kiteRegion,allRotorReg);
      }else if(needRunBEM){
        simu.println("M600 PreProc: Creating BEM Objects...");
        simu.println("BEM MSG: Loading All BEM FOIL");
        ArrayList<FileTable> foilTables =getFileTables("BEM", "FOIL");
        simu.println("BEM MSG: Loading BEM CHORD");
        ArrayList<FileTable> chordVSrArr=getFileTables("BEM","CHORD");
        simu.println("BEM MSG: Loading BEM TWIST");
        ArrayList<FileTable> twistVSrArr=getFileTables("BEM","TWIST");
        if ((chordVSrArr.size()>1)|| (twistVSrArr.size()>1)){
            simu.println("BEM MSG: ERROR! Multiple CHORD or TWIST File Tables Found! ");
            simu.close();
        }
        FileTable[] chordVSr = chordVSrArr.toArray(new FileTable[chordVSrArr.size()]);
        FileTable[] twistVSr = twistVSrArr.toArray(new FileTable[twistVSrArr.size()]);
        simu.println("BEM MSG: Loading Rotor Coordinates under PreProc");
        ArrayList<CartesianCoordinateSystem> allRotorCoords=getAllRotorCoords();
        for(CartesianCoordinateSystem rtrCoord:allRotorCoords){
          String coordName=rtrCoord.getPresentationName();
          RotatingMotion rtrRM=getRotatingMotion(coordName);
          double rtrRotSpeed;
          switch (coordName) {
            case "Rotor 1 Lower": rtrRotSpeed=rtr_1_low; break;
            case "Rotor 2 Lower": rtrRotSpeed=rtr_2_low; break;
            case "Rotor 3 Lower": rtrRotSpeed=rtr_3_low; break;
            case "Rotor 4 Lower": rtrRotSpeed=rtr_4_low; break;
            case "Rotor 1 Upper": rtrRotSpeed=rtr_1_upp; break;
            case "Rotor 2 Upper": rtrRotSpeed=rtr_2_upp; break; 
            case "Rotor 3 Upper": rtrRotSpeed=rtr_3_upp; break; 
            case "Rotor 4 Upper": rtrRotSpeed=rtr_4_upp; break; 
            default: rtrRotSpeed=0.0; break;
          }
          setRotatingMotionSpeed(rtrRM,rtrRotSpeed,0,0);

          // Blade geometry
          int azRes=30; int radRes=30; //azimuthal and radial resolution of disk
          int nBlades=5; double bldThickness=0.1;
          double innerRad=0.32; double outerRad=1.16;
          VirtualDisk bemRtr=makeBEMRotor(allUsedFluidPhysics,coordName,rtrCoord,rtrRotSpeed,
                        azRes,radRes,nBlades,innerRad, outerRad,bldThickness,
                        foilTables, chordVSr[0],twistVSr[0]);
          // Make custom volume control
          GeometryPart bemVC = getBemVC(bemRtr);
          // Inject custom mesh operation into "Volume Mesh"
          simu.println("Injecting BEM VC:");
          injectBemVC(bemVC,bldThickness);

          // Create BEM Reports of Interest
          Report vdFxReport = bemForceReport(bemRtr, 0);
          Report vdFyReport = bemForceReport(bemRtr, 1);
          Report vdFzReport = bemForceReport(bemRtr, 2);

          Report vdMxReport = bemMomentReport(bemRtr, 0);
          Report vdMyReport = bemMomentReport(bemRtr, 1);
          Report vdMzReport = bemMomentReport(bemRtr, 2);

          Monitor bemFXMon = MonitorTool.reportIterationMonitor(simu, vdFxReport, maxSteadySteps, 1, 1);
          Monitor bemFYMon = MonitorTool.reportIterationMonitor(simu, vdFyReport, maxSteadySteps, 1, 1);
          Monitor bemFZMon = MonitorTool.reportIterationMonitor(simu, vdFzReport, maxSteadySteps, 1, 1);

          Monitor bemMXMon = MonitorTool.reportIterationMonitor(simu, vdMxReport, maxSteadySteps, 1, 1);
          Monitor bemMYMon = MonitorTool.reportIterationMonitor(simu, vdMxReport, maxSteadySteps, 1, 1);
          Monitor bemMZMon = MonitorTool.reportIterationMonitor(simu, vdMxReport, maxSteadySteps, 1, 1);

        }
        // Configure flap settings
        setWingHighYPlusMesh(12, 0.001);
        setFlapHighYPlusMesh(12, 0.001);
      }

      //=======================================
      // Post-processing setup
      //=======================================
      simu.println("setting up case free stream reports");
      setupCaseFreeStreamParameters();

      // Annotations:
      simu.println("Setting up HPC Reports:");
      makeHPCReports(100);

      // reports w/ monitors
      simu.println("Setting up standard views");
      setStandardKiteViews();

      simu.println("Setting up derived parts");
      Collection<Region> allRegions=simu.getRegionManager().getRegions();

      setupDerivedParts(simu.getRegionManager().getRegions());
      
      //
      simu.println("Making kite reports");
      makeKiteReports(kiteRegion,allRadReg,true);
      if(needRunRotors){
        simu.println("Making rotor reports");
        makeRotorReports(allRotorReg,true);
      }

      //Individual components
      simu.println("M600 PreProc: Making individual component reports.");
      makePartCoefficientReports(kiteRegion,"MainWing ","1",1,true); //main wing
      makePartMomentReports(kiteRegion, "MainWing ", "1", 1, 1, true);
      try{
        makePartCoefficientReports(kiteRegion,"Slat ","29",1,true);    //slat
        makePartMomentReports(kiteRegion, "Slat ", "29", 1, 1, true);
      }catch(NeoException e){
        simu.println("M600 PreProc: Warning! No slat detected!");
      }
      makePartCoefficientReports(kiteRegion,"Flap A","2",8,true);   //flaps
      makePartMomentCoefficientReports(kiteRegion, "Flap A ", "2", 1, 8, true);

      makePartCoefficientReports(kiteRegion,"Pylon ","4",4,true);    //pylons
      makePartMomentCoefficientReports(kiteRegion, "Pylon ", "4", 1, 4, true);

      makePartCoefficientReports(kiteRegion,"Fuselage ","6",1,true); //fuselage
      makePartMomentCoefficientReports(kiteRegion, "Fuselage ", "6", 1, 1, true);

      makePartCoefficientReports(kiteRegion,"V Tail ","52",1,true);  //vtail
      makePartMomentCoefficientReports(kiteRegion, "V Tail ", "52", 1, 1, true);

      makePartCoefficientReports(kiteRegion,"Rudder ","53",1,true);  //rudder
      makePartMomentCoefficientReports(kiteRegion, "Rudder ", "53", 1, 1, true);

      makePartCoefficientReports(kiteRegion,"H Tail ","54",1,true);  //htail
      makePartMomentCoefficientReports(kiteRegion, "H Tail ", "54", 1, 1, true);

      // Rotating aerodynamic surface local axis moments
      try{
        makePartLocalMomentReport(kiteRegion,"Slat ","29",1,1,bodyCsysManager
            .getObject("CAD Slat Axis"),xOnlyAxis,zeroOrigin,true);
      }catch(NeoException e){
      }
      makePartLocalMomentReport(kiteRegion,"Flap A","2",1,2,bodyCsysManager
          .getObject("CAD Flap A1A2 Axis"),xOnlyAxis,zeroOrigin,true);
      makePartLocalMomentReport(kiteRegion,"Flap A","2",3,4,bodyCsysManager
          .getObject("CAD Flap A3A6 Axis"),xOnlyAxis,zeroOrigin,true);
      makePartLocalMomentReport(kiteRegion,"Flap A","2",7,2,bodyCsysManager
          .getObject("CAD Flap A7A8 Axis"),xOnlyAxis,zeroOrigin,true);
      //
      makePartLocalMomentReport(kiteRegion,"Rudder ","53",1,1,bodyCsysManager
          .getObject("CAD Rudder Axis"),xOnlyAxis,zeroOrigin,true);
      makePartLocalMomentReport(kiteRegion,"H Tail ","54",1,1,bodyCsysManager
          .getObject("CAD H Tail Axis"),xOnlyAxis,zeroOrigin,true);
      //
      groupResidualMonitors();

      // Set up the Monitors to not track data for every step during initial convergence
      MonitorTool.setAllReportBasedMonitorFrequency(simu, 200);

      // Set relevant Stopping Criteria
      setKiteBasedStoppingCriteria(needRunUnsteady, nDataSamples);
      setOutputMonitorDisplay();

      // Plots
      simu.println("M600 PreProc: Setting up Plots.");
      setupInitialPlots();
      postProcessPlots("It",false);

      // Annotations
      simu.println("M600 PreProc: Setting up Annotations.");
      initAnnotations(simu);

      //scenes
      simu.println("M600 PreProc: Setting up Scenes.");
      setupKiteScenes(needRunRotors);
      setupFieldScenes(needRunRotors);
      if(needRunRotors) setupRotorScenes();

      // Track the fact that the case was Preprocessed
      setCaseCondition("PREPROC");
      setOldCaseName(caseName);

      // Save case only if pre-processing is the only option
      if(needRunPreProc && !needRunMesh && !needRunSolver &&!needRunPost &&!needQuickPost){
        // just save a pre-processed file
        simu.saveState(simPathName+File.separator+caseName+"_pre.sim");
      }
    }

    //=======================================
    //      MESHING
    //=======================================
    if(needRunMesh){
      simu.println("M600 MESH: Executing kite meshing.");

      // Make volume controls for a rotated operation
      if( (Math.abs(kiteAlpha_o) > SMALL_EPS) ||
          (Math.abs(kiteBeta_o) > SMALL_EPS) ){
        createMeshSensitiveVCs();
      }

      simu.println("M600 MESH: Setting Up Beta_o Coordinate system.");
      // The Beta_0 coordinate system exists to allow the mesh operation pipeline
      // to not be updated for a combination of Alpha_0 and Beta_0 to not break
      // the mesh update. During meshing it can be reset, then rotated to the 
      // correct Beta_0 value.
      modifyCoordSys( "Beta_o", xOnlyAxis, yOnlyAxis, zeroOrigin);
      CoordinateSystem tmpCsys= SimTool.getLabBasedCoordinate(simu, "Beta_o");
          ((CartesianCoordinateSystem) tmpCsys).getOrigin()
              .setCoordinate(prefUVec, prefUVec, prefUVec,
                  new DoubleVector(zeroOrigin));
      ((CartesianCoordinateSystem) tmpCsys).setBasis0(new DoubleVector(xOnlyAxis));
      ((CartesianCoordinateSystem) tmpCsys).setBasis1(new DoubleVector(yOnlyAxis));
      // Rotate this coordinate system to beta_o position and update the mesh
      // automation pipeline.
      double[] negativeZAxis = {0.0, 0.0, -1.0};
      SimTool.rotateLabBasedCoordinateSystem(simu, kiteBeta_o, negativeZAxis, tmpCsys);
      setTransformRotationAngle("Kite Rotate Beta0", labCsys, kiteBeta_o, zeroOrigin, negativeZAxis); //negative rotation about z for positive Beta
      if(needRunRotors){
        setTransformRotationAngle("Rotor Rotate Beta0", labCsys, kiteBeta_o, zeroOrigin, negativeZAxis);
      }
      // The mesh automation pipeline does not need to have an alpha_o coordinate system in place
      setTransformRotationAngle("Kite Rotate Alpha0",tmpCsys,kiteAlpha_o,zeroOrigin,yOnlyAxis);
      if(needRunRotors) setTransformRotationAngle("Rotor Rotate Alpha0",tmpCsys,kiteAlpha_o,zeroOrigin,yOnlyAxis);

      // If alpha_0 and beta_0 have been set, then the Body Coordinate System
      // should not longer be aligned with the CCM body coordinate system. It
      // must be changed to align with the new CAD orientation.
      //
      // Because we are remeshing, we can just reset the body coordinate system
      // and perform the same rotations as was done for the beta_0. Resetting
      // handles the kiteAlpha_o=kiteBeta_o=0 condition.
      //
      // Ramifications:
      //   The Body Coordinate system must refer back to Alpha_0 and Beta_0
      //   during solve, regardless of whether the mesh flag has been optioned.
      bodyCsys.setBasis0(new DoubleVector(xOnlyAxis));
      bodyCsys.setBasis1(new DoubleVector(yOnlyAxis));
      SimTool.rotateLabBasedCoordinateSystem(simu, kiteBeta_o,
          new double[] {0., 0., -1.0}, bodyCsys);
      SimTool.rotateCoordinateSystem(simu, kiteAlpha_o, yOnlyAxis, bodyCsys, bodyCsys);
      
      // Because the case is freshly meshed and the body coordinate system must
      // naturally align with this new mesh, we can just copy the basis set from
      // bodyCsys to meshAngleCsys
      meshAngleCsys.setBasis0(bodyCsys.getBasis0());
      meshAngleCsys.setBasis1(bodyCsys.getBasis1());
      
      // Set Flap Angles, H Tail Angle, Rudder Angle
      setFlapAngles(flapA1_angle, flapA2_angle, flapA3_angle, flapA4_angle,
                    flapA5_angle, flapA6_angle, flapA7_angle, flapA8_angle);
      setHTailAngle(htail_angle);
      setRudderAngle(rudder_angle);

      //=======================================
      // Execute Surface Meshing Operations
      //=======================================
      simu.println("M600 MESH: Completing surface mesh operations.");
      try{
          ((TransformPartsOperation)  simu.get(MeshOperationManager.class)
                  .getObject("Kite Rotate Alpha0")).execute();
          if(needRunRotors){
              ((TransformPartsOperation)  simu.get(MeshOperationManager.class)
                  .getObject("Rotor Rotate Alpha0")).execute();
          }
      }catch(NeoException e){
          simu.println("M600 MESH WARNING: Kite Rotate Alpha0. Not Found!");
      }

      //=======================================
      // Execute Volume Meshing Operations
      //=======================================
      try{
        simu.getMeshPipelineController().generateVolumeMesh();
        simu.println("M600 MESH: VOLUME MESH GENERATED SUCCESSFULLY.");
      }catch(NeoException e){ // case failed to mesh - save it out before crash
        caseName = fileStringCompiler(needRunUnsteady,
                doesCaseNeedTransitionModel(getKiteAlpha()),needRunRotors);
        simu.saveState(simPathName+File.separator+caseName+"_Surf2VolFail.sim");
        simu.println("M600 MESH ERROR: VOLUME MESHER FAILED.");
      }

      // Track the fact that the case was ReMeshed and what the caseName is
      setCaseCondition("REMESHED");
      setOldCaseName(caseName);

      // Note: The code below, particularly the GRT and else statement is set up
      //   to handle a case that has been remeshed after already being solved.
      caseName = fileStringCompiler(needRunUnsteady,
              doesCaseNeedTransitionModel(getKiteAlpha()),needRunRotors);
      if(needRunMesh && !needRunSolver &&!needRunPost &&!needQuickPost){
        // just save a file that was only specified for meshing
        simu.saveState(simPathName+File.separator+caseName+"_meshed.sim");
      }else{ // the case has been remeshed, save and proceed
        simu.saveState(simPathName+File.separator+caseName+".sim");  
      }
    }

      //=======================================
      //      SOLVING
      //=======================================
      if(needRunSolver){
        // Figure out if case is from a remesh or a previous solve
        simu.println("M600 SOLVE: Getting Case Conditions");
        String oldIC = getCaseCondition(); //what happened to it previously
        String oldCaseName = getOldCaseName(); // what case was it
        if(oldIC.endsWith("REMESHED")){
          setOldCaseName("Remeshed from: "+oldCaseName);
        }else if(oldIC.endsWith("PREPROC")){
          setOldCaseName("Preproc redone on: "+oldCaseName);
        }else{//Case From a previously solved case
          setOldCaseName("Previous Solution: "+caseName);
        }

        // Set up for standard zero omega run
        simu.println("M600 SOLVE: Setting up free stream paramters.");
        setupCaseFreeStreamParameters();

        // Set Body Csys to be aligned with any alpha_0, beta_0 configuration
        // Get alpha_0, beta_0
        kiteBeta_o = MeshOpTool.getRotationAngle(simu,"Kite Rotate Beta0");
        kiteAlpha_o = MeshOpTool.getRotationAngle(simu,"Kite Rotate Alpha0");
        
        simu.println("kiteBeta_o= " + kiteBeta_o);
        simu.println("kiteAlpha_o= " + kiteAlpha_o);

        // Ramifications:
        //   The Body Coordinate system must refer back to Alpha_0 and Beta_0
        //   during solve, regardless of whether the mesh flag has been optioned.
        bodyCsys.setBasis0(new DoubleVector(xOnlyAxis));
        bodyCsys.setBasis1(new DoubleVector(yOnlyAxis));
        
        simu.println("rotating kiteAlpha_0");
        
        SimTool.rotateLabBasedCoordinateSystem(simu, kiteBeta_o,
            new double[] {0., 0., -1.0}, bodyCsys);
        SimTool.rotateCoordinateSystem(simu, kiteAlpha_o, 
                yOnlyAxis, bodyCsys, bodyCsys);
        // Set proper Alpha/Beta for velocity inlet
        simu.println("M600 SOLVE: Setting inlet vector alpha and beta.");
        setInletVectorAlphaBeta();
        // Set standard initial conditions
        simu.println("M600 SOLVE: Setting initial conditions.");
        set_VelocityIC();
        //set_PressureIC(); //experimental!
        try{
          set_TemperatureIC();
        }catch(NeoException e){
          
        }
        simu.println("M600 SOLVE: Setting outlet boundary conditions.");
        set_OutletBC();
        
        //Set Lab Ref Frame in Case previous case is XWind
        applyLabRefFrame(allBodyRegs);
        if(needRunRotors) applyLabRefFrame(allRotorRegs);

        // If Flying XWind
        simu.println("M600 SOLVE: Checking for crosswind case.");

        double omega_x = omega_hat_x*2*airSpeed/wingSpanNorm;
        double omega_y = omega_hat_y*2*airSpeed/nominalChord;
        double omega_z = omega_hat_z*2*airSpeed/wingSpanNorm;

        // no origin of rotation unless there are omegas!
        setupXWindOmegaOriginReports(0.0, 0.0, 0.0);
        
        boolean isCompressible = true;
		
        if(needRunXWind(omega_x,omega_y,omega_z)){
		  this.isCrossWindMRFCase = true;
          // TODO: formulate compressible flow boundary condition
          isCompressible = false;

          // To remain consistent with the aerodatabases, the body omega *hat*
          // values, and representative origin are reported as entered.
          setupXWindOmegaReports(omega_hat_x, omega_hat_y, omega_hat_z, kitePhi);
          setupXWindOmegaOriginReports(om_origin_rx, om_origin_ry, om_origin_rz);

          // Dimensional omega vector field functions
          double[] omega_body = {omega_x, omega_y, omega_z};

          // Note that lab and body are assumed to be oriented
          double[][] lab2ground = getR_X(-1.*kitePhi*Math.PI/180.);
          double[] omega_ground = SimTool.transformVector(SimTool.get3x3Transpose(lab2ground), omega_body);
          simu.println("Omega array in ground:");
          SimTool.simPrint1x3Array(simu, omega_ground);
          simu.println(" ");

          // Omega Ground
          UserFieldFunction omega_ground_FF = SimTool.getUserVectorFF(simu,"Omega Ground");
          omega_ground_FF.setFunctionName("omega_ground");
          omega_ground_FF.setDefinition("[" + omega_ground[0] + "," +omega_ground[1] 
                                            + "," + omega_ground[2] + "]");

          // Omega Lab
          UserFieldFunction omega_lab_FF = SimTool.getUserVectorFF(simu,"Omega Lab");
          omega_lab_FF.setFunctionName("omega_lab");
          omega_lab_FF.setDefinition("[$$omega_ground[0],"
                  +"$$omega_ground[1]*cos(${phi}*${pi}/180.) - $$omega_ground[2]*sin(${phi}*${pi}/180.),"
                  +"$$omega_ground[1]*sin(${phi}*${pi}/180.) + $$omega_ground[2]*cos(${phi}*${pi}/180.)]");

          // Omega Magnitude
          UserFieldFunction omega_mag_FF = SimTool.getUserScalarFF(simu,"Omega_Mag");
          omega_mag_FF.setFunctionName("omega_mag");
          omega_mag_FF.setDefinition("mag($$omega_ground)");

          // Set XWind I.C.s
          setIterationRamp(1000);
          setOmegaRampFF(omega_body[0], omega_body[1], omega_body[2]);
          setXWindVelocityIC(omega_ground[0], omega_ground[1], omega_ground[2]); //currently a basic implementation

          // We define the Xwind Rotation Motion in the CCM Lab Coordinate system.
          // So, we use the ccm translated values for both omega and origin_r
          setXWindRotatingMotion(groundCsys, omega_ground[0], omega_ground[1], omega_ground[2]);

          // Rotating motion velocity vector field functions
          UserFieldFunction up_ground_FF = SimTool.getUserVectorFF(simu,"uP_Ground");
          up_ground_FF.setFunctionName("up_ground");
          up_ground_FF.setDefinition("$$Velocity(@ReferenceFrame(\"ReferenceFrame for Kite XWind MRF\"))");
    
          // Rotating motion velocity vector field functions
          UserFieldFunction up_body_FF = SimTool.getUserVectorFF(simu,"uP_Body");
          up_body_FF.setFunctionName("up_body");
          up_body_FF.setDefinition("$$up_ground(@CoordinateSystem(\"Laboratory.Body Csys\"))");

          // XWind B.C.s
          double[] r_BwrtGround = {om_origin_rx, om_origin_ry, om_origin_rz};
          setInletToWind_BC(groundCsys, airSpeed, 
                            omega_ground[0], omega_ground[1], omega_ground[2],
                            r_BwrtGround[0], r_BwrtGround[1], r_BwrtGround[2],
                            kiteAlpha, kiteBeta, kitePhi);

          //setInletToLabZero_BC();
          setSwitchToOutlet_BC();

          // Alpha & Beta Field Functions & Point Probes
          UserFieldFunction mrf_alpha_FF = SimTool.getUserScalarFF(simu,"MRF Alpha Field");
          mrf_alpha_FF.setFunctionName("mrf_alpha_field");
          mrf_alpha_FF.setDefinition("atan2(-$$up_body[2],-$$up_body[0])*180/${pi}");
          
          UserFieldFunction mrf_beta_FF = SimTool.getUserScalarFF(simu,"MRF Beta Field");
          mrf_beta_FF.setFunctionName("mrf_beta_field");
          mrf_beta_FF.setDefinition("asin(-$$up_body[1]/mag($$up_body) )*180/${pi}");
          
          PointPart originProbe = DerivedPartTool.makePointPart(simu, "Body Origin", labCsys, zeroOrigin);
          Region region_0 = 
            simu.getRegionManager().getRegion("Kite");
          originProbe.getInputParts().setObjects(region_0);
          MaxReport originAlpha  = ReportTool.maxReport(simu, "Origin Alpha", mrf_alpha_FF, Collections.singleton(originProbe), simProxy);
          MaxReport originBeta  = ReportTool.maxReport(simu, "Origin Beta", mrf_beta_FF, Collections.singleton(originProbe), simProxy);
          originAlpha.setSmooth(true);
          originBeta.setSmooth(true);

          simu.getMonitorManager().createMonitorAndPlot(new NeoObjectVector(new Object[] {originAlpha, originBeta}), true, "Alpha/Beta Plot");
          ReportMonitor reportMonitor_0 = 
            ((ReportMonitor) simu.getMonitorManager().getMonitor("Origin Alpha Monitor"));
          ReportMonitor reportMonitor_1 = 
            ((ReportMonitor) simu.getMonitorManager().getMonitor("Origin Beta Monitor"));
          MonitorPlot monitorPlot_0 = 
            simu.getPlotManager().createMonitorPlot(new NeoObjectVector(new Object[] {reportMonitor_0, reportMonitor_1}), "Alpha/Beta Plot");
          
          // XWind Ref Frame for the kite and body regions
          applyXwindRotatingReferenceFrame(allBodyRegs);
          if(needRunRotors && needRunUnsteady){
            //Note: This is not the totally correct setup for a Rotor in MRF
            //      but it should be the correct setup for a XWind RBM Rotor Case
            applyXwindRotatingReferenceFrame(allBodyRegs);
          }
        }
        
        //PHYSICAL ROTOR METHODS
        if(needRunRotors){
          //enforce rotation rates
          setRotorSpeeds(rtr_1_low,rtr_2_low,rtr_3_low,rtr_4_low,
                         rtr_1_upp,rtr_2_upp,rtr_3_upp,rtr_4_upp,
                         needRunUnsteady);

        //BLADE ELEMENT METHOD SOLVER/PHYSICS UPDATE
        simu.println("M600 SOLVE: Checking if BEM enabled.");
        }else if(needRunBEM){
          simu.println("BEM MSG: Loading All BEM FOIL");
          ArrayList<FileTable> foilTables = getFileTables("BEM", "FOIL");
          simu.println("BEM MSG: Loading BEM CHORD");
          ArrayList<FileTable> chordVSrArr= getFileTables("BEM","CHORD");
          simu.println("BEM MSG: Loading BEM TWIST");
          ArrayList<FileTable> twistVSrArr= getFileTables("BEM","TWIST");
          if ((chordVSrArr.size()>1)|| (twistVSrArr.size()>1)){
            simu.println("BEM MSG: ERROR! Multiple CHORD or TWIST File Tables Found! ");
            simu.close();
          }
          FileTable[] chordVSr = chordVSrArr.toArray(new FileTable[chordVSrArr.size()]);
          FileTable[] twistVSr = twistVSrArr.toArray(new FileTable[twistVSrArr.size()]);
          simu.println("BEM MSG: Loading Rotor Coordinates under Solver");
          ArrayList<CartesianCoordinateSystem> allRotorCoords=getAllRotorCoords();
          for(CartesianCoordinateSystem rtrCoord:allRotorCoords){
            String coordName=rtrCoord.getPresentationName();
            RotatingMotion rtrRM=getRotatingMotion(coordName);
            double rtrRotSpeed;
            switch (coordName) {
              case "Rotor 1 Lower": rtrRotSpeed=rtr_1_low; break;
              case "Rotor 2 Lower": rtrRotSpeed=rtr_2_low; break;
              case "Rotor 3 Lower": rtrRotSpeed=rtr_3_low; break;
              case "Rotor 4 Lower": rtrRotSpeed=rtr_4_low; break;
              case "Rotor 1 Upper": rtrRotSpeed=rtr_1_upp; break;
              case "Rotor 2 Upper": rtrRotSpeed=rtr_2_upp; break; 
              case "Rotor 3 Upper": rtrRotSpeed=rtr_3_upp; break; 
              case "Rotor 4 Upper": rtrRotSpeed=rtr_4_upp; break; 
              default: rtrRotSpeed=0.0; break;
            }
            setRotatingMotionSpeed(rtrRM,rtrRotSpeed,0,0);

            //blade geometry
            int azRes=30; int radRes=30; //azimuthal and radial resolution of disk
            int nBlades=5; double bldThickness=0.1;
            double innerRad=0.32; double outerRad=1.16;
            VirtualDisk bemRtr=makeBEMRotor(allUsedFluidPhysics,coordName,rtrCoord,rtrRotSpeed,
                          azRes,radRes,nBlades,innerRad, outerRad,bldThickness,
                          foilTables, chordVSr[0],twistVSr[0]);
            //Create BEM Reports of Interest
            Report vdFxReport=bemForceReport(bemRtr,0);
            Report vdFyReport=bemForceReport(bemRtr,1);
            Report vdFzReport=bemForceReport(bemRtr,2);

            Report vdMxReport=bemMomentReport(bemRtr,0);
            Report vdMyReport=bemMomentReport(bemRtr,1);
            Report vdMzReport=bemMomentReport(bemRtr,2);

            Monitor bemFXMon=MonitorTool.reportIterationMonitor(simu,vdFxReport,maxSteadySteps,1,1);
            Monitor bemFYMon=MonitorTool.reportIterationMonitor(simu,vdFyReport,maxSteadySteps,1,1);
            Monitor bemFZMon=MonitorTool.reportIterationMonitor(simu,vdFzReport,maxSteadySteps,1,1);

            Monitor bemMXMon=MonitorTool.reportIterationMonitor(simu,vdMxReport,maxSteadySteps,1,1);
            Monitor bemMYMon=MonitorTool.reportIterationMonitor(simu,vdMxReport,maxSteadySteps,1,1);
            Monitor bemMZMon=MonitorTool.reportIterationMonitor(simu,vdMxReport,maxSteadySteps,1,1);
          }
        }//end rotor/bem selection

        // If unsteady get the time step
        if(needRunUnsteady){
          if(getRecommendedTimeStep()<unSteadyTimeStep) unSteadyTimeStep=getRecommendedTimeStep();
        }
        
        //Change all reports and Scenes to Volume Mesh
        simu.println("M600 SOLVE: Confirming proxy and setting elements to Volume Mesh.");
        setProxyToVolumeMesh();
        makeAllReportProxiesVolumeMesh();
        makeAllSceneProxiesVolumeMesh();
        setResidualsToAbsolute();
        setFlowFieldCoefficients();

        //=======================================================================================
        // Save & Run Solver
        // 
        // We desire to run the solver without checking on any reports
        //   and/or monitors until the solution is converged, or close to it.
        // Once the solution is converged, we are going to collect all the remaining data.
        // The data and reports will appear empty to a GUI-connected user, but this should make
        //   simulations run much faster.
        //=======================================================================================
        simu.println("M600 SOLVE: Setting up solver to run.");
        if(true) debugAlphaBetaValue();
        
        // Clear solution option if desired
        if(needClearSolution){
          simu.clearSolution();
          try{
            
          }catch(NeoException e){
            SolverStoppingCriterion maxSteps = simu
                .getSolverStoppingCriterionManager().getSolverStoppingCriterion("Maximum Steps");
            ((StepStoppingCriterion) maxSteps).setMaximumNumberSteps(0);
          }
        }

        // Make sure the casename is up to date for file saves
        simu.println("M600 SOLVE: Setting up for correct case name.");
        caseName = fileStringCompiler(needRunUnsteady,doesCaseNeedTransitionModel(kiteAlpha),needRunRotors);
        
        // Make sure any relevant post-processing directories exist
        // case's post-porcessing folder
        simu.println("M600 SOLVE: Setting up post-processing directories in case needed.");
        postSubFolder=caseName;
        studyPostDir = m600PostProcFolder+File.separator+postSubFolder+File.separator;
        simu.println("Study Post Processing Directories: ");
        simu.println("  Base: "+studyPostDir);
        SystemTool.touchDirectory(m600PostProcFolder);
        SystemTool.touchDirectory(studyPostDir);
        studyPostDir = m600PostProcFolder+File.separator+postSubFolder+File.separator;
        createPostProcDirectories(simu, studyPostDir);
        
        //make sure all annotations are up to date
        simu.println("M600 SOLVE: Initializing standard annotations.");
        initAnnotations(simu);
        
        // Set data to track
        simu.println("M600 SOLVE: Setting up relevant monitor outputs.");
        setOutputMonitorDisplay();
        // Run the case out to convergence
        simu.println("M600 SOLVE: Attempting to start solving...");
        if(needRunUnsteady){
          run_M600_DES_kwSST(unSteadyTimeStep, unSteadyInnterIterations,false,needRunRotors);
        }else{
          // Set up the Monitors to not track data for every step during the initial run
          MonitorTool.setAllReportBasedMonitorFrequency(simu, 1);

          // Automatically sets relevant criteria for primary kite based metrics
          // such as lift, drag, lateral force, and body moments.
          
          // We allow these kite stopping criterion the license to stop the
          // solver prematurely. If it does not stop the solver, then its clear
          // that the case has run out anyway, and that's when we collect the
          // data.
          ArrayList<MonitorIterationStoppingCriterion> activeKiteStopCrit = 
            setKiteBasedStoppingCriteria(needRunUnsteady, nDataSamples);

          // File name string compiler.
          String tmpCaseFileStr =
              fileStringCompiler(false, doesCaseNeedTransitionModel(kiteAlpha),
                  needRunRotors);
          
          // Run the RANS solver.
          run_M600_kwSST(maxSteadySteps, isCompressible,
              doesCaseNeedTransitionModel(kiteAlpha), needRunRotors);
        
          // Flag that this is now a solved case and save the solution.
          // We do this even if post-processing is enabled because the solve is
          // so expensive.
          setCaseCondition("SOLVED"); 
          setOldCaseName(tmpCaseFileStr);
          simu.saveState(simPathName + File.separator + tmpCaseFileStr + ".sim");
        }
      }
      //=======================================
      //      POST-PROCESSING
      //=======================================
      if(needQuickPost){
        // Set Body Csys to be aligned with any alpha_0, beta_0 configuration
        // Get alpha_0, beta_0
        kiteBeta_o = MeshOpTool.getRotationAngle(simu,"Kite Rotate Beta0");
        kiteAlpha_o = MeshOpTool.getRotationAngle(simu,"Kite Rotate Alpha0");
        simu.println("kiteBeta_o= " + kiteBeta_o);
        simu.println("kiteAlpha_o= " + kiteAlpha_o);
        
        simu.println("QUICK PROC: Quick post selected.");
        writeSimDataToSpreadsheetFormat(needRunUnsteady,
          CFD_Physics.isGRTinUse(allUsedFluidPhysics),needRunRotors,needRunBEM);

        quickPostFieldScenes();

      }else if(needRunPost){
        simu.println("POST PROC: Begin.");
        //Make sure post processing subfolder directories exist
        // case's post-porcessing folder
        studyPostDir = m600PostProcFolder+File.separator+postSubFolder+File.separator;
        simu.println("POST PROC: Study Post Processing Directories: ");
        simu.println("           Base Folder: "+studyPostDir);
        SystemTool.touchDirectory(m600PostProcFolder);
        SystemTool.touchDirectory(studyPostDir);

        //Set up inlet vector alpha and beta properly
        double[] tmpVect = getCaseFreeStreamParameters();
        this.airDensity = tmpVect[0];
        this.airSpeed   = tmpVect[1];
        this.kiteAlpha  = tmpVect[2];
        this.kiteBeta   = tmpVect[3];

        double[] omega_hats = getXWindOmegaReports();
        this.omega_hat_x=omega_hats[0];
        this.omega_hat_y=omega_hats[1];
        this.omega_hat_z=omega_hats[2];

        
        // Set proper Alpha/Beta for velocity inlet
        setInletVectorAlphaBeta();

        //set all coefficients properly
        setFlowFieldCoefficients();

        // Write out spreadsheet monitored data to csv file
        simu.println("M600 POST: Writing data to file");
        writeSimDataToSpreadsheetFormat(needRunUnsteady,
          CFD_Physics.isGRTinUse(allUsedFluidPhysics),needRunRotors,needRunBEM);

        // Case Name Annotation Update
        simu.println("M600 POST: Updating case name annotation.");
        caseName = fileStringCompiler(needRunUnsteady,
                doesCaseNeedTransitionModel(kiteAlpha),needRunRotors);
        //make sure all annotations are up to date
        initAnnotations(simu);
        simu.println("M600 POST: Case name annotation is "+studyName+": "+caseName);

        // Make sure the relevant directories exist
        simu.println("M600 POST: Creating post processing directories.");
        createPostProcDirectories(simu, studyPostDir);

        // post process pitotTubes to their own csv file
        simu.println("M600 POST: Post processing pitot probes to CSV.");
        String pitotFileName ="AllPitotAveragedValues.CSV";
        postProcessMeanPitotTubeValuesToCSVFile(studyPostDir,pitotFileName,nDataSamples);                        

        // plots
        simu.println("M600 POST: Changing residuals to absolute values");
        setResidualsToAbsolute();
        String appEnd="It";
        if(needRunUnsteady){
            appEnd="UNS";
        }

        simu.println("M600 POST: Post processing plots.");
        postProcessPlots(appEnd,true);
        
        //Set up Derived Parts
        simu.println("M600 POST: setting up derived parts.");
        setupDerivedParts(allBodyRegs);
        //Scenes
        simu.println("M600 POST: Setting up Scene views.");
        setStandardKiteViews();
        //
        simu.println("M600 POST: Setting up Kite body scenes.");
        ArrayList<Scene> bodyScenes = setupKiteScenes(needRunRotors);
        //
        simu.println("M600 POST: Setting up Kite field scenes.");
        ArrayList<Scene> fieldScenes = setupFieldScenes(needRunRotors);
        // Inject rotors into scenes if needed
        ArrayList<Scene> rotorScenes = new ArrayList();
        if(needRunRotors){
          simu.println("M600 POST: Setting up Rotor scenes views.");
          rotorScenes = setupRotorScenes();
        }
        // Close all scenes before post-processing
        // This speeds up post-processing and prevents memory problems.
        for(Scene tmpScene:simu.getSceneManager().getObjects()){
            tmpScene.close(true);
        }

        //Set all scene proxies to Volume Mesh
        simu.println("M600 POST: Setting up Rotor scenes views.");
        setProxyToVolumeMesh();
 
        //Post process Field Scenes
        postProcessFieldScenes(fieldScenes);
      
        //Printout of scenes with *tons* of views
        // This method must be first, otherwise everything gets printed out
        //   add special scene post-processing below
        //
        simu.println("M600 POST: Turning on advanced rendering for all scenes.");
        for(Scene tmpScene:bodyScenes){
          tmpScene.setAdvancedRenderingEnabled(true);
        }
        for(Scene tmpScene:fieldScenes){
          tmpScene.setAdvancedRenderingEnabled(true);
        }
        for(Scene tmpScene:rotorScenes){
          tmpScene.setAdvancedRenderingEnabled(true);
        }
        //
        simu.println("M600 POST: Post processing Body scenes.");
        postProcessBodyScenes(bodyScenes);

        if(needRunRotors) postProcessRotorScenes(rotorScenes);

        // Special Scene Post Processing
        long initTime = System.nanoTime();
//        postYPlaneSlices(fieldScenes);
        long endTime = System.nanoTime();
        double elapsedSec = ((double)(endTime-initTime))/1.0E9;
//        simu.println(String.format("Total Time Spent in postYPlaneSlices: %1$10.3f (s)",(elapsedSec)));

        initTime = System.nanoTime();
 //       postYWingSlices(fieldScenes);
        endTime = System.nanoTime();
        elapsedSec = ((double)(endTime-initTime))/1.0E9;
//        simu.println(String.format("Total Time Spent in postYWingSlices: %1$10.3f (s)",(elapsedSec)));
        //
        initTime = System.nanoTime();
//        postZPlaneSlices(fieldScenes);
        endTime = System.nanoTime();
        elapsedSec = ((double)(endTime-initTime))/1.0E9;
//        simu.println(String.format("Total Time Spent in postZPlaneSlices: %1$10.3f (s)",(elapsedSec)));
        //
        initTime = System.nanoTime();
//        postYFlapSlices(fieldScenes);
        endTime = System.nanoTime();
        elapsedSec = ((double)(endTime-initTime))/1.0E9;
//        simu.println(String.format("Total Time Spent in postYFlapSlices: %1$10.3f (s)",(elapsedSec)));

        //
        if(needRunSolver){
          simu.saveState(simPathName+File.separator+caseName+".sim");
        }else{
          // this case was just being loaded to post processing - no need to save
          simu.saveState(simPathName+File.separator+caseName+".sim");
        }
      }
  }
  
  //==============================================
  // File I/O Utilities
  //==============================================
  private String getSubFolderName(){
    /* getSubFolderName returns the name of the subFolder
        from which the simulation was launched.
        This is referred to as the study directory.
    */
    int lastIndx=simPathName.lastIndexOf(File.separator);
    return simPathName.substring(lastIndx+1);
  }
  public void createPostProcDirectories(Simulation tmpSim, String folderPath){
    SystemTool.touchDirectory(folderPath);
    
    // BUILD CFD and USER Directories
    String userPath = folderPath + "USER" + File.separator;
    String cfdPath = folderPath + "CFD" + File.separator;
    //
    SystemTool.touchDirectory(userPath + File.separator);
    SystemTool.touchDirectory(cfdPath + File.separator);
    
    //USER IMAGE PATH
    this.userPlots  = userPath + "PLOTS" + File.separator;
    this.userScenes = userPath + "SCENES" + File.separator;
    this.userCSV    = userPath + "CSV" + File.separator;

    SystemTool.touchDirectory(userPlots);
    SystemTool.touchDirectory(userCSV);
    SystemTool.touchDirectory(userScenes);

    //CFD IMAGE PATH
    this.cfdPlots   = cfdPath + "PLOTS"  + File.separator;
    this.cfdScenes  = cfdPath + "SCENES" + File.separator;
    this.cfdCSV     = cfdPath + "CSV"    + File.separator;

    SystemTool.touchDirectory(cfdPlots);
    SystemTool.touchDirectory(cfdCSV);
    SystemTool.touchDirectory(cfdScenes);
    
  }
  
  //==============================================
  // GEOMETRY
  //==============================================
  private void replaceKitePart(String kiteCompositeName,String xtFileName){
    // delete old Kite
    CompositePart kiteCompositePart = 
      ((CompositePart) simu.get(SimulationPartManager.class).getPart(kiteCompositeName));
    try{
        GeometryPart deletedPart = (GeometryPart) kiteCompositePart.getChildParts().getPart("Kite");
        kiteCompositePart.getChildParts().removeParts(new NeoObjectVector(new Object[] {deletedPart}));
    }catch (NeoException e){
    }
    // List of All Other Parts
    Collection<GeometryPart> oldOtherParts = simu.getGeometryPartManager().getParts();

    // Import new Kite .x_t or .x_b
    PartImportManager partImportManager = simu.get(PartImportManager.class);
    try{
        simu.println("Trying x_t");
    partImportManager.importCadPart(resolvePath(simPathName+File.separator+
        xtFileName+".x_t"), "SharpEdges", 30.0, 3, true, 1.0E-5, true, false);
    }catch(NeoException e){
        simu.println("Trying x_b");
    partImportManager.importCadPart(resolvePath(simPathName+File.separator+
        xtFileName+".x_b"), "SharpEdges", 30.0, 3, true, 1.0E-5, true, false);
    }

    Collection<GeometryPart> allNewParts = simu.getGeometryPartManager().getParts();
    allNewParts.removeAll(oldOtherParts);
    GeometryPart newKitePart = (GeometryPart) allNewParts.toArray()[0];
    newKitePart.setPresentationName("Kite");
    newKitePart.reparent(kiteCompositePart.getChildParts());
  }

  //==============================================
  // KITE SPECIFIC MESH METHODS
  //==============================================
  private void createNeededSurfVCs(){
      // Winglet Cutters for CAD geometry with winglets.
      CartesianCoordinateSystem cutter1 = SimTool.getNestedCoordinate(labCsys, "Winglet Cutter Flap A1");
      modifyCoordSys("Winglet Cutter Flap A1",
                  new double[]{-0.623709437324682, 0.013175164867327957, -0.7815452340222205},
                  new double[]{0.7777003349372436, 0.11092595768086051, -0.6187710569759087},
                  new double[]{-0.37954655215586963, -12.680972078439861, -0.41376925946384674});
      CartesianCoordinateSystem cutter8 = SimTool.getNestedCoordinate(labCsys, "Winglet Cutter Flap A8");
      modifyCoordSys("Winglet Cutter Flap A8",
                  new double[]{-0.023655564012555908, -0.07778751810047807, -0.9966892777184952},
                  new double[]{-0.9966319876344636, 0.08013692409709927, 0.01739984540259725},
                  new double[]{-0.39794654544796904, 12.683005171699609, -0.40652363699220345});
      GeometryTool.makeBlock(simu,"Cutter Winglet IB",cutter1,
                  new double[]{-0.6290356180152046, -1.0393559924801024, -0.01610308012053102},
                  new double[]{1.0938676207476545, 0.9043766957125208, 0.8124492667466374});
      GeometryTool.makeBlock(simu,"Cutter Winglet OB",cutter8,
                  new double[]{-0.9806262502831782, -1.5998566066286264, -0.011402882091091637},
                  new double[]{1.3414421654179454, 0.35592864220970455, 1.4286810323220762});

      // M600 - invT7 - Best Practices - SURFACE MESH CONTROL BLOCKS
      CartesianCoordinateSystem Flap_A1A2_Axis = SimTool.getLabBasedCoordinate(simu, "Surf CAD Flap A1A2 Axis");
      CartesianCoordinateSystem Flap_A3A6_Axis = SimTool.getLabBasedCoordinate(simu, "Surf CAD Flap A3A6 Axis");
      CartesianCoordinateSystem Flap_A7A8_Axis = SimTool.getLabBasedCoordinate(simu, "Surf CAD Flap A7A8 Axis");
      GeometryTool.makeBlock(simu,"Surf A1A2 Intersect",Flap_A1A2_Axis,
              new double[] { -6.870,-0.025,-0.25 },
              new double[] { -0.585, 0.175,-0.155});
      GeometryTool.makeBlock(simu,"Surf A3A6 Intersect",Flap_A3A6_Axis,
              new double[] { -3.4, -0.05, -0.2},
              new double[] { 3.4, 0.125, -0.15});           
      GeometryTool.makeBlock(simu,"Surf A7A8 Intersect",Flap_A7A8_Axis,
              new double[] {  9.25,-0.025,-0.25},
              new double[] {  2.50,0.175,-0.155});     

      GeometryTool.makeBlock(simu,"Surf Pylon 1 To Wing",labCsys,
              new double[] {  0.115,-4.00,-0.01 },
              new double[] { 0.275,-3.60,0.06});
      GeometryTool.makeBlock(simu,"Surf Pylon 2 To Wing",labCsys,
              new double[] {  0.115,-1.55,-0.01 },
              new double[] {  0.275,-1.15,0.06});
      GeometryTool.makeBlock(simu,"Surf Pylon 3 To Wing",labCsys,
              new double[] {   0.115,1.25,-0.01 },
              new double[] {  0.275,0.85,0.06 });
      GeometryTool.makeBlock(simu,"Surf Pylon 4 To Wing",labCsys,
              new double[] {  0.115,3.3,-0.01 },
              new double[] {  0.275,3.7,0.06 });
  }
  private void createMeshSensitiveVCs(){
    // This should only be called before mesh generation
    // M600 - invT7 - Best Practices - VOLUME MESH CONTROL BLOCKS
    CartesianCoordinateSystem Flap_A1A2_Axis;
    CartesianCoordinateSystem Flap_A3A6_Axis;
    CartesianCoordinateSystem Flap_A7A8_Axis;

    // Some Volume controls need to rotate with mesh Alpha and mesh Beta
    // angles, but some do not. Candidates are volume controls near the kite
    // body, but ones that do not need to change are wakes:
    GeometryTool.makeBlock(simu, "VC Near Wake", labCsys,
            new double[] {-30.0, -6.5, -3.},
            new double[] {  4.0,  6.5,  5.0});

    // If the kite is solved in a moving reference frame, the VC must capture
    // the curvature of the kite wake
    if(isCrossWindMRFCase){
      // kite wake - we want this in body csys to capture tip vortices cleanly
      ArrayList<MeshPart> tmpAL = new ArrayList();
      // try to always keep the wing tip of the M600 encapsulated even when rotated
      // +-12.5 takes it to kitePhi=30 deg
      double minZ = -12.5;
      double maxZ =  12.5;
      CadPart tmpDonut = GeometryTool.getDonut(simu, groundCsys, 120.0, 80.0, minZ, maxZ);
      double bigNum = 200;
      SimpleBlockPart tmp1 = GeometryTool.makeBlock(simu, "CUT AHEAD", groundCsys,
              new double[] {    0.0,    0.0, -bigNum},
              new double[] {  bigNum, bigNum, bigNum});
      SimpleBlockPart tmp2 = GeometryTool.makeBlock(simu, "CUT PORT", groundCsys,
              new double[] { -bigNum,    0.0, -bigNum},
              new double[] {  bigNum, -bigNum,  bigNum});
      SimpleBlockPart tmp3 = GeometryTool.makeBlock(simu, "CUT AFT", groundCsys,
              new double[] {  -50.0,  bigNum, -bigNum},
              new double[] { -bigNum,-bigNum,  bigNum});
      tmpAL.add(tmpDonut);
      tmpAL.add(tmp1);
      tmpAL.add(tmp2);
      tmpAL.add(tmp3);
      CadPart mrfVCkiteWake = GeometryTool.makeDummyCADSubtractPart(simu, tmpAL, tmpDonut);
      mrfVCkiteWake.setPresentationName("VC Kite Wake");
      simu.get(SimulationPartManager.class)
            .removeParts(new NeoObjectVector(
                    new Object[] {tmpDonut, tmp1, tmp2, tmp3}));
      tmpAL.clear();

      // far wake - we want this in the lab to adequately capture the convection
      // of the trailing edge vortices and attempt to reduce grid induced diffusion
      // in the far field. it is wide to capture high beta conditions
      simu.println("M600 MESH: Generating Far Wake Block.");
      minZ = -15.0;
      maxZ =  15.0;
      tmpDonut = GeometryTool.getDonut(simu, groundCsys, 140.0, 60.0, minZ, maxZ);
      tmp1 = GeometryTool.makeBlock(simu, "CUT AHEAD", groundCsys,
              new double[] {    5.0, -bigNum, -400.0},
              new double[] {  bigNum, bigNum,  400.0});
      tmp2 = GeometryTool.makeBlock(simu, "CUT PORT", groundCsys,
              new double[] { -bigNum,     0.0, -bigNum},
              new double[] {  bigNum, -bigNum, bigNum});
      // no tmp3 as this goes to the exit boundary
      tmpAL.add(tmpDonut);
      tmpAL.add(tmp1);
      tmpAL.add(tmp2);
      CadPart mrfVCFarWake = GeometryTool.makeDummyCADSubtractPart(simu, tmpAL, tmpDonut);
      mrfVCFarWake.setPresentationName("VC Far Wake");
      simu.get(SimulationPartManager.class)
            .removeParts(new NeoObjectVector(
                    new Object[] {tmpDonut, tmp1, tmp2}));
    }else{
      try{ // is case fresh?
        simu.println("M600 MESH: Attempting to set up SLF Wake VCs.");
        GeometryTool.makeBlock(simu, "VC Kite Wake", labCsys,
                new double[] {-55.0, -20.0, -10.0},
                new double[] {  0.0,  20.0,  10.0});
        GeometryTool.makeBlock(simu, "VC Far Wake", labCsys,
                new double[] { -55.0, -20.0, -10.0},
                new double[] {-175.0,  20.0,  10.0});
        simu.println("M600 MESH: Completed set up of SLF Wake VCs.");
      }catch(NeoException e){ // do the mrf parts exist?
        simu.println("M600 MESH: This case already appears to have MRF Volume Controls.");
        GeometryPart tmpPart1 = simu.getGeometryPartManager().getPart("VC Kite Wake");
        GeometryPart tmpPart2 = simu.getGeometryPartManager().getPart("VC Kite Wake");
        simu.println("M600 MESH: Attempting removal!");
        simu.get(SimulationPartManager.class)
            .removeParts(new NeoObjectVector(
                new Object[] {tmpPart1,tmpPart2}));
        simu.println("M600 MESH: Parts removed. Recreating SimpleBlockParts for SLF");
        
        // try again
        GeometryTool.makeBlock(simu, "VC Kite Wake", labCsys,
                new double[] {-55.0, -20.0, -10.0},
                new double[] {  0.0,  20.0,  10.0});
        GeometryTool.makeBlock(simu, "VC Far Wake", labCsys,
                new double[] { -55.0, -20.0, -10.0},
                new double[] {-175.0,  20.0,  10.0});
      }
    }

    // Need to understand if the Volume Controls need to Rotate w/ the Body_0 angles or not
    simu.println("M600 MESH: Assigning VCs to rotated with the body angles... or not.");
    CartesianCoordinateSystem whichCSYS;
    if( (Math.abs(kiteAlpha_o) > SMALL_EPS) || (Math.abs(kiteBeta_o) > SMALL_EPS) ){
      simu.println("M600 MESH: Large deflection meshing detected. Setting nested coordinates.");
      // if the kite CAD rotation angle is non-zero, then we need to make sure
      // that the flap gap volume controls rotate as well.
      whichCSYS = meshAngleCsys;
      Flap_A1A2_Axis = SimTool.getNestedCoordinate(meshAngleCsys, "CAD Flap A1A2 Axis");
      Flap_A3A6_Axis = SimTool.getNestedCoordinate(meshAngleCsys, "CAD Flap A3A6 Axis");
      Flap_A7A8_Axis = SimTool.getNestedCoordinate(meshAngleCsys, "CAD Flap A7A8 Axis");
      
    }else{ // no CFD controlled CAD rotation, can just use the basic Lab
      simu.println("M600 MESH: No deflection meshing detected. Setting lab coordinates.");
      whichCSYS = zeroCADAngleBodyCsys;
      Flap_A1A2_Axis = SimTool.getLabBasedCoordinate(simu, "Lab CAD Flap A1A2 Axis");
      Flap_A3A6_Axis = SimTool.getLabBasedCoordinate(simu, "Lab CAD Flap A3A6 Axis");
      Flap_A7A8_Axis = SimTool.getLabBasedCoordinate(simu, "Lab CAD Flap A7A8 Axis");
    }
    simu.println("M600 MESH: Setting up flow gaps VC.");
    // flap gaps flow field
    GeometryTool.makeBlock(simu, "VC A1A2 Flap Gap", Flap_A1A2_Axis,
            new double[] { -6.82, -0.04, 0.0},
            new double[] {-0.775, -0.12, 0.1});
    GeometryTool.makeBlock(simu, "VC A3A6 Flap Gap", Flap_A3A6_Axis,
            new double[] { -6.65, -0.06, 0.025},
            new double[] {  6.65, -0.12, 0.1});        
    GeometryTool.makeBlock(simu, "VC A7A8 Flap Gap", Flap_A7A8_Axis,
            new double[] { 3.1, -0.04, 0.025},
            new double[] { 9.15, -0.12, 0.11});

    simu.println("M600 MESH: Setting up slat gap VC.");
    // slat flow field
    try{
      GeometryPart tmpPart1 = simu.getGeometryPartManager().getPart("VC_Slat_3.125");
      simu.println("M600 MESH: Found possible leaf leve slat dummy CAD part. Deleting...");
      simu.println("M600 MESH: Attempting removal!");
      simu.get(SimulationPartManager.class)
          .removeParts(new NeoObjectVector(
              new Object[] {tmpPart1}));
    }catch(NeoException e){
      simu.println("M600 MESH: Recreating slat block based VC. ");
      GeometryTool.makeBlock(simu, "VC_Slat_3.125", whichCSYS,
              new double [] { 0.36569857597351074, -3.950000047683716, -0.4486660361289978},
              new double [] { 0.7294485569000244, 3.6500000953674316, 0.04747813194990158});
    }
    // pylon flow field
    simu.println("M600 MESH: Setting up slat all pylon VCs.");
    GeometryTool.makeBlock(simu, "VC Pylon 1 to Wing", whichCSYS,
            new double[] {-0.54, -4.18, -0.53},
            new double[] { 0.70, -3.45,  0.24});     

    GeometryTool.makeBlock(simu, "VC Pylon 2 to Wing", whichCSYS,
            new double[] {-0.55, -1.72, -0.52},
            new double[] {0.685, -0.985, 0.25});     

    GeometryTool.makeBlock(simu, "VC Pylon 3 to Wing", whichCSYS,
            new double[] {-0.41, 0.655, -0.618},
            new double[] {0.76, 1.385, 0.1535});
    GeometryTool.makeBlock(simu, "VC Pylon 4 to Wing", whichCSYS,
            new double[] {-0.448, 3.126, -0.372},
            new double[] { 0.833, 3.856,  0.389});

    // body flow field
    GeometryTool.makeBlock(simu, "VC Main Wing", whichCSYS,
            new double[] {-1.5, -13.5, -0.7},
            new double[] { 0.5,  13.5,  0.4});

    GeometryTool.makeBlock(simu,"VC Center Fuselage to Main Wing", whichCSYS,
            new double[] {-0.535, -0.362, -0.535},
            new double[] { 0.70,   0.368, 0.237});

    simu.println("M600 MESH: Mesh sensitive VC setup complete.");
  }
  
  private void setupMeshOperationTree(double charBaseSize,boolean needRunRotors){
    /* Method setupMeshOperationTree
        Sets up the mesh operation tree and dependencies required 
    */
    String strKiteAssembly="CFD Kite Parts";
    String strRadiatorAssembly ="CFD Kite Parts";
    String strControlsAssembly ="CFD Control Parts";
    String strRotorBladeAssembly = "CFD Rotor Parts";

    String strFarfieldPart="Farfield";
    String strFirstKiteBodyPart = "Kite";
    String strRotorIntAssembly ="Rotor Interface Parts";

    //
    // Define Rotor CAD Interface
    ArrayList<GeometryPart> rotorIntGeom = new ArrayList();
    CompositePart rotorIntAssembly = 
      ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRotorIntAssembly));
    for(int i=1;i<=4;i++){
        rotorIntGeom.add((SimpleCylinderPart) rotorIntAssembly.getChildParts().getPart("Rotor "+i+" Lower Interface"));
        rotorIntGeom.add((SimpleCylinderPart) rotorIntAssembly.getChildParts().getPart("Rotor "+i+" Upper Interface"));
    }
    // Define Rotor CAD Blades
    ArrayList<GeometryObject> rotorBladeGeom = new ArrayList();
    if(needRunRotors){
        CompositePart bladesAssembly = 
          ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRotorBladeAssembly));
        for(int i=1;i<=4;i++){
            rotorBladeGeom.add(bladesAssembly.getChildParts().getPart("Rotor "+i+" Lower"));
            rotorBladeGeom.add(bladesAssembly.getChildParts().getPart("Rotor "+i+" Upper"));
        }
    }
    // Define Radiator CAD Bodies
    ArrayList<GeometryPart> radiatorGeom = new ArrayList();
    CompositePart radiatorAssembly = 
      ((CompositePart) simu.get(SimulationPartManager.class).getPart(strRadiatorAssembly));
    for(int i=1;i<=4;i++){
        radiatorGeom.add(radiatorAssembly.getChildParts().getPart("Radiator "+i+" Lower"));
        radiatorGeom.add(radiatorAssembly.getChildParts().getPart("Radiator "+i+" Upper"));
    }
    // Define Control Surfaces CAD geometry
    ArrayList<GeometryObject> controlsGeom = new ArrayList();
    ArrayList<GeometryObject> flapGeom = new ArrayList();
    CompositePart controlsAssmb = 
      ((CompositePart) simu.get(SimulationPartManager.class).getPart(strControlsAssembly));
    controlsGeom.add(controlsAssmb.getChildParts().getPart("H Tail"));
    controlsGeom.add(controlsAssmb.getChildParts().getPart("Rudder"));
    for(int i=1;i<=8;i++){
        flapGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Flap A"+i));
        controlsGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Flap A"+i));
    }
    // slat
    try{
        flapGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Slat"));
        controlsGeom.add((GeometryPart) controlsAssmb.getChildParts().getPart("Slat"));
    }catch(NeoException e){
      simu.println("no slat parts detected");
    }

      //
      //======================================================
      // Initialize Operations:
      //   Recommended best practice default values
      //   Associate Parts
      //======================================================
      CompositePart kiteBodyAssmb = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strKiteAssembly));
      if(needRunRotors){
          // Geometry Manipulation 1st
          GeometryTool.makeBlock(simu,"Cutter Make Plug",labCsys,
                  new double[] {-10.14, -15, -6.2},
                  new double[] {0.780, 15.0, 4.1755});
          SubtractPartsOperation rotorCADPlug = setupSubtractParts("Rotors CAD Plug",false);
          rotorCADPlug.setPerformCADBoolean(true);
          rotorCADPlug.getInputGeometryObjects().setQuery(null);
          rotorCADPlug.getInputGeometryObjects().setObjects(kiteBodyAssmb.getChildParts().getPart(strFirstKiteBodyPart),
                  simu.get(SimulationPartManager.class).getPart("Cutter Make Plug"));
          rotorCADPlug.setTargetPart((MeshPart) kiteBodyAssmb.getChildParts().getPart(strFirstKiteBodyPart));
      }
      // PBM Operations
      // farfield
      CompositePart farfieldAssmb = 
        ((CompositePart) simu.get(SimulationPartManager.class).getPart(strKiteAssembly));
      AutoMeshOperation farfieldRemesh = setupSurfPBMO("Farfield Remesh",1.6,800.0,200.0,36.0,1.2); 
      farfieldRemesh.getInputGeometryObjects().setQuery(null);
      farfieldRemesh.getInputGeometryObjects().setObjects(farfieldAssmb.getChildParts().getPart(strFarfieldPart));
      // rotor interfaces
      if(needRunRotors){
          AutoMeshOperation rotorInterfaceRemesh = setupSurfPBMO("Rotors Remesh Rotor Interfaces",4.0,0.5,0.2,36.0,1.5);
          rotorOperations.add(rotorInterfaceRemesh);
          rotorInterfaceRemesh.getInputGeometryObjects().setQuery(null);
          rotorInterfaceRemesh.getInputGeometryObjects().setObjects(rotorIntGeom);
      }
      // kite main body
      AutoMeshOperation kiteRemesh = setupSurfPBMO("Kite Remesh",charBaseSize,100.0,12.5,36.0,1.1);
      kiteRemesh.getInputGeometryObjects().setQuery(null);
      kiteRemesh.getInputGeometryObjects().setObjects(kiteBodyAssmb.getChildParts().getPart(strFirstKiteBodyPart));
      // radiators remesh - not needed
      /*
      AutoMeshOperation radiatorsRemesh = setupSurfPBMO("Radiators Remesh",charBaseSize,100.0,6.25,36.0,1.1);
      radiatorsRemesh.getInputGeometryObjects().setQuery(null);
      radiatorsRemesh.getInputGeometryObjects().setObjects(radiatorGeom);
      radiatorsRemesh.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.CONCURRENT);
      radiatorsRemesh.setMeshPartByPart(true);
      */
      // contols remesh
      AutoMeshOperation controlsRemesh = setupSurfPBMO("Controls Remesh",charBaseSize,100.0,12.5,36.0,1.1);
      controlsRemesh.getInputGeometryObjects().setQuery(null);
      controlsRemesh.getInputGeometryObjects().setObjects(controlsGeom);
      controlsRemesh.setMeshPartByPart(true);
      controlsRemesh.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.CONCURRENT);

      // controls rotate flaps, slat, htail, vtail
      TransformPartsOperation Flap_A1_Rot = setupRotTransform("Kite Rotate Flap A1","Lab CAD Flap A1A2 Axis",xOnlyAxis);
      Flap_A1_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A1_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A1"));
      //
      TransformPartsOperation Flap_A2_Rot = setupRotTransform("Kite Rotate Flap A2","Lab CAD Flap A1A2 Axis",xOnlyAxis);
      Flap_A2_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A2_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A2"));
      //
      TransformPartsOperation Flap_A3_Rot = setupRotTransform("Kite Rotate Flap A3","Lab CAD Flap A3A6 Axis",xOnlyAxis);
      Flap_A3_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A3_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A3"));
      //
      TransformPartsOperation Flap_A4_Rot = setupRotTransform("Kite Rotate Flap A4","Lab CAD Flap A3A6 Axis",xOnlyAxis);
      Flap_A4_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A4_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A4"));
      //
      TransformPartsOperation Flap_A5_Rot = setupRotTransform("Kite Rotate Flap A5","Lab CAD Flap A3A6 Axis",xOnlyAxis);
      Flap_A5_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A5_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A5"));
      //
      TransformPartsOperation Flap_A6_Rot = setupRotTransform("Kite Rotate Flap A6","Lab CAD Flap A3A6 Axis",xOnlyAxis);
      Flap_A6_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A6_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A6"));
      //
      TransformPartsOperation Flap_A7_Rot = setupRotTransform("Kite Rotate Flap A7","Lab CAD Flap A7A8 Axis",xOnlyAxis);
      Flap_A7_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A7_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A7"));
      //
      TransformPartsOperation Flap_A8_Rot = setupRotTransform("Kite Rotate Flap A8","Lab CAD Flap A7A8 Axis",xOnlyAxis);
      Flap_A8_Rot.getInputGeometryObjects().setQuery(null);
      Flap_A8_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Flap A8"));
      //
      try{
          TransformPartsOperation Slat_Rot = setupRotTransform("Kite Rotate Slat","Lab CAD Slat Axis",xOnlyAxis);
          Slat_Rot.getInputGeometryObjects().setQuery(null);
          Slat_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Slat"));
      }catch(NeoException e){

      }
      //
      TransformPartsOperation Htail_Rot = setupRotTransform("Kite Rotate H Tail","Lab CAD H Tail Axis",xOnlyAxis);
      Htail_Rot.getInputGeometryObjects().setQuery(null);
      Htail_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("H Tail"));
      //
      TransformPartsOperation Vtail_Rot = setupRotTransform("Kite Rotate Rudder","Lab CAD Rudder Axis",xOnlyAxis);
      Vtail_Rot.getInputGeometryObjects().setQuery(null);
      Vtail_Rot.getInputGeometryObjects().setObjects(controlsAssmb.getChildParts().getPart("Rudder"));
      //create final remeshed kite body
      UnitePartsOperation kiteBody = setupUniteParts("Kite CFD Body",true);
      kiteBody.getInputGeometryObjects().setQuery(null);
      Collection<GeometryObject> combineKite = new ArrayList();
      combineKite.addAll(controlsGeom);
      combineKite.add(kiteBodyAssmb.getChildParts().getPart(strFirstKiteBodyPart));
      kiteBody.getInputGeometryObjects().setObjects(combineKite);
      //rotor blade remeshing

      AutoMeshOperation rotorsRemesh;
      AutoMeshOperation rotorPlugRemesh;
      SubtractPartsOperation rotor1_low=null;SubtractPartsOperation rotor1_upp=null;
      SubtractPartsOperation rotor2_low=null;SubtractPartsOperation rotor2_upp=null;
      SubtractPartsOperation rotor3_low=null;SubtractPartsOperation rotor3_upp=null;
      SubtractPartsOperation rotor4_low=null;SubtractPartsOperation rotor4_upp=null;
      if(needRunRotors){
        rotorsRemesh = setupSurfPBMO("Rotors Props Remesh",1.0,100.0,1.0,36.0,1.3);
        rotorOperations.add(rotorsRemesh);
        rotorsRemesh.getInputGeometryObjects().setQuery(null);
        rotorsRemesh.getInputGeometryObjects().setObjects(rotorBladeGeom);
        rotorsRemesh.setMeshPartByPart(true);
        rotorsRemesh.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.CONCURRENT);
        //  ROTORS - SURFACE Custom Mesh Controls
        SurfaceCustomMeshControl sC_Rtr_Blades     = MeshOpTool.surfControl(rotorsRemesh,"800 Blades");
          MeshOpTool.surfFilter( sC_Rtr_Blades,1);
          MeshOpTool.surfCustTargSize(sC_Rtr_Blades,"Relative",0.2);
          MeshOpTool.surfCustMinSize( sC_Rtr_Blades,"Relative",0.05);
        SurfaceCustomMeshControl sC_Rtr_Blades_TE     = MeshOpTool.surfControl(rotorsRemesh,"800 Blades TE");
          MeshOpTool.surfFilter_TE( sC_Rtr_Blades_TE,1);  
          MeshOpTool.surfCustMinSize( sC_Rtr_Blades_TE,"Relative",0.01);
          MeshOpTool.surfEdgeProx(  sC_Rtr_Blades_TE,1.0);
        //
        // ROTOR PLUG - CREATE MESH
        rotorPlugRemesh = setupSurfPBMO("Rotors Remesh Plug",charBaseSize,100.0,1.0,36.0,1.3);
        rotorOperations.add(rotorPlugRemesh);
        rotorPlugRemesh.getInputGeometryObjects().setQuery(null);
        rotorPlugRemesh.getInputGeometryObjects().setObjects(simu.get(SimulationPartManager.class).getPart("Rotors CAD Plug"));
        //  ROTOR PLUG - Create Filtered Custom Mesh Controls
        SurfaceCustomMeshControl sC_Plug_Spinners  = MeshOpTool.surfControl(rotorPlugRemesh,"4X8 Pylon Spinners");
          surf2ndWildFilter(sC_Plug_Spinners);
           MeshOpTool.surfCustTargSize(sC_Plug_Spinners, "Relative",6.25);
          MeshOpTool.surfCustMinSize(sC_Plug_Spinners,"Relative",3.125);
        //
        TransformPartsOperation Rot_Plug_To_Rotors = setupRotTransform("Rotors Rotate Plug to Rotors","Lab_Csys",yOnlyAxis);
        Rot_Plug_To_Rotors.getInputGeometryObjects().setQuery(null);
        Rot_Plug_To_Rotors.getInputGeometryObjects().setObjects(simu.get(SimulationPartManager.class).getPart("Rotors CAD Plug"));
        setTransformRotationAngle("Rotors Rotate Plug to Rotors", labCsys,100.0,zeroOrigin,yOnlyAxis);
        // unite rotors to plug
        UnitePartsOperation rotorWithPlug = setupUniteParts("Rotors Join Rotors to Plug",true);
        rotorOperations.add(rotorWithPlug);
        rotorWithPlug.getInputGeometryObjects().setQuery(null);
        Collection<GeometryObject> combineRotors = new ArrayList();
        combineRotors.addAll(rotorBladeGeom);
        combineRotors.add(simu.get(SimulationPartManager.class).getPart("Rotors CAD Plug"));
        rotorWithPlug.getInputGeometryObjects().setObjects(combineRotors);
        GeometryPart rotorWithPlugPart =
                simu.get(SimulationPartManager.class).getPart(rotorWithPlug.getOutputPartNames()
                        .substring(1,rotorWithPlug.getOutputPartNames().length()-1));
        //anti-rotate plug to Rotors
        TransformPartsOperation antiRot_Plug_To_Rotors = setupRotTransform("Rotors Anti-Rotate Plug w Rotors","Lab_Csys",yOnlyAxis);
        antiRot_Plug_To_Rotors.getInputGeometryObjects().setQuery(null);
        antiRot_Plug_To_Rotors.getInputGeometryObjects().setObjects(rotorWithPlugPart);
        setTransformRotationAngle("Rotors Anti-Rotate Plug w Rotors", labCsys,-100.0,zeroOrigin,yOnlyAxis);
        // Rotor 1 l/u CFD Domain
        rotor1_low = setupSubtractParts("Rotors Rotor 1 Lower",true);
        rotorOperations.add(rotor1_low);
        rotor1_low.getInputGeometryObjects().setQuery(null);
        rotor1_low.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(0));
        rotor1_low.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(0));
        rotor1_upp = setupSubtractParts("Rotors Rotor 1 Upper",true);
        rotorOperations.add(rotor1_upp);
        rotor1_upp.getInputGeometryObjects().setQuery(null);
        rotor1_upp.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(1));
        rotor1_upp.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(1));
        // Rotor 2 l/u CFD Domain
        rotor2_low = setupSubtractParts("Rotors Rotor 2 Lower",true);
        rotorOperations.add(rotor2_low);
        rotor2_low.getInputGeometryObjects().setQuery(null);
        rotor2_low.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(2));
        rotor2_low.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(2));
        rotor2_upp = setupSubtractParts("Rotors Rotor 2 Upper",true);
        rotorOperations.add(rotor2_upp);
        rotor2_upp.getInputGeometryObjects().setQuery(null);
        rotor2_upp.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(3));
        rotor2_upp.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(3));
        // Rotor 3 l/u CFD Domain
        rotor3_low = setupSubtractParts("Rotors Rotor 3 Lower",true);
        rotorOperations.add(rotor3_low);
        rotor3_low.getInputGeometryObjects().setQuery(null);
        rotor3_low.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(4));
        rotor3_low.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(4));
        rotor3_upp = setupSubtractParts("Rotors Rotor 3 Upper",true);
        rotorOperations.add(rotor3_upp);
        rotor3_upp.getInputGeometryObjects().setQuery(null);
        rotor3_upp.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(5));
        rotor3_upp.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(5));
        // Rotor 4 l/u CFD Domain
        rotor4_low = setupSubtractParts("Rotors Rotor 4 Lower",true);
        rotorOperations.add(rotor4_low);
        rotor4_low.getInputGeometryObjects().setQuery(null);
        rotor4_low.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(6));
        rotor4_low.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(6));
        rotor4_upp = setupSubtractParts("Rotors Rotor 4 Upper",true);
        rotorOperations.add(rotor4_upp);
        rotor4_upp.getInputGeometryObjects().setQuery(null);
        rotor4_upp.getInputGeometryObjects().setObjects(rotorWithPlugPart,rotorIntGeom.get(7));
        rotor4_upp.setTargetPart((SimpleCylinderPart) rotorIntGeom.get(7));

        //Get objects from mesh ops
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor1_low) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor2_low) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor3_low) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor4_low) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor1_upp) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor2_upp) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor3_upp) );
        rotatableParts.add( MeshOpTool.getSubtractOpPart(simu, rotor4_upp) );

      }
      // Geometry Beta
      TransformPartsOperation Kite_Beta0 = setupRotTransform("Kite Rotate Beta0","Lab_Csys",new double[]{0.0, 0.0, -1.0});
      Collection<GeometryObject> stationaryRotParts = new ArrayList();
      stationaryRotParts.addAll(radiatorGeom);
      stationaryRotParts.add(simu.get(SimulationPartManager.class).getPart("Kite CFD Body"));
      Kite_Beta0.getInputGeometryObjects().setQuery(null);
      Kite_Beta0.getInputGeometryObjects().setObjects(stationaryRotParts);
      //
      if(needRunRotors){
          TransformPartsOperation Rotor_Beta0 = setupRotTransform("Rotor Rotate Beta0","Lab_Csys",new double[]{0.0, 0.0, -1.0});
          Rotor_Beta0.getInputGeometryObjects().setQuery(null);
          Rotor_Beta0.getInputGeometryObjects().setObjects(rotatableParts);
/*            Rotor_Beta0.getInputGeometryObjects().setObjects(
        simu.get(SimulationPartManager.class).getPart(rotor1_low.getOutputPartNames().substring(1,rotor1_low.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor1_upp.getOutputPartNames().substring(1,rotor1_upp.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor2_low.getOutputPartNames().substring(1,rotor2_low.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor2_upp.getOutputPartNames().substring(1,rotor2_upp.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor3_low.getOutputPartNames().substring(1,rotor3_low.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor3_upp.getOutputPartNames().substring(1,rotor3_upp.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor4_low.getOutputPartNames().substring(1,rotor4_low.getOutputPartNames().length()-1)),
        simu.get(SimulationPartManager.class).getPart(rotor4_upp.getOutputPartNames().substring(1,rotor4_upp.getOutputPartNames().length()-1))
        );*/
      }

      // Geometry Alpha
      TransformPartsOperation Kite_Alpha0 = setupRotTransform("Kite Rotate Alpha0","Beta_o",yOnlyAxis);
      Kite_Alpha0.getInputGeometryObjects().setQuery(null);
      Kite_Alpha0.getInputGeometryObjects().setObjects(stationaryRotParts);
      if(needRunRotors){
          TransformPartsOperation Rotor_Alpha0 = setupRotTransform("Rotor Rotate Alpha0","Beta_o",yOnlyAxis);
          Rotor_Alpha0.getInputGeometryObjects().setQuery(null);
          Rotor_Alpha0.getInputGeometryObjects().setObjects(rotatableParts);
/*            Rotor_Alpha0.getInputGeometryObjects().setObjects(
            simu.get(SimulationPartManager.class).getPart(rotor1_low.getOutputPartNames().substring(1,rotor1_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor1_upp.getOutputPartNames().substring(1,rotor1_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor2_low.getOutputPartNames().substring(1,rotor2_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor2_upp.getOutputPartNames().substring(1,rotor2_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor3_low.getOutputPartNames().substring(1,rotor3_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor3_upp.getOutputPartNames().substring(1,rotor3_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor4_low.getOutputPartNames().substring(1,rotor4_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor4_upp.getOutputPartNames().substring(1,rotor4_upp.getOutputPartNames().length()-1))
            );*/
      }
      // Create Stationary Kite Domain
      SubtractPartsOperation cfdKite = setupSubtractParts("Kite Domain",false);
      cfdKite.getInputGeometryObjects().setQuery(null);
      if(needRunRotors){

          ArrayList<GeometryPart> tmpArray = new ArrayList();
          for(GeometryPart tmp:rotorIntGeom){
              tmpArray.add(tmp);
          }
          tmpArray.add(simu.get(SimulationPartManager.class).getPart("Kite CFD Body"));
          tmpArray.add(kiteBodyAssmb.getChildParts().getPart("Farfield"));
          cfdKite.getInputGeometryObjects().setObjects(tmpArray);
      }else{
          cfdKite.getInputGeometryObjects().setObjects(simu.get(SimulationPartManager.class).getPart("Kite CFD Body"),
                                                       kiteBodyAssmb.getChildParts().getPart(strFarfieldPart));
      }
      cfdKite.setTargetPart((MeshPart) kiteBodyAssmb.getChildParts().getPart(strFarfieldPart));

      // Volume Mesh Kite Model Setup
      AutoMeshOperation kiteVolumeMesh = setupKiteTrimPrismPBMO("Kite Volume Mesh", labCsys, charBaseSize);
      kiteVolumeMesh.getInputGeometryObjects().setQuery(null);
      kiteVolumeMesh.getInputGeometryObjects().setObjects(simu.get(SimulationPartManager.class).getPart("Kite Domain"));


      // Thin Mesh Radiator Setup 
      //AutoMeshOperation radiatorThinMesh = setupThinRadiators("Radiators Volume",charBaseSize, 10);
      //radiatorThinMesh.getInputGeometryObjects().setQuery(null);
      //radiatorThinMesh.getInputGeometryObjects().setObjects(radiatorGeom);
      setupDMRadiators("Radiators DM", radiatorGeom, "71", "72", 0.0125, 1.0e-4, 0.03, 10, 10);
      // Rotors Setup
      if(needRunRotors){
          AutoMeshOperation rotorVolMesh = setupRotorTrimPrismPBMO("Rotors Volume Mesh",SimTool.getNestedCoordinate(bodyCsys, "Rotor 1 Lower"),1.0);
          rotorVolMesh.setMeshPartByPart(true);
          rotorVolMesh.getInputGeometryObjects().setQuery(null);
          rotorVolMesh.getInputGeometryObjects().setObjects(
            simu.get(SimulationPartManager.class).getPart(rotor1_low.getOutputPartNames().substring(1,rotor1_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor1_upp.getOutputPartNames().substring(1,rotor1_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor2_low.getOutputPartNames().substring(1,rotor2_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor2_upp.getOutputPartNames().substring(1,rotor2_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor3_low.getOutputPartNames().substring(1,rotor3_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor3_upp.getOutputPartNames().substring(1,rotor3_upp.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor4_low.getOutputPartNames().substring(1,rotor4_low.getOutputPartNames().length()-1)),
            simu.get(SimulationPartManager.class).getPart(rotor4_upp.getOutputPartNames().substring(1,rotor4_upp.getOutputPartNames().length()-1))
            );
          //  ROTORS - VOLUME Custom Mesh Controls
          SurfaceCustomMeshControl sC_Rtr_Hub     = MeshOpTool.surfControl(rotorVolMesh,"400 Hubs");
            MeshOpTool.surfFilter( sC_Rtr_Hub,1);
            MeshOpTool.surfPrismThick(    sC_Rtr_Hub,"Absolute",0.02);
            MeshOpTool.surfPrismNearWall( sC_Rtr_Hub,2.5e-5);
            MeshOpTool.surfPrismNumLayers(sC_Rtr_Hub,16);
          SurfaceCustomMeshControl sC_Rtr_Int     = MeshOpTool.surfControl(rotorVolMesh,"890 Interface");
            MeshOpTool.surfFilter(        sC_Rtr_Int,2);
            MeshOpTool.surfPrismOverride( sC_Rtr_Int,true);
            MeshOpTool.surfPrismThick(    sC_Rtr_Int,"Absolute",.03125);
            MeshOpTool.surfPrismNearWall( sC_Rtr_Int,0.015625);
            MeshOpTool.surfPrismNumLayers(sC_Rtr_Int,2);
          // Volume controls
          VolumeCustomMeshControl vC_Rtr_TipRefine = surfVolumeCntrl(rotorVolMesh,"VC Rotor Tips");
            custVolTrimIsoSize(vC_Rtr_TipRefine,"Relative",1.5625);
      }
      //==================================================  
      //  CONTROLS - Create Filtered Custom Mesh Controls
      //==================================================
      SurfaceCustomMeshControl sC_Cnt_Flaps     = MeshOpTool.surfControl(controlsRemesh,"200 Flaps");
        MeshOpTool.surfFilter( sC_Cnt_Flaps,1);
        MeshOpTool.surfNCircle(sC_Cnt_Flaps,72.0);
        MeshOpTool.surfCustTargSize( sC_Cnt_Flaps,"Relative",12.5);
        MeshOpTool.surfCustMinSize(  sC_Cnt_Flaps,"Relative",0.78125);
      SurfaceCustomMeshControl sC_Cnt_Flaps_TE     = MeshOpTool.surfControl(controlsRemesh,"200 Flaps TE");
        MeshOpTool.surfFilter_TE(  sC_Cnt_Flaps_TE,1);
        MeshOpTool.surfCustMinSize(sC_Cnt_Flaps_TE,"Relative",3.125);
        MeshOpTool.surfEdgeProx(   sC_Cnt_Flaps_TE,1.0);
      SurfaceCustomMeshControl sC_Cnt_Empennage = MeshOpTool.surfControl(controlsRemesh,"500 Empennage");
       MeshOpTool.surfFilter(sC_Cnt_Empennage,1);
      SurfaceCustomMeshControl sC_Cnt_Empennage_TE = MeshOpTool.surfControl(controlsRemesh,"500 Empennage TE");
        MeshOpTool.surfFilter_TE(sC_Cnt_Empennage_TE,1);          
        MeshOpTool.surfCustMinSize(sC_Cnt_Empennage_TE,"Relative",3.125);
        MeshOpTool.surfEdgeProx(sC_Cnt_Empennage_TE,1.0);
      SurfaceCustomMeshControl sC_Cnt_Vtail     = MeshOpTool.surfControl(controlsRemesh,"520 Vertical Tail");
       MeshOpTool.surfFilter(      sC_Cnt_Vtail,2);
         MeshOpTool.surfCustTargSize(sC_Cnt_Vtail,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Cnt_Vtail,"Relative",3.125);
        MeshOpTool.surfNCircle(     sC_Cnt_Vtail,54.0);

      SurfaceCustomMeshControl sC_Cnt_Cover     = MeshOpTool.surfControl(controlsRemesh,"521 Control Cover");
       MeshOpTool.surfFilter(      sC_Cnt_Cover,3);
        MeshOpTool.surfNCircle(     sC_Cnt_Cover,24.0);

      SurfaceCustomMeshControl sC_Cnt_Rudder     = MeshOpTool.surfControl(controlsRemesh,"530 Rudder");
       MeshOpTool.surfFilter(      sC_Cnt_Rudder,2);
        MeshOpTool.surfCustTargSize(sC_Cnt_Rudder,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Cnt_Rudder,"Relative",0.78125);
        MeshOpTool.surfNCircle(     sC_Cnt_Rudder,54.0);
      SurfaceCustomMeshControl sC_Cnt_Htail     = MeshOpTool.surfControl(controlsRemesh,"540 Horizontal Tail");
       MeshOpTool.surfFilter(     sC_Cnt_Htail,2);
        MeshOpTool.surfCustTargSize(sC_Cnt_Htail,"Relative",12.5);
        MeshOpTool.surfCustMinSize(sC_Cnt_Htail,"Relative",0.78125);

      //==================================================  
      //  KITE - Create Filtered Custom Mesh Controls
      //==================================================
      kiteSurfMeshControls(kiteRemesh);
      kiteVolMeshControls(kiteVolumeMesh);

      //======================================================
      //  RADIATORS - Create Filtered Custom Mesh Controls
      //======================================================
      /* - Not needed with DM
      SurfaceCustomMeshControl sC_Radiators = MeshOpTool.surfControl(radiatorsRemesh,"700 Radiators");
       MeshOpTool.surfFilter(      sC_Radiators,1);
        MeshOpTool.surfCustTargSize( sC_Radiators,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Radiators,"Relative",3.125);
        MeshOpTool.surfEdgeProx(    sC_Radiators,4.0);
      */
  }
  // kite specific settings
  private void kiteSurfMeshControls(AutoMeshOperation kiteRemesh ){
      //==================================================  
      //  KITE - Create Filtered Custom Mesh Controls
      //==================================================
      SurfaceCustomMeshControl sC_Kite_Wing      = MeshOpTool.surfControl(kiteRemesh,"100 Main Wing");
        MeshOpTool.surfFilter(      sC_Kite_Wing,1);
        MeshOpTool.surfCustTargSize(sC_Kite_Wing,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Kite_Wing,"Relative",3.125);
        MeshOpTool.surfNCircle(     sC_Kite_Wing,54.0);
      SurfaceCustomMeshControl sC_Kite_Wing_TE    = MeshOpTool.surfControl(kiteRemesh,"100 Main Wing TE");
        MeshOpTool.surfFilter_TE(   sC_Kite_Wing_TE,1);
        MeshOpTool.surfCustMinSize( sC_Kite_Wing_TE,"Relative",1.5625);
        MeshOpTool.surfEdgeProx(    sC_Kite_Wing_TE,1.0);

      SurfaceCustomMeshControl sC_Kite_Winglet    = MeshOpTool.surfControl(kiteRemesh,"160 Winglets");
        MeshOpTool.surfFilter(      sC_Kite_Winglet,2);
         MeshOpTool.surfCustTargSize(sC_Kite_Winglet,"Relative",6.25);
        MeshOpTool.surfCustMinSize( sC_Kite_Winglet,"Relative",0.78215);
        MeshOpTool.surfEdgeProx(    sC_Kite_Winglet,2.0);
        MeshOpTool.surfNCircle(     sC_Kite_Winglet,54.0);

      SurfaceCustomMeshControl sC_Kite_Flaps     = MeshOpTool.surfControl(kiteRemesh,"200 Flaps");
        MeshOpTool.surfFilter(sC_Kite_Flaps,1);
        MeshOpTool.surfCustTargSize(sC_Kite_Flaps,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Kite_Flaps,"Relative",3.125);
        MeshOpTool.surfNCircle(sC_Kite_Flaps,54.0);

        //surf2ndWildFilter(SurfaceCustomMeshControl custControl,int numSurf)
      SurfaceCustomMeshControl sC_Kite_Flaps_TE     = MeshOpTool.surfControl(kiteRemesh,"200 Flaps TE");
        MeshOpTool.surfFilter_TE(sC_Kite_Flaps_TE,1);
        MeshOpTool.surfCustMinSize(sC_Kite_Flaps_TE,"Relative",3.125);
        MeshOpTool.surfEdgeProx(sC_Kite_Flaps_TE,1.0);
      SurfaceCustomMeshControl sC_Kite_Actuators = MeshOpTool.surfControl(kiteRemesh,"300 Actuators");
       MeshOpTool.surfFilter(sC_Kite_Actuators,1);
        MeshOpTool.surfCustMinSize(sC_Kite_Actuators,"Relative",3.125);
      SurfaceCustomMeshControl sC_Kite_Pylons    = MeshOpTool.surfControl(kiteRemesh,"400 Pylons");
        MeshOpTool.surfFilter(sC_Kite_Pylons,1);
         MeshOpTool.surfCustTargSize(sC_Kite_Pylons,"Relative",25.0);
      SurfaceCustomMeshControl sC_Kite_Pylons_TE    = MeshOpTool.surfControl(kiteRemesh,"400 Pylons TE");  
        MeshOpTool.surfFilter_TE(sC_Kite_Pylons_TE,1);
        MeshOpTool.surfCustMinSize(sC_Kite_Pylons_TE,"Relative",3.125);
        MeshOpTool.surfEdgeProx(sC_Kite_Pylons_TE,1.0);
      SurfaceCustomMeshControl sC_Kite_Ducts  = MeshOpTool.surfControl(kiteRemesh,"4X1 Ducts");
        surf2ndWildFilter(sC_Kite_Ducts);
        MeshOpTool.surfCustMinSize(sC_Kite_Ducts,"Relative",6.25);
      SurfaceCustomMeshControl sC_Kite_DuctInletRad   = MeshOpTool.surfControl(kiteRemesh,"4X2 Duct Inlet Radius");
        surf2ndWildFilter(sC_Kite_DuctInletRad);
         MeshOpTool.surfCustTargSize(sC_Kite_DuctInletRad,"Relative",6.25);
        MeshOpTool.surfCustMinSize(sC_Kite_DuctInletRad,"Relative",1.5625);
      SurfaceCustomMeshControl sC_Kite_Spinners  = MeshOpTool.surfControl(kiteRemesh,"4X8 Pylon Spinners");
        surf2ndWildFilter(sC_Kite_Spinners);
         MeshOpTool.surfCustTargSize(sC_Kite_Spinners, "Relative",6.25);
        MeshOpTool.surfCustMinSize(sC_Kite_Spinners,"Relative",3.125);
      SurfaceCustomMeshControl sC_Kite_Empennage = MeshOpTool.surfControl(kiteRemesh,"500 Empennage");
        MeshOpTool.surfFilter(sC_Kite_Empennage,1);
      SurfaceCustomMeshControl sC_Kite_Empennage_TE = MeshOpTool.surfControl(kiteRemesh,"500 Empennage TE");
        MeshOpTool.surfFilter_TE(sC_Kite_Empennage_TE,1);          
        MeshOpTool.surfCustMinSize(sC_Kite_Empennage_TE,"Relative",3.125);
        MeshOpTool.surfEdgeProx(sC_Kite_Empennage_TE,1.0);
      SurfaceCustomMeshControl sC_Kite_Vtail     = MeshOpTool.surfControl(kiteRemesh,"520 Vertical Tail");
        MeshOpTool.surfFilter(      sC_Kite_Vtail,2);
         MeshOpTool.surfCustTargSize(sC_Kite_Vtail,"Relative",12.5);
        MeshOpTool.surfCustMinSize( sC_Kite_Vtail,"Relative",6.25);
        MeshOpTool.surfNCircle(     sC_Kite_Vtail,54.0);
      SurfaceCustomMeshControl sC_Kite_Htail     = MeshOpTool.surfControl(kiteRemesh,"540 Horizontal Tail");
        MeshOpTool.surfFilter(sC_Kite_Htail,2);
      SurfaceCustomMeshControl sC_Kite_Fillet  = MeshOpTool.surfControl(kiteRemesh,"601 Fuselage Fillet");
        MeshOpTool.surfFilter(     sC_Kite_Fillet,3);
        MeshOpTool.surfCustMinSize(sC_Kite_Fillet,"Relative",3.125);
      SurfaceCustomMeshControl sC_Kite_Boom      = MeshOpTool.surfControl(kiteRemesh,"640_Boom");
        MeshOpTool.surfFilter( sC_Kite_Boom,2);
        MeshOpTool.surfNCircle(sC_Kite_Boom,54.0);

      //Volume controls
      VolumeCustomMeshControl vC_Kite_FlapIntersects = surfVolumeCntrl(kiteRemesh,"VC Surf Flaps to Body");
        custVolSurfSize(      vC_Kite_FlapIntersects,"Relative",3.125);
        assignVolControlPart( vC_Kite_FlapIntersects,simu.get(SimulationPartManager.class).getPart("Surf Pylon 1 To Wing"));
        assignVolControlPart( vC_Kite_FlapIntersects,simu.get(SimulationPartManager.class).getPart("Surf Pylon 2 To Wing"));
        assignVolControlPart( vC_Kite_FlapIntersects,simu.get(SimulationPartManager.class).getPart("Surf Pylon 3 To Wing"));
        assignVolControlPart( vC_Kite_FlapIntersects,simu.get(SimulationPartManager.class).getPart("Surf Pylon 4 To Wing"));

      VolumeCustomMeshControl vC_Kite_PylonToWing = surfVolumeCntrl(kiteRemesh,"VC Surf Pylons to Wing");
        custVolSurfSize(      vC_Kite_PylonToWing,"Relative",3.125);
        assignVolControlPart( vC_Kite_PylonToWing,simu.get(SimulationPartManager.class).getPart("Surf A1A2 Intersect"));
        assignVolControlPart( vC_Kite_PylonToWing,simu.get(SimulationPartManager.class).getPart("Surf A3A6 Intersect"));
        assignVolControlPart( vC_Kite_PylonToWing,simu.get(SimulationPartManager.class).getPart("Surf A7A8 Intersect"));
  }
  private void kiteVolMeshControls(AutoMeshOperation kiteVolMesh){
      //==================================================  
      //  KITE - Create Filtered Custom Mesh Controls
      //==================================================
      // Reference for Methods on Prisms on Control Surfaces:
      //   MeshOpTool.surfDisablePrisms(SurfaceCustomMeshControl,true/false)
      //   MeshOpTool.surfPrismOverride(SurfaceCustomMeshControl, true/false)
      //   MeshOpTool.surfPrismThick(SurfaceCustomMeshControl,"Relative"/"Absolute",totalThicknessValue)
      //   MeshOpTool.surfPrismNearWall(SurfaceCustomMeshControl,absoluteNearWallThickness)
      //   MeshOpTool.surfPrismNumLayers(SurfaceCustomMeshControl, int numLayers)
      
      // turn On prism layers for the inlet only
      SurfaceCustomMeshControl sC_Kite_Far     = MeshOpTool.surfControl(kiteVolMesh,"000 Farfield Prims");
        MeshOpTool.surfFilter(        sC_Kite_Far,3);
        MeshOpTool.surfPrismOverride( sC_Kite_Far,true);
        MeshOpTool.surfPrismThick(    sC_Kite_Far,"Relative",6400.0);
        MeshOpTool.surfPrismNearWall( sC_Kite_Far, 3.2);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Far,2);
        MeshOpTool.surfCustTargSize(  sC_Kite_Far,"Relative",6400.0);

      // make sure all other far field prisms get the appropriate target size
      SurfaceCustomMeshControl sC_Kite_Far2 = MeshOpTool.surfControl(kiteVolMesh,"000 Farfield Target Size");
        MeshOpTool.surfFilter(        sC_Kite_Far2,1);
        MeshOpTool.surfCustTargSize(  sC_Kite_Far2,"Relative",3200.0);
      //
      SurfaceCustomMeshControl sC_Kite_Wing      = MeshOpTool.surfControl(kiteVolMesh,"100 Main Wing");
        MeshOpTool.surfFilter(        sC_Kite_Wing,1);
        MeshOpTool.surfPrismThick(    sC_Kite_Wing,"Absolute",0.013);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Wing,23);
        MeshOpTool.surfPrismNearWall( sC_Kite_Wing,6.5e-6);
      //
      SurfaceCustomMeshControl sC_Kite_Wing_TE    = MeshOpTool.surfControl(kiteVolMesh,"100 Main Wing TE");
        MeshOpTool.surfFilter_TE(   sC_Kite_Wing_TE,1);
        MeshOpTool.surfPrismThick(  sC_Kite_Wing_TE,"Absolute",0.0075);
      //
      SurfaceCustomMeshControl sC_Kite_Winglet    = MeshOpTool.surfControl(kiteVolMesh,"160 Winglets");
        MeshOpTool.surfFilter(        sC_Kite_Winglet,2);
        MeshOpTool.surfPrismThick(    sC_Kite_Winglet,"Absolute",0.013);
        MeshOpTool.surfPrismNearWall( sC_Kite_Winglet,1e-5);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Winglet,14);
      //
      SurfaceCustomMeshControl sC_Kite_Flaps     = MeshOpTool.surfControl(kiteVolMesh,"200 Flaps");
        MeshOpTool.surfFilter(        sC_Kite_Flaps,1);
        MeshOpTool.surfPrismThick(    sC_Kite_Flaps,"Absolute",0.013);
        MeshOpTool.surfPrismNearWall( sC_Kite_Flaps,6.5e-6);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Flaps,23);
      //
      SurfaceCustomMeshControl sC_Kite_Flaps_TE     = MeshOpTool.surfControl(kiteVolMesh,"200 Flaps TE");
        MeshOpTool.surfFilter_TE(     sC_Kite_Flaps_TE,1);
        MeshOpTool.surfPrismThick(    sC_Kite_Flaps_TE,"Absolute",0.0075);
      //
      SurfaceCustomMeshControl sC_Kite_Actuators = MeshOpTool.surfControl(kiteVolMesh,"300 Actuators");
        MeshOpTool.surfFilter(     sC_Kite_Actuators,1);
        MeshOpTool.surfPrismThick( sC_Kite_Actuators,"Absolute",0.02);
      //
      SurfaceCustomMeshControl sC_Kite_Pylons    = MeshOpTool.surfControl(kiteVolMesh,"400 Pylons");
        MeshOpTool.surfFilter(sC_Kite_Pylons,1);
        // No controls at this time
      SurfaceCustomMeshControl sC_Kite_Pylons_TE    = MeshOpTool.surfControl(kiteVolMesh,"400 Pylons TE");  
        MeshOpTool.surfFilter_TE( sC_Kite_Pylons_TE,1);
        MeshOpTool.surfPrismThick(sC_Kite_Pylons_TE,"Absolute",0.0075);
      //
      SurfaceCustomMeshControl sC_Kite_Ducts  = MeshOpTool.surfControl(kiteVolMesh,"4X1 Ducts");
        surf2ndWildFilter(sC_Kite_Ducts);
        MeshOpTool.surfPrismThick(    sC_Kite_Ducts,"Absolute",0.03);
        //surfPrismNearWall( sC_Kite_Ducts, 1.0e-4);
        //surfPrismNumLayers(sC_Kite_Ducts, 10);
      //
      SurfaceCustomMeshControl sC_Kite_DuctInletRad   = MeshOpTool.surfControl(kiteVolMesh,"4X3 Duct Inlet");
        surf2ndWildFilter(sC_Kite_DuctInletRad);
        MeshOpTool.surfDisablePrisms(sC_Kite_DuctInletRad);
      //
      SurfaceCustomMeshControl sC_Kite_DuctOutletRad   = MeshOpTool.surfControl(kiteVolMesh,"4X4 Duct Outlet");
        surf2ndWildFilter(sC_Kite_DuctOutletRad);
        MeshOpTool.surfDisablePrisms(sC_Kite_DuctOutletRad);
      //
      SurfaceCustomMeshControl sC_Kite_Empennage = MeshOpTool.surfControl(kiteVolMesh,"500 Empennage");
        MeshOpTool.surfFilter(        sC_Kite_Empennage,1);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Empennage, 28);
        MeshOpTool.surfPrismNearWall( sC_Kite_Empennage,5.0e-6);
      //
      SurfaceCustomMeshControl sC_Kite_Empennage_TE = MeshOpTool.surfControl(kiteVolMesh,"500 Empennage TE");
        MeshOpTool.surfFilter_TE( sC_Kite_Empennage_TE,1);          
        MeshOpTool.surfPrismThick(sC_Kite_Empennage_TE,"Absolute",0.0075);
      //
      SurfaceCustomMeshControl sC_Kite_Boom      = MeshOpTool.surfControl(kiteVolMesh,"640 Boom");
        MeshOpTool.surfFilter(        sC_Kite_Boom,2);
        MeshOpTool.surfPrismNumLayers(sC_Kite_Boom, 14);
        MeshOpTool.surfPrismNearWall( sC_Kite_Boom,1.0e-5);
      //
      SurfaceCustomMeshControl sC_Kite_RotorInt  = MeshOpTool.surfControl(kiteVolMesh,"890 Rotor Interfaces");
        MeshOpTool.surfFilter(        sC_Kite_RotorInt,2);
        MeshOpTool.surfPrismOverride( sC_Kite_RotorInt,true);
        MeshOpTool.surfPrismNumLayers(sC_Kite_RotorInt, 2);
        MeshOpTool.surfPrismNearWall( sC_Kite_RotorInt,0.1);
        MeshOpTool.surfPrismThick(    sC_Kite_RotorInt,"Absolute",0.2);




      //Volume controls
      VolumeCustomMeshControl vC_Kite_KiteWake = surfVolumeCntrl(kiteVolMesh,"VC Kite Wake");
        custVolTrimIsoSize(vC_Kite_KiteWake,"Relative",100.0);
        assignVolControlPart( vC_Kite_KiteWake,simu.get(SimulationPartManager.class).getPart("VC Kite Wake"));
      //
      VolumeCustomMeshControl vC_Kite_FarWake = surfVolumeCntrl(kiteVolMesh,"VC Far Wake");
        custVolTrimIsoSize(vC_Kite_FarWake,"Relative",400.0);
        assignVolControlPart( vC_Kite_FarWake,simu.get(SimulationPartManager.class).getPart("VC Far Wake"));
      //
      VolumeCustomMeshControl vC_Kite_FlapGap = surfVolumeCntrl(kiteVolMesh,"VC Flap Gaps");
        custVolTrimIsoSize(vC_Kite_FlapGap,"Relative",3.125);
        assignVolControlPart( vC_Kite_FlapGap,simu.get(SimulationPartManager.class).getPart("VC A1A2 Flap Gap"));
        assignVolControlPart( vC_Kite_FlapGap,simu.get(SimulationPartManager.class).getPart("VC A3A6 Flap Gap"));
        assignVolControlPart( vC_Kite_FlapGap,simu.get(SimulationPartManager.class).getPart("VC A7A8 Flap Gap"));

      // main wing
      VolumeCustomMeshControl vC_Kite_MainWing = surfVolumeCntrl(kiteVolMesh,"VC Main Wing");
        custVolTrimIsoSize(vC_Kite_MainWing,"Relative",12.5);
        assignVolControlPart( vC_Kite_MainWing,simu.get(SimulationPartManager.class).getPart("VC Main Wing"));

      // aircraft near wake
      VolumeCustomMeshControl vC_Kite_MainWingWake = surfVolumeCntrl(kiteVolMesh,"VC Near Wake");
        custVolTrimIsoSize(vC_Kite_MainWingWake,"Relative",50.0);    
        assignVolControlPart( vC_Kite_MainWingWake,simu.get(SimulationPartManager.class).getPart("VC Near Wake"));


      //
      VolumeCustomMeshControl vC_Kite_PylonWakeRefinement = surfVolumeCntrl(kiteVolMesh,"VC Attachments to Wing");
        custVolTrimIsoSize(   vC_Kite_PylonWakeRefinement,"Relative",25.0);     
        assignVolControlPart( vC_Kite_PylonWakeRefinement,simu.get(SimulationPartManager.class).getPart("VC Pylon 1 to Wing"));
        assignVolControlPart( vC_Kite_PylonWakeRefinement,simu.get(SimulationPartManager.class).getPart("VC Pylon 2 to Wing"));
        assignVolControlPart( vC_Kite_PylonWakeRefinement,simu.get(SimulationPartManager.class).getPart("VC Pylon 3 to Wing"));
        assignVolControlPart( vC_Kite_PylonWakeRefinement,simu.get(SimulationPartManager.class).getPart("VC Pylon 4 to Wing"));
        assignVolControlPart( vC_Kite_PylonWakeRefinement,simu.get(SimulationPartManager.class).getPart("VC Center Fuselage to Main Wing"));

        
        
      // Compute total volume mesh prediction:
      ArrayList<AutoMeshOperation> allTheOps = new ArrayList(
        Arrays.asList(vC_Kite_KiteWake, vC_Kite_FarWake, vC_Kite_FlapGap, vC_Kite_MainWing, vC_Kite_MainWingWake,
          vC_Kite_PylonWakeRefinement));
        
  }
  // Solution validity check
  private boolean areVolumeMeshOperationsUpToDate(boolean needRunRotors){
    boolean allVolumeOpsUpToDate=true;
    boolean allRadiatorOpsUpToDate=true;
    boolean allrotorOpsUpToDate=true;

    AutoMeshOperation tmpOp;
    try{
      tmpOp = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Kite Volume Mesh"));
      if( MeshOpTool.isAutoMeshOpUpToDate(tmpOp) ){
        simu.println("    Kite Volume Appears Up To Date");
      }else{
        simu.println("    WARNING: Kite Volume Mesh DOES NOT APPEAR UP TO DATE");
        allVolumeOpsUpToDate=false;
      }
    }catch(NeoException e){
      simu.println("   There is no Volume Mesh Operation called Kite Volume Mesh!");
    }

//        if(checkVolumeMeshUpToDate("Radiators DM")){
//            simu.println("    Radiators DM Appears Up To Date");
//        }else{
//            simu.println("    WARNING: Radiators DM DOES NOT APPEAR UP TO DATE");
//            allRadiatorOpsUpToDate=false;
//            allVolumeOpsUpToDate=false;
//        }

    if(needRunRotors){
      try{
        tmpOp = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Rotors Volume Mesh"));
        MeshOpTool.isAutoMeshOpUpToDate( tmpOp ) ;
      }catch(NeoException e){
      simu.println("   There is no Volume Mesh Operation called Rotors Volume Mesh!");
      }
    }else{
      if(needRunRotors){
          simu.println("    WARNING: Rotors Volume Mesh DOES NOT APPEAR UP TO DATE");
      }else{
          simu.println("    There are no Rotors in this case.");
      }
      allrotorOpsUpToDate=false;
    }
    if(allVolumeOpsUpToDate){
      return true;
    }else{
      return false;
    }
  }
  // bem methods
  private VolumeCustomMeshControl injectBemVC(GeometryPart bemPart, double bladeThickness){
      AutoMeshOperation autoMO = 
          ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Kite Volume Mesh"));

      VolumeCustomMeshControl volCMC;
      try{
          volCMC = (VolumeCustomMeshControl) autoMO.getCustomMeshControls().getObject("VC BEM");

      }catch (NeoException e){
          volCMC = autoMO.getCustomMeshControls().createVolumeControl();
      }
      simu.println("Adding Part");
      volCMC.setPresentationName("VC BEM");
      volCMC.getGeometryObjects().addPart(bemPart);

      //Isotropic refinement
      VolumeControlTrimmerSizeOption vcTrimOption = 
        volCMC.getCustomConditions().get(VolumeControlTrimmerSizeOption.class);
      vcTrimOption.setVolumeControlBaseSizeOption(true);
      //Calculate proper base size

      VolumeControlSize vcSize = 
        volCMC.getCustomValues().get(VolumeControlSize.class);
      GenericRelativeSize relativeSize = 
        ((GenericRelativeSize) vcSize.getRelativeSize());

      double refinePct = 1.0;
      while (charBaseSize*refinePct>bladeThickness){
          refinePct=refinePct/2.0;
      }
      relativeSize.setPercentage(refinePct*100.0);
      return volCMC;
  }
  private void injectLowYPLusMeshPrismSettings(){








  }

  //=============================================
  // Mesh Operations Methods
  //=============================================
  private void removeRotorOperationsFromTree(){
      AutoMeshOperation tmpMeshOp = 
      ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Rotors Volume Mesh"));
      simu.get(MeshOperationManager.class).removeObjects(tmpMeshOp);
      //
      TransformPartsOperation transformPartsOperation_2 = 
        ((TransformPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotor Rotate Alpha0"));
      simu.get(MeshOperationManager.class).removeObjects(transformPartsOperation_2);
      //
      TransformPartsOperation transformPartsOperation_1 = 
        ((TransformPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotor Rotate Beta0"));
      simu.get(MeshOperationManager.class).removeObjects(transformPartsOperation_1);
      //
      SubtractPartsOperation subtractPartsOperation_0 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 4 Upper"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_0);
      //
      SubtractPartsOperation subtractPartsOperation_1 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 4 Lower"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_1);
      //
      SubtractPartsOperation subtractPartsOperation_2 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 3 Upper"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_2);
      //
      SubtractPartsOperation subtractPartsOperation_3 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 3 Lower"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_3);
      //
      SubtractPartsOperation subtractPartsOperation_4 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 2 Upper"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_4);
      //
      SubtractPartsOperation subtractPartsOperation_5 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 2 Lower"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_5);
      //
      SubtractPartsOperation subtractPartsOperation_6 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 1 Upper"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_6);
      //
      SubtractPartsOperation subtractPartsOperation_7 = 
        ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotor 1 Lower"));
      simu.get(MeshOperationManager.class).removeObjects(subtractPartsOperation_7);
      //
      UnitePartsOperation unitePartsOperation_0 = 
        ((UnitePartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Join Rotors to Plug"));
      simu.get(MeshOperationManager.class).removeObjects(unitePartsOperation_0);
      //
      TransformPartsOperation transformPartsOperation_3 = 
        ((TransformPartsOperation) simu.get(MeshOperationManager.class).getObject("Rotors Rotate Plug to Rotors"));
      simu.get(MeshOperationManager.class).removeObjects(transformPartsOperation_3);
      //
      AutoMeshOperation autoMeshOperation_1 = 
        ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Rotors Remesh Plug"));
      simu.get(MeshOperationManager.class).removeObjects(autoMeshOperation_1);
      //
      AutoMeshOperation autoMeshOperation_2 = 
        ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Rotors Props Remesh"));
      simu.get(MeshOperationManager.class).removeObjects(autoMeshOperation_2);
      //
      AutoMeshOperation autoMeshOperation_3 = 
        ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Rotors Remesh Rotor Interfaces"));
      simu.get(MeshOperationManager.class).removeObjects(autoMeshOperation_3);
  }
  private void enableRotorRegionLink(){

  }
  private void disableRotorRegionLink(){

  }
  private void checkPartsEmpty(){
      if(  simu.getPartManager().getObjects().isEmpty()){
          GeometryTool.makeBlock(simu,"Dummy",labCsys,new double[]{0.,0.,0.},new double[]{1.,1.,1.});
      }
  }

  private UnitePartsOperation setupUniteParts(String opName,boolean remeshOnIntersection){
      UnitePartsOperation unitePBMO;
      try{
          unitePBMO = ((UnitePartsOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          unitePBMO  = (UnitePartsOperation) simu.get(MeshOperationManager.class).createUnitePartsOperation(new NeoObjectVector(new Object[] {}));
          unitePBMO.setPresentationName(opName);
          String bodyName =  unitePBMO.getOutputPartNames();
          MeshOperationPart mOP = 
        ((MeshOperationPart) simu.get(SimulationPartManager.class).getPart(bodyName.substring(1,bodyName.length()-1)));
          mOP.setPresentationName(opName);
      }
      if(remeshOnIntersection){
        RemeshIntersectionCurveOption remeshOption = 
          unitePBMO.getBooleanOperationValuesManager().get(RemeshIntersectionCurveOption.class);
        remeshOption.setRemeshIntersectionCurve(true);
      }
      return unitePBMO;
  }
  private SubtractPartsOperation setupSubtractParts(String opName,boolean remeshOnIntersection){
      SubtractPartsOperation subPBMO;
      try{
          subPBMO = ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          subPBMO  = (SubtractPartsOperation) simu.get(MeshOperationManager.class).createSubtractPartsOperation(new NeoObjectVector(new Object[] {}));
          subPBMO.setPresentationName(opName);
          String bodyName =  subPBMO.getOutputPartNames();
          MeshOperationPart mOP = 
        ((MeshOperationPart) simu.get(SimulationPartManager.class).getPart(bodyName.substring(1,bodyName.length()-1)));
          mOP.setPresentationName(opName);
      }
      if(remeshOnIntersection){
        RemeshIntersectionCurveOption remeshOption = 
          subPBMO.getBooleanOperationValuesManager().get(RemeshIntersectionCurveOption.class);
        remeshOption.setRemeshIntersectionCurve(true);
      }
      return subPBMO;
  }
  private TransformPartsOperation setupRotTransform(String opName,String cSys, double[] rotAxis){
      TransformPartsOperation transPartsOp;
      try{
          transPartsOp = 
            (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(opName);
      }catch(NeoException e){
          transPartsOp = 
            (TransformPartsOperation) simu.get(MeshOperationManager.class).createTransformPartsOperation(new NeoObjectVector(new Object[] {}));
          transPartsOp.setPresentationName(opName);
      }
      // empty whatever is in here
      for(TransformControl tmp:transPartsOp.getTransforms().getObjects()){
          transPartsOp.getTransforms().remove(tmp);
      }
      RotationControl rotControl = 
            transPartsOp.getTransforms().createRotationControl();
      switch (cSys) {
          case "Body Csys":
              rotControl.setCoordinateSystem(bodyCsys);
              break;
          case "Lab_Csys":
              rotControl.setCoordinateSystem(labCsys);
              break;
          case "Beta_o":
              rotControl.setCoordinateSystem(labCsys.getCoordinateSystemManager().getCoordinateSystem("Beta_o"));
              break;
          default:
              rotControl.setCoordinateSystem(SimTool.getLabBasedCoordinate(simu, cSys));
              break;
      }
      rotControl.getAxisVector().setCoordinate(prefUVec, prefUVec, prefUVec,new DoubleVector(rotAxis));
      rotControl.getAngle().setUnits((Units) simu.getUnitsManager().getObject("deg"));
      rotControl.getAngle().setValue(0.0);
      return transPartsOp;    
  }
  private void setTransformRotationAngle(String meshOpName, CoordinateSystem tmpCsys,double rotAngle, double[] rotOrigin, double[] rotAxis ){
      TransformPartsOperation tmpOp;
      try{
          tmpOp = (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(meshOpName);
          ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).getAngle().setValue(rotAngle);
          ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).getAxisVectorInput().setVector(new DoubleVector(rotAxis));
          ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).getOriginInput().setVector(new DoubleVector(rotOrigin));
          ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).setCoordinateSystem(tmpCsys);
      }catch(NeoException e){
          simu.println("WARNING: There is no rotation Mesh Operation entitled "+meshOpName);
      }
  }
  private double getRotationAngle(String meshOpName){
      TransformPartsOperation tmpOp;
      double retVal;
      try{
          tmpOp = (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(meshOpName);
          retVal = ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).getAngle().getRawValue();
      }catch(NeoException e){
          simu.println("WARNING: There is no rotation Mesh Operation entitled "+meshOpName);
          retVal = -999.;
      }
      return retVal;
  }

  // Set up mesh settings for the kite
  private AutoMeshOperation setupSurfPBMO(String opName, double baseSize, double targetSizePct, double minSizePct,double nCircle, double sGRValue){
      /* Method setupSurfPBMO
          A method to create or get a particular surface remesh PBM Operation
      */
      //Standard M600 requirements
      String selectRemesher="star.resurfacer.ResurfacerAutoMesher";
      String selectAutoRepair="star.resurfacer.AutomaticSurfaceRepairAutoMesher";
      double minTriQual = 0.2;
      boolean proxRef = false;
      double numProx = 2.0; // Not currently used

      AutoMeshOperation pBMO;

      try{ 
          pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          pBMO =
              simu.get(MeshOperationManager.class).createAutoMeshOperation(new StringVector(new String[] {selectRemesher,selectAutoRepair}), new NeoObjectVector(new Object[] {}));
          pBMO.setPresentationName(opName);
          ResurfacerAutoMesher remeshModel = ((ResurfacerAutoMesher) pBMO.getMeshers().getObject("Surface Remesher"));
          //Auto Repair model setup
          AutomaticSurfaceRepairAutoMesher automaticSurfaceRepairAutoMesher_0 = 
            ((AutomaticSurfaceRepairAutoMesher) pBMO.getMeshers().getObject("Automatic Surface Repair"));
          automaticSurfaceRepairAutoMesher_0.setMinimumFaceQuality(minTriQual);

          // Remesher model setup
          if(!proxRef){
              remeshModel.setDoProximityRefinement(proxRef);
          }else{
              remeshModel.setDoProximityRefinement(!proxRef);
          }
          remeshModel.setMinimumFaceQuality(minTriQual);
          pBMO.getDefaultValues().get(BaseSize.class).setValue(baseSize);

          // Target Size
          PartsTargetSurfaceSize targetSS = 
            pBMO.getDefaultValues().get(PartsTargetSurfaceSize.class);
          GenericRelativeSize genericRelativeSize_0 = 
            ((GenericRelativeSize) targetSS.getRelativeSize());
          genericRelativeSize_0.setPercentage(targetSizePct);

          // Minimum Size
          PartsMinimumSurfaceSize partsMinimumSurfaceSize_0 = 
            pBMO.getDefaultValues().get(PartsMinimumSurfaceSize.class);
          GenericRelativeSize genericRelativeSize_1 = 
            ((GenericRelativeSize) partsMinimumSurfaceSize_0.getRelativeSize());
          genericRelativeSize_1.setPercentage(minSizePct);

          //Surface curvature
          pBMO.getDefaultValues().get(SurfaceCurvature.class).setNumPointsAroundCircle(nCircle);

          //Surface growth rate
          SurfaceGrowthRate sGR = pBMO.getDefaultValues().get(SurfaceGrowthRate.class);
          sGR.setGrowthRate(sGRValue);
      }

  return pBMO;
  }
  private AutoMeshOperation setupKiteTrimPrismPBMO(String opName,CoordinateSystem cartCsys, double baseSize){
      AutoMeshOperation pBMO;
      //Prism Default Settings
      double gapFillPct = 40.0;
      double minPrismThickPct = 0.1;
      double layerChopPct = 20.0;
      double nearCoreAR = 0.75;
      int nPrisms = 16;
      double firstCellThickness=2.5E-5;
      double prismAbsThickness=0.05;
      // Trimmer Default Settings
      double trimToPrismR = 2.0;
      double biggestCellSizePct = 6400.0;
      int trimGR = 8;

      try{ 
          pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          pBMO = simu.get(MeshOperationManager.class)
                  .createAutoMeshOperation(new StringVector(new String[] {"star.trimmer.TrimmerAutoMesher",
                    "star.prismmesher.PrismAutoMesher"}), new NeoObjectVector(new Object[] {}));
          pBMO.setPresentationName(opName);
      }
      // Trimmer Model Settings
      pBMO.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.PARALLEL);
      TrimmerAutoMesher trimmerModel = 
        ((TrimmerAutoMesher) pBMO.getMeshers().getObject("Trimmed Cell Mesher"));
      trimmerModel.setCoordinateSystem(cartCsys);
      trimmerModel.setDoMeshAlignment(true);
      // Prism Model Settings
      PrismAutoMesher prismMesher = 
          ((PrismAutoMesher) pBMO.getMeshers().getObject("Prism Layer Mesher"));
      prismMesher.getPrismStretchingFunction().setSelected(PrismStretchingFunction.Type.HYPERBOLIC_TANGENT);
      prismMesher.getPrismStretchingOption().setSelected(PrismStretchingOption.Type.WALL_THICKNESS);
      prismMesher.setGapFillPercentage(gapFillPct);
      prismMesher.setMinimumThickness(minPrismThickPct);
      prismMesher.setLayerChoppingPercentage(layerChopPct);
      prismMesher.setNearCoreLayerAspectRatio(nearCoreAR);
      // Default Trimmer Value Settings
      pBMO.getDefaultValues().get(BaseSize.class).setValue(baseSize);
      //
      MaxTrimmerSizeToPrismThicknessRatio trimmerToPrismRatio = 
          pBMO.getDefaultValues().get(MaxTrimmerSizeToPrismThicknessRatio.class);
      trimmerToPrismRatio.setLimitCellSizeByPrismThickness(true);
      SizeThicknessRatio trimToPrismRatio = 
      trimmerToPrismRatio.getSizeThicknessRatio();
      trimToPrismRatio.setNeighboringThicknessMultiplier(trimToPrismR);
      //
      MaximumCellSize maxCellSize = 
        pBMO.getDefaultValues().get(MaximumCellSize.class);
      maxCellSize.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
      GenericRelativeSize gRS = 
          ((GenericRelativeSize) maxCellSize.getRelativeSize());
      gRS.setPercentage(biggestCellSizePct);
      //
      PartsSimpleTemplateGrowthRate trimGrowthRate = 
        pBMO.getDefaultValues().get(PartsSimpleTemplateGrowthRate.class);
      trimGrowthRate.getGrowthRateOption().setSelected(PartsGrowthRateOption.Type.CUSTOM);
      PartsCustomLayersDefaultGrowthRate partsCustomLayersDefaultGrowthRate_0 = 
      trimGrowthRate.getCustomLayersDefaultGrowthRate();
      partsCustomLayersDefaultGrowthRate_0.setCustomLayers(trimGR);
      // Prisms Default Settings
      NumPrismLayers numPrismLayers = 
          pBMO.getDefaultValues().get(NumPrismLayers.class);
      numPrismLayers.setNumLayers(nPrisms);
      //
      pBMO.getDefaultValues().get(PrismWallThickness.class).setValue(firstCellThickness);
      //
      PrismThickness prismThickness_0 = 
      pBMO.getDefaultValues().get(PrismThickness.class);
      prismThickness_0.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
      GenericAbsoluteSize gAS = 
        ((GenericAbsoluteSize) prismThickness_0.getAbsoluteSize());
      gAS.getValue().setValue(prismAbsThickness);
  return pBMO;
  }
  private AutoMeshOperation setupThinRadiators(String opName,double baseSize, int numLayers){
      AutoMeshOperation pBMO;
      //Prism Default Settings
      int nPrisms = 15;
      double nearWallThick = 1.0e-4;
      double prismTotalThick=0.03;
      //Thin Mesher Default Settings
      double minFaceQual=0.2;
      double minSurfSizePct = 12.5;
      double nPtsGap = 4.0;
      double surfGR = 1.2;

      try{
          pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          pBMO =
            simu.get(MeshOperationManager.class).createAutoMeshOperation(
              new StringVector(
//                  new String[] {"star.resurfacer.ResurfacerAutoMesher", "star.resurfacer.AutomaticSurfaceRepairAutoMesher",
//                                "star.dualmesher.DualAutoMesher", "star.solidmesher.ThinAutoMesher", "star.prismmesher.PrismAutoMesher"}),
                new String[] {"star.dualmesher.DualAutoMesher", "star.solidmesher.ThinAutoMesher", "star.prismmesher.PrismAutoMesher"}),
              new NeoObjectVector(new Object[] {}));
          pBMO.setPresentationName(opName);
          pBMO.setMeshPartByPart(true);
          pBMO.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.CONCURRENT);
          // Remesher Model
//            ResurfacerAutoMesher surfRemesher = ((ResurfacerAutoMesher) pBMO.getMeshers().getObject("Surface Remesher"));
//            surfRemesher.setMinimumFaceQuality(minFaceQual);
          // AutoRepair Model
//            AutomaticSurfaceRepairAutoMesher autoRepair = 
//             ((AutomaticSurfaceRepairAutoMesher) pBMO.getMeshers().getObject("Automatic Surface Repair"));
//            autoRepair.setMinimumFaceQuality(minFaceQual);
          //Prism Layer Model
          PrismAutoMesher prismModel = 
            ((PrismAutoMesher) pBMO.getMeshers().getObject("Prism Layer Mesher"));
          prismModel.getPrismStretchingFunction().setSelected(PrismStretchingFunction.Type.HYPERBOLIC_TANGENT);
          prismModel.getPrismStretchingOption().setSelected(PrismStretchingOption.Type.WALL_THICKNESS);
          // Default Value Settings
          pBMO.getDefaultValues().get(BaseSize.class).setValue(baseSize);
//            PartsMinimumSurfaceSize partsMinSurfSize = 
//              pBMO.getDefaultValues().get(PartsMinimumSurfaceSize.class);
//            GenericRelativeSize gRS = 
//              ((GenericRelativeSize) partsMinSurfSize.getRelativeSize());
//            gRS.setPercentage(minSurfSizePct);
          //
//            SurfaceProximity surfaceProximity_1 = 
//              pBMO.getDefaultValues().get(SurfaceProximity.class);
//            surfaceProximity_1.setNumPointsInGap(nPtsGap);
          //
//            SurfaceGrowthRate surfaceGrowthRate = 
//              pBMO.getDefaultValues().get(SurfaceGrowthRate.class);
//            surfaceGrowthRate.setGrowthRate(surfGR);
          //
          ThinNumLayers thinNumLayers = pBMO.getDefaultValues().get(ThinNumLayers.class);
          thinNumLayers.setLayers(numLayers);
          // prisms here for side wall interfacing
          NumPrismLayers numPrismLayers_6 = 
          pBMO.getDefaultValues().get(NumPrismLayers.class);
          numPrismLayers_6.setNumLayers(nPrisms);
          pBMO.getDefaultValues().get(PrismWallThickness.class).setValue(nearWallThick);

          PrismThickness prismThickness = 
            pBMO.getDefaultValues().get(PrismThickness.class);
          prismThickness.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);

          GenericAbsoluteSize gAS = 
            ((GenericAbsoluteSize) prismThickness.getAbsoluteSize());
          gAS.getValue().setValue(prismTotalThick);
      }
      return pBMO;
  }
  private DirectedMeshOperation setupDMRadiators(String opName, ArrayList<GeometryPart> dMParts, String srcSurfPre, String trgSurfPre,
          double baseSize, double nearWallThick, double absTotThick, int nPrisms, int numLayers){
      //Prism Default Settings
      double minPrismThickPct = 0.1;
      double layerChopPct = 20.0;
      //Thin Mesher Default Settings
      double minFaceQual=0.2;
      double minSurfSizePct = 3.125;
      double surfGR = 1.1;

      ArrayList<PartSurface> sourceSurf = new ArrayList();
      ArrayList<PartSurface> targetSurf = new ArrayList();
      for(GeometryObject tmp:dMParts){
          for(PartSurface tmp2:tmp.getPartSurfaces()){
              if(tmp2.getPresentationName().startsWith(srcSurfPre)){
                  sourceSurf.add(tmp2);
              }
              if(tmp2.getPresentationName().startsWith(trgSurfPre)){
                  targetSurf.add(tmp2);
              }
          }
      }
      DirectedMeshOperation dMeshOp;
      try{
          dMeshOp = (DirectedMeshOperation) simu.get(MeshOperationManager.class).getObject("Radiators DM");
          dMeshOp.getInputGeometryObjects().setQuery(null);
          dMeshOp.updateInputParts(dMParts);

      }catch(NeoException e){
          dMeshOp = 
              (DirectedMeshOperation) simu.get(MeshOperationManager.class).createDirectedMeshOperation(dMParts);
          dMeshOp.setPresentationName("Radiators DM");


      }
      dMeshOp.getSourceSurfaceGroup().setObjects(sourceSurf);
      dMeshOp.getTargetSurfaceGroup().setObjects(targetSurf);

      ArrayList<Object> realColl = new ArrayList();
      for(GeometryObject tmp:dMParts){
          DirectedMeshPartCollection dMeshPartColl = 
        ((DirectedMeshPartCollection) dMeshOp.getGuidedMeshPartCollectionManager().getObject(tmp.getPresentationName()));
          // add to volume distribution list
          realColl.add(dMeshPartColl);
          //
          DirectedAutoSourceMesh directAutoSourceMesh;
          try{
              directAutoSourceMesh = 
          ((DirectedAutoSourceMesh) dMeshOp.getGuidedSurfaceMeshBaseManager().getObject(tmp.getPresentationName()));
          }catch(NeoException e){
              dMeshOp.getGuidedSurfaceMeshBaseManager().createAutoSourceMesh(
                  new StringVector(new String[] {"star.twodmesher.DualAutoMesher2d", "star.prismmesher.PrismAutoMesher"}), new NeoObjectVector(new Object[] {dMeshPartColl}));
              directAutoSourceMesh = 
          ((DirectedAutoSourceMesh) dMeshOp.getGuidedSurfaceMeshBaseManager().getObject("Auto Mesh"));
              directAutoSourceMesh.setPresentationName(tmp.getPresentationName());
          }



          DualAutoMesher2d dualAutoMesher2d_0 = 
              ((DualAutoMesher2d) directAutoSourceMesh.getMeshers().getObject("Polygonal Mesher"));
          dualAutoMesher2d_0.setMinimumFaceQuality(minFaceQual);
          // prism settings
          PrismAutoMesher prismMesher = 
              ((PrismAutoMesher) directAutoSourceMesh.getMeshers().getObject("Prism Layer Mesher"));



          prismMesher.getPrismStretchingOption().setSelected(PrismStretchingOption.Type.THICKNESS_RATIO);
          prismMesher.getPrismStretchingFunction().setSelected(PrismStretchingFunction.Type.HYPERBOLIC_TANGENT);
          prismMesher.getPrismStretchingOption().setSelected(PrismStretchingOption.Type.WALL_THICKNESS);


          prismMesher.setMinimumThickness(minPrismThickPct);
          prismMesher.setLayerChoppingPercentage(layerChopPct);
          directAutoSourceMesh.getDefaultValues().get(PrismWallThickness.class).setValue(nearWallThick);
          directAutoSourceMesh.getDefaultValues().get(BaseSize.class).setValue(baseSize);
          PartsMinimumSurfaceSize partsMinimumSurfaceSize_0 = 
              directAutoSourceMesh.getDefaultValues().get(PartsMinimumSurfaceSize.class);
          GenericRelativeSize genericRelativeSize_0 = 
              ((GenericRelativeSize) partsMinimumSurfaceSize_0.getRelativeSize());
          genericRelativeSize_0.setPercentage(minSurfSizePct);
          dualAutoMesher2d_0.setDoProximityRefinement(false);
          NumPrismLayers numPrismLayers_0 = 
              directAutoSourceMesh.getDefaultValues().get(NumPrismLayers.class);
          numPrismLayers_0.setNumLayers(nPrisms);
          SurfaceGrowthRate surfaceGrowthRate_0 = 
              directAutoSourceMesh.getDefaultValues().get(SurfaceGrowthRate.class);
          surfaceGrowthRate_0.setGrowthRate(surfGR);
          PrismThickness prismThickness_0 = 
              directAutoSourceMesh.getDefaultValues().get(PrismThickness.class);
          prismThickness_0.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
          GenericAbsoluteSize genericAbsoluteSize_0 = 
              ((GenericAbsoluteSize) prismThickness_0.getAbsoluteSize());
          genericAbsoluteSize_0.getValue().setValue(absTotThick);

      }

      DirectedMeshDistribution thinStack;
      try{
          thinStack = (DirectedMeshDistribution) dMeshOp.getDirectedMeshDistributionManager().getObject("Volume Distribution");
      }catch(NeoException e){
          thinStack = dMeshOp.getDirectedMeshDistributionManager().createDirectedMeshDistribution(new NeoObjectVector(realColl.toArray()), "Constant");
          thinStack.setPresentationName("Volume Distribution");
      }

      DirectedMeshDistribution directedMeshDistribution_0 = 
        ((DirectedMeshDistribution) dMeshOp.getDirectedMeshDistributionManager().getObject("Volume Distribution"));
      DirectedMeshNumLayers directedMeshNumLayers_0 = 
        directedMeshDistribution_0.getDefaultValues().get(DirectedMeshNumLayers.class);
      directedMeshNumLayers_0.setNumLayers(numLayers);

      return dMeshOp;
  }
  private AutoMeshOperation setupRotorTrimPrismPBMO(String opName, CartesianCoordinateSystem cartCsys, double baseSize){
      //Prism Default Settings
      double gapFillPct = 40.0;
      double minPrismThickPct = 0.1;
      double layerChopPct = 20.0;
      double nearCoreAR = 0.75;
      int nPrisms = 20;
      double firstCellThickness=5e-6;
      double prismAbsThickness=0.01;
      // Trimmer Default Settings
      double trimToPrismR = 2.0;
      double biggestCellSizePct = 6400.0;
      int trimGR = 10;
      AutoMeshOperation pBMO;
      try{ 
          pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
          // Trimmer Model Settings
          pBMO = simu.get(MeshOperationManager.class).createAutoMeshOperation(new StringVector(new String[] {"star.trimmer.TrimmerAutoMesher", "star.prismmesher.PrismAutoMesher"}), new NeoObjectVector(new Object[] {}));
          pBMO.setPresentationName(opName);
      }
      // Trimmer Model Settings
      pBMO.getMesherParallelModeOption().setSelected(MesherParallelModeOption.Type.PARALLEL);
      TrimmerAutoMesher trimmerModel = 
        ((TrimmerAutoMesher) pBMO.getMeshers().getObject("Trimmed Cell Mesher"));
      trimmerModel.setCoordinateSystem(labCsys);
      trimmerModel.setDoMeshAlignment(true);
      // Prism Model Settings
      PrismAutoMesher prismMesher = 
          ((PrismAutoMesher) pBMO.getMeshers().getObject("Prism Layer Mesher"));
      prismMesher.getPrismStretchingFunction().setSelected(PrismStretchingFunction.Type.HYPERBOLIC_TANGENT);
      prismMesher.getPrismStretchingOption().setSelected(PrismStretchingOption.Type.WALL_THICKNESS);
      prismMesher.setGapFillPercentage(gapFillPct);
      prismMesher.setMinimumThickness(minPrismThickPct);
      prismMesher.setLayerChoppingPercentage(layerChopPct);
      prismMesher.setNearCoreLayerAspectRatio(nearCoreAR);
      // Default Trimmer Value Settings
      pBMO.getDefaultValues().get(BaseSize.class).setValue(baseSize);
      //
      MaxTrimmerSizeToPrismThicknessRatio trimmerToPrismRatio = 
          pBMO.getDefaultValues().get(MaxTrimmerSizeToPrismThicknessRatio.class);
      trimmerToPrismRatio.setLimitCellSizeByPrismThickness(true);
      SizeThicknessRatio trimToPrismRatio = 
      trimmerToPrismRatio.getSizeThicknessRatio();
      trimToPrismRatio.setNeighboringThicknessMultiplier(trimToPrismR);
      //
      MaximumCellSize maxCellSize = 
        pBMO.getDefaultValues().get(MaximumCellSize.class);
      maxCellSize.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
      GenericRelativeSize gRS = 
          ((GenericRelativeSize) maxCellSize.getRelativeSize());
      gRS.setPercentage(biggestCellSizePct);
      //
      PartsSimpleTemplateGrowthRate trimGrowthRate = 
        pBMO.getDefaultValues().get(PartsSimpleTemplateGrowthRate.class);
      trimGrowthRate.getGrowthRateOption().setSelected(PartsGrowthRateOption.Type.CUSTOM);
      PartsCustomLayersDefaultGrowthRate partsCustomLayersDefaultGrowthRate_0 = 
      trimGrowthRate.getCustomLayersDefaultGrowthRate();
      partsCustomLayersDefaultGrowthRate_0.setCustomLayers(trimGR);
      
      // Prisms Default Settings
      NumPrismLayers numPrismLayers = 
          pBMO.getDefaultValues().get(NumPrismLayers.class);
      numPrismLayers.setNumLayers(nPrisms);
      //
      pBMO.getDefaultValues().get(PrismWallThickness.class).setValue(firstCellThickness);
      //
      PrismThickness prismThickness_0 = 
      pBMO.getDefaultValues().get(PrismThickness.class);
      prismThickness_0.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
      GenericAbsoluteSize gAS = 
        ((GenericAbsoluteSize) prismThickness_0.getAbsoluteSize());
      gAS.getValue().setValue(prismAbsThickness);

      return pBMO;
  }

  // set up rotations for control surface
  private void setFlapAngles(double f1,double f2,double f3,double f4,
                               double f5,double f6,double f7,double f8){

      setTransformRotationAngle("Kite Rotate Flap A1",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A1A2 Axis"),f1,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A2",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A1A2 Axis"),f2,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A3",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A3A6 Axis"),f3,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A4",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A3A6 Axis"),f4,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A5",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A3A6 Axis"),f5,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A6",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A3A6 Axis"),f6,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A7",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A7A8 Axis"),f7,zeroOrigin,xOnlyAxis);
      setTransformRotationAngle("Kite Rotate Flap A8",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Flap A7A8 Axis"),f8,zeroOrigin,xOnlyAxis);
  }
  private void setHTailAngle(double h1){
      setTransformRotationAngle("Kite Rotate H Tail",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD H Tail Axis"),h1,zeroOrigin,xOnlyAxis);
  }
  private void setRudderAngle(double r1){
      setTransformRotationAngle("Kite Rotate Rudder",labCsys.getCoordinateSystemManager().getCoordinateSystem("Lab CAD Rudder Axis"),r1,zeroOrigin,xOnlyAxis);
  }
  //
  //=============================================
  // Custom Surface Control Settings Methods
  //=============================================
  private void surfNotContainFilter(SurfaceCustomMeshControl custControl,int nDigits){
      String preChar="";
      for(int i=0;i<=nDigits-1;i++){
          preChar = preChar + custControl.getPresentationName().charAt(i);
      }
      custControl.getGeometryObjects().setQuery(new Query(
        new CompoundPredicate(CompoundOperator.Or, 
          Arrays.asList(new QueryPredicate[]{
          new NamePredicate(NameOperator.DoesNotContain, "."+preChar),
          new NamePredicate(NameOperator.DoesNotStartWith, ""+preChar)}))));
  }
  private void surf2ndWildFilter(SurfaceCustomMeshControl custControl){
      String firstChar = ""+custControl.getPresentationName().charAt(0);
      String thirdChar = ""+custControl.getPresentationName().charAt(2);

      custControl.getGeometryObjects().setQuery(new Query(
        new CompoundPredicate(CompoundOperator.Or,
          Arrays.asList(new QueryPredicate[]{
            new NamePredicate(NameOperator.Contains, "."+firstChar+"1"+thirdChar), 
            new NamePredicate(NameOperator.Contains, "."+firstChar+"2"+thirdChar),
            new NamePredicate(NameOperator.Contains, "."+firstChar+"3"+thirdChar),
            new NamePredicate(NameOperator.Contains, "."+firstChar+"4"+thirdChar),
            new NamePredicate(NameOperator.StartsWith, firstChar+"1"+thirdChar),
            new NamePredicate(NameOperator.StartsWith, firstChar+"2"+thirdChar),
            new NamePredicate(NameOperator.StartsWith, firstChar+"3"+thirdChar),
            new NamePredicate(NameOperator.StartsWith, firstChar+"4"+thirdChar)}))));
  }
  private void surf2ndWildFilter(SurfaceCustomMeshControl custControl,int numSurf){
      String firstChar = ""+custControl.getPresentationName().charAt(0);
      String thirdChar = ""+custControl.getPresentationName().charAt(2);

      List<QueryPredicate> surfList = new ArrayList();
      for(int j=1;j<=numSurf;j++){
          surfList.add(new NamePredicate(NameOperator.Contains, "."+firstChar+j+thirdChar));
          surfList.add(new NamePredicate(NameOperator.StartsWith, firstChar+j+thirdChar));
      }

      custControl.getGeometryObjects().setQuery(new Query(
          new CompoundPredicate(CompoundOperator.Or,
          surfList)));


  }
  private void setWingHighYPlusMesh(int nLayers, double firstCellThickness){
      AutoMeshOperation autoMO = 
        ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Kite Volume Mesh"));
      //Modify Main Wing to High Y+ Settings
      SurfaceCustomMeshControl wingSurfCMC = 
        ((SurfaceCustomMeshControl) autoMO.getCustomMeshControls()
                .getObject("100 Main Wing"));
      NumPrismLayers nPrisms = 
        wingSurfCMC.getCustomValues().get(CustomPrismValuesManager.class)
                .get(NumPrismLayers.class);
      nPrisms.setNumLayers(nLayers);
      wingSurfCMC.getCustomValues().get(CustomPrismValuesManager.class)
              .get(PrismWallThickness.class).setValue(firstCellThickness);
  }
  private void setFlapHighYPlusMesh(int nLayers, double firstCellThickness){
      //Modify Flap High Y+ Settings
      AutoMeshOperation autoMO = 
        ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject("Kite Volume Mesh"));
      SurfaceCustomMeshControl flapSurfCMC = 
        ((SurfaceCustomMeshControl) autoMO.getCustomMeshControls().getObject("200 Flaps"));
      NumPrismLayers nFlapPrisms = 
        flapSurfCMC.getCustomValues().get(CustomPrismValuesManager.class)
                .get(NumPrismLayers.class);
      nFlapPrisms.setNumLayers(nLayers);
      flapSurfCMC.getCustomValues().get(CustomPrismValuesManager.class)
              .get(PrismWallThickness.class).setValue(firstCellThickness);

  }
  //
  //=========================
  // VOLUME CONTROL METHODS
  //=========================
  private VolumeCustomMeshControl surfVolumeCntrl(AutoMeshOperation pBMO, String controlName){
      /* Method getCustSurf
          Gets or creates a specified custom surface control
      */

      VolumeCustomMeshControl volControl;
      try{
          volControl = (VolumeCustomMeshControl) pBMO.getCustomMeshControls().getObject(controlName);
      }catch(NeoException e){
          volControl= pBMO.getCustomMeshControls().createVolumeControl();
          volControl.setPresentationName(controlName);
      }
      volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
      return volControl;
  }
  //
  private void volFilter(VolumeCustomMeshControl volControl,int nDigits){
      String preChar = ""+volControl.getPresentationName().substring(0,nDigits-1);
      volControl.getGeometryObjects().setQuery(new Query(
        new CompoundPredicate(CompoundOperator.Or, 
          Arrays.asList(new QueryPredicate[]{
          new NamePredicate(NameOperator.Contains, "."+preChar),
          new NamePredicate(NameOperator.StartsWith, ""+preChar)}))));
  }
  private void custVolSurfSize(VolumeCustomMeshControl volControl, String defRelAbs, double newVal){

      // Enable Surface Remeshing Volume Control
      VolumeControlResurfacerSizeOption volContSurfSizeOption = 
        volControl.getCustomConditions().get(VolumeControlResurfacerSizeOption.class);
      volContSurfSizeOption.setVolumeControlBaseSizeOption(true);

      if(defRelAbs.equals("Absolute")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
          GenericAbsoluteSize gAS = ((GenericAbsoluteSize) vCS.getAbsoluteSize());
          gAS.getValue().setValue(newVal);
      }else if(defRelAbs.equals("Relative")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
          GenericRelativeSize gRS = ((GenericRelativeSize) vCS.getRelativeSize());
          gRS.setPercentage(newVal);
      }
      volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);

   }
  private void assignVolControlPart(VolumeCustomMeshControl volControl,GeometryPart newPart){
      volControl.getGeometryObjects().addPart(newPart);
  }
  private void custVolTrimIsoSize(VolumeCustomMeshControl volControl, String defRelAbs, double newVal){
      // Enable Surface Remeshing Volume Control
      VolumeControlTrimmerSizeOption sizeOption = 
        volControl.getCustomConditions().get(VolumeControlTrimmerSizeOption.class);
      sizeOption.setVolumeControlBaseSizeOption(true);
      sizeOption.setTrimmerAnisotropicSizeOption(false);

      if(defRelAbs.equals("Absolute")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
          GenericAbsoluteSize gAS = ((GenericAbsoluteSize) vCS.getAbsoluteSize());
          gAS.getValue().setValue(newVal);
      }else if(defRelAbs.equals("Relative")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
          GenericRelativeSize gRS = ((GenericRelativeSize) vCS.getRelativeSize());
          gRS.setPercentage(newVal);
      }
      volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
   }
  //
  //=========================
  // SOLVER DRIVERS
  //==========================  
  private ArrayList<MonitorIterationStoppingCriterion> setKiteBasedStoppingCriteria(boolean needRunUnsteady, int convSamp){
    ArrayList<MonitorIterationStoppingCriterion> activeKiteStopCrit = new ArrayList();
    String appEnd = " - It";
    if(needRunUnsteady){
      appEnd = " - UNS";
    }
    double tmpStdDev = 0.;
    String repName = "";
    Monitor tmpMon;
    MonitorIterationStoppingCriterion stopCrit;
    //=================================
    // KITE DATABASE SELECTED VALUES
    //=================================
    // We choose the kite drag because there is no cosine relationship in its
    // value comparedd to CX, the sign does not flip based on alpha, and the
    // drag coefficient of the kite is more indicative of the aerodynamic
    // performance.
    // Kite CD
    repName = "Aero Kite Cd" + appEnd;
    tmpStdDev = 5.0E-4;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu, tmpMon, tmpStdDev, convSamp);
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    //Kite CY
    repName = "Body Kite CY"+appEnd;
    tmpStdDev = 2.0E-3;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu, tmpMon, tmpStdDev, convSamp);
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    // We choose the kite lift because it is related to the power curve and the
    // lift coefficient of the kite is more indicative of the aerodynamic
    // performance.
    repName = "Aero Kite Cl"+appEnd;
    tmpStdDev = 1.0E-2;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu,tmpMon,tmpStdDev,convSamp);
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    //Kite CmX
    repName = "Body Kite CmX"+appEnd;
    tmpStdDev = 2.5E-4;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu,tmpMon,tmpStdDev,convSamp);
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    //Kite CmY
    repName = "Body Kite CmY"+appEnd;
    tmpStdDev = 5.0E-3;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu,tmpMon,tmpStdDev,convSamp);
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    //Kite CmZ
    repName = "Body Kite CmZ"+appEnd;
    tmpStdDev = 2.5E-4;
    tmpMon = simu.getMonitorManager().getObject(repName);
    MonitorTool.setMonitorIterationFrequency((ReportMonitor) tmpMon,1);
    stopCrit =
        (MonitorIterationStoppingCriterion) CriterionTool
            .setItMonStdDevCriteria(simu,tmpMon,tmpStdDev,convSamp); 
    stopCrit.getLogicalOption()
        .setSelected(SolverStoppingCriterionLogicalOption.Type.AND);
    stopCrit.setIsUsed(true);
    activeKiteStopCrit.add(stopCrit);
    
    return activeKiteStopCrit;
  }
  private void disableKiteBasedStoppingCriteria(boolean needRunUnsteady,
    ArrayList<MonitorIterationStoppingCriterion> activeKiteStopCrit){
    for(MonitorIterationStoppingCriterion tmpCrit:activeKiteStopCrit){
      tmpCrit.setIsUsed(false);
    }
  }
  //
  private String omegaFileString(){
    // OMEGA DESCRIPTION
    double[] om_tmp = getXWindOmegaReports();
    String omegaString = "";
    NumberFormat formatter = new DecimalFormat("0.##E0");
    if(Math.abs(om_tmp[0])>1e-6){
        omegaString = omegaString+"_Ox"+formatter.format(om_tmp[0]);
    }
    if(Math.abs(om_tmp[1])>1e-6){
        omegaString = omegaString+"_Oy"+formatter.format(om_tmp[1]);
    }
    if(Math.abs(om_tmp[2])>1e-6){
        omegaString = omegaString+"_Oz"+formatter.format(om_tmp[2]);
    }
    //final string compile
    return omegaString;
  }
  private String getControlSurfaceFileString(){
      String tmpStr="";
      double eps=1.e-6;
      double tmpVal;
      double defFlap=0.0;

      tmpVal = getRotationAngle("Kite Rotate H Tail");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_HT"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Rudder");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_RD"+dblToFileStr(tmpVal);
      }
      // flaps
      tmpVal = getRotationAngle("Kite Rotate Flap A1");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_1F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A2");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_2F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A3");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_3F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A4");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_4F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A5");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_5F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A6");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_6F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A7");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_7F"+dblToFileStr(tmpVal);
      }
      tmpVal = getRotationAngle("Kite Rotate Flap A8");
      if(Math.abs(tmpVal-defFlap)>eps){
          tmpStr=tmpStr+"_8F"+dblToFileStr(tmpVal);
      }

      return tmpStr;
  }
  private String fileStringCompiler(boolean isUnsteady, boolean needGRT,boolean needRunRotors){
      String retStr;
      
      // Mesh Angles
      String kiteMeshSTR = "";
      if(Math.abs(kiteAlpha_o) > SMALL_EPS){
        kiteMeshSTR = kiteMeshSTR + "MSHA" + dblToFileStr(kiteAlpha_o) + "_";
      }
      if(Math.abs(kiteBeta_o) > SMALL_EPS){
        kiteMeshSTR = kiteMeshSTR + "MSHB" + dblToFileStr(kiteBeta_o) +"_";
      }      
      // Gamma Re Theta
      String tmpGRT = "";
      if(needGRT) tmpGRT="GRT";

      // ROTOR FILE STRING
      String needRtr= ""; if(needRunRotors) needRtr="_RTR";
      String tmpRtr =""; 
      if(needRunRotors){
          tmpRtr="";
          tmpRtr=tmpRtr+"_1RL"+getRotorRPMString("Rotor 1 Lower");
          tmpRtr=tmpRtr+"_1RU"+getRotorRPMString("Rotor 1 Upper");
      }

      // unsteady simulation tracker
      String tmpUNS = ""; if(isUnsteady) tmpUNS="UNS";

      //final string compile
      retStr = "M600_"+kiteMeshSTR+tmpUNS+needRtr+"kwSST"+tmpGRT+
              "_V"+dblToFileStr(airSpeed)+"_A"+dblToFileStr(kiteAlpha)+"_B"+
              dblToFileStr(kiteBeta)
              + omegaFileString() + getControlSurfaceFileString() + tmpRtr;

      //
      return retStr;
  }
  //
  private void run_M600_kwSST(int numIterations,boolean isCompressible,boolean needGRT,boolean needRunRotors){
      /* Method to run a RANS angle of attack where the physical body rotates relative to the freestream velocity
      */
      //PRINT OUT HEADER TO LOG
      simu.println("  ==========================================");
      simu.println("   Run M600 Trans-In/OUT:                   ");
      simu.println("     RANS, Menter kw-SST                    ");
      simu.println("  ==========================================");
      simu.println("    ");
      simu.println("    Number of iterations entered:  "+numIterations);
      simu.println("    Using GRT model: "+needGRT);
      simu.println("    ");

      boolean allVolumeOpsUpToDate = areVolumeMeshOperationsUpToDate(needRunRotors);

      simu.println("    ");
      simu.println("    Flight Conditions:");
      simu.println("    Speed: "+airSpeed);
      simu.println("    Alpha: "+kiteAlpha);
      simu.println("    Beta:  "+kiteBeta);

      if(allVolumeOpsUpToDate){
          simu.println("    Mesh Settings:");
          simu.println("    Alpha0: "+getRotationAngle("Kite Rotate Alpha0"));
          simu.println("    Beat0:  "+getRotationAngle("Kite Rotate Beta0"));
          simu.println("    ");
          simu.println("    Flap A1 Angle: "+getRotationAngle("Kite Rotate Flap A1"));
          simu.println("    Flap A2 Angle: "+getRotationAngle("Kite Rotate Flap A2"));
          simu.println("    Flap A3 Angle: "+getRotationAngle("Kite Rotate Flap A3"));
          simu.println("    Flap A4 Angle: "+getRotationAngle("Kite Rotate Flap A4"));
          simu.println("    Flap A5 Angle: "+getRotationAngle("Kite Rotate Flap A5"));
          simu.println("    Flap A6 Angle: "+getRotationAngle("Kite Rotate Flap A6"));
          simu.println("    Flap A7 Angle: "+getRotationAngle("Kite Rotate Flap A7"));
          simu.println("    Flap A8 Angle: "+getRotationAngle("Kite Rotate Flap A8"));
          simu.println("    ");
          simu.println("    H Tail Angle: "+getRotationAngle("Kite Rotate H Tail"));
          simu.println("    Rudder Angle: "+getRotationAngle("Kite Rotate Rudder"));
      }else{
          simu.println("    WARNING: MESH OPERATIONS NOT UP TO DATE - ");
          simu.println("             NO CONTROL SETTINGS INFORMATION.");
      }

      if(needRunRotors){
          simu.println("    ");
          simu.println("    Rotor RPMS:  "+kiteBeta);
          for(int i=1;i<=4;i++){
              simu.println("      Rotor "+i+" Lower:  "+getRotorSpeed("Rotor "+i+" Lower")
                              +"  Rotor "+i+" Upper:  "+getRotorSpeed("Rotor "+i+" Upper"));
          }
      }

      // GSI SEQUENCING for SEGREGATED SOLVER - EXPERIMENTAL!
      if(false){
        for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          CFD_Physics.set_RANS_KwSST(tmpPhys,true,false,isCompressible,false,needGRT,wallDistanceToFS);
        }
        SolverDriver.coupledGSIInit(simu,10,50,5.0,0.01);
        simu.initializeSolution();
      }

      if(true){ //allows manual shut down of solver
        for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          CFD_Physics.set_RANS_KwSST(tmpPhys,true,true,isCompressible,false,needGRT,wallDistanceToFS);
          //Physics Solver Best Practice Settings
          CFD_Physics.setKWSSTURF(simu, 0.6);
          CFD_Physics.setKWSSTViscosity(simu, 0.8, 1.0E20);
          CFD_Physics.setMinimumReferenceWallDistance(tmpPhys, 1.0e-7);
          if(isCompressible){
            CFD_Physics.setMinimumAllowableTemperature(tmpPhys, 150.0); // Below Freezing
            CFD_Physics.setMaximumAllowableTemperature(tmpPhys, 373.0); // Below Boiling
          }
        }
        
        if(true){
          if(needHighViscosityInit){
            // Track the fact that the case was ReMeshed
            setCaseCondition("HIGH VISC INIT");
            for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
              // tunes the turbulence model to generate large turbulence near the wall
              highViscosityKwSSTInit(tmpPhys.getModelManager().getModel(SstKwTurbModel.class), true);
              // set velocity initial condition to be next to the wall
              // make sure simulation knows about the highly viscous initialization
              stepSimulation(numHighViscItSteps);

              // Output STAR-VIEW state of the flow field
              simu.println("High Viscosity Initialization Completed. Outputting i.c. STAR-View File.");
              Scene stateOfInitializedField = setupKiteCfWithReversedFlow(needRunRotors);
              SceneTool.exportSceneToStarView(simu, cfdScenes + "STARVIEW",
                stateOfInitializedField, "High Viscosity Initialized Field", "M600_CFD_Scenes", true);

              //better turn it off!
              highViscosityKwSSTInit(tmpPhys.getModelManager().getModel(SstKwTurbModel.class), false);
              needHighViscosityInit=false;
            }
          }

//          SolverDriver.setSolverContinuityInit(simu, true,10); // experimental!
//          SolverDriver.setKWBoundaryLayerInit(simu,true);   // experimental!
          if(isCompressible){
            SolverDriver.setEnergyURF(simu,0.8);
          }
          SolverDriver.setkwSSTURF(simu,0.6);
          SolverDriver.setMaxTVR(simu,1.e20,0.8);
        }
        // RUN SIMULATION
        stepSimulation(numIterations);
        
      }
  }
  private void run_M600_DES_kwSST(double timeStepVal, int numInnerIts,boolean needGRT, boolean needRunRotors){
      /* Method to run a DES simulation of the M600
      */
      //PRINT OUT HEADER TO LOG
      simu.println("  ==========================================");
      simu.println("   Run M600 Trans-In/OUT:                   ");
      simu.println("     RANS, Menter DES kw-SST                ");
      simu.println("  ==========================================");
      simu.println("    Time step entered:  "+timeStepVal);
      simu.println("    Inner Iterations:   "+numInnerIts);
      simu.println("    ");
      simu.println("    "+"Using GRT model: "+needGRT);
      simu.println("    ");
      simu.println("    ");
      // FILE NAME STRING COMPILER
      String tmpCaseFileStr = fileStringCompiler(true,needGRT,needRunRotors);

      // SET UP PHYSICS
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
        CFD_Physics.set_DES_KwSST(tmpPhys,true,true,true,false,needGRT,wallDistanceToFS);
      }

      SolverDriver.setTimeStep(simu,timeStepVal);
      SolverDriver.set2ndOrderTimeDisc(simu);
      SolverDriver.setInnerIts(simu, numInnerIts);

      // Switch MRF to RBM
      //  -relies on physics being unsteady
      if(needRunRotors){
          simu.println("Applying RBM to Rotors...");
          applyRotorRigidBodyMotions();
          simu.println("Getting New Automatic Time Step Value...");
          SolverDriver.setTimeStep(simu,getRecommendedTimeStep());
          simu.println("Rotor RBM Sets New Time Step: "+getRecommendedTimeStep());
      }
      // Need to make sure unsteady quantities are tracked now
      setupUnsteadyMonitors();
      postProcessPlots("UNS",false);

      //unsteady scenes of interest


      //Autosave
      AutoSave autoSave_0 = 
        simu.getSimulationIterator().getAutoSave();
      StarUpdate starUpdate_0 = 
        autoSave_0.getStarUpdate();
      autoSave_0.setMaxAutosavedFiles(1);
      starUpdate_0.getUpdateModeOption().setSelected(StarUpdateModeOption.Type.TIMESTEP);
      TimeStepUpdateFrequency timeStepUpdateFrequency_0 = 
      starUpdate_0.getTimeStepUpdateFrequency();
      timeStepUpdateFrequency_0.setTimeSteps(250);

      // RUN SIMULATION
      if(true){
          SolverDriver.setEnergyURF(simu,0.9);
          SolverDriver.setkwSSTURF(simu,0.8);
          SolverDriver.setMaxTVR(simu,1.e20,0.8);
      }
      simu.getSolverManager().getSolver(WallDistanceSolver.class).setFrozen(false);
      simu.getSimulationIterator().step(1);
      //freeze wall distance solver for rotors
      simu.getSolverManager().getSolver(WallDistanceSolver.class).setFrozen(true);
      //run out simulation
      simu.getSimulationIterator().run(true);
      simu.saveState(simPathName+File.separator+caseName+tmpCaseFileStr+".sim");
}
  //
  //===================================
  // PHYSICS METHODS
  //===================================
  private ScalarGlobalParameter getCaseStartIterationParameter(){
    String paramName = "GP-CaseStartIteration";
    return getScalarParameter(paramName);
  }
  private ScalarGlobalParameter getScalarParameter(String paramName){
    ScalarGlobalParameter retParam;
    try{
      retParam = (ScalarGlobalParameter) simu.get(GlobalParameterManager.class).getObject(paramName);
    }catch(NeoException e){
      retParam = (ScalarGlobalParameter) simu.get(GlobalParameterManager.class).createGlobalParameter(ScalarGlobalParameter.class, "Scalar");
      retParam.setPresentationName(paramName);
    }
    return retParam;
  }
  // Automatically decides how the simulation should proceed
  private void stepSimulation(int numSteps){
      /* Method to run simulation for numSteps *and* also respect additional
          stopping criteria. This should increase simulation efficiency for
          two reasons:
                  1) It is possible to connect to the server and set different
                     values while the sim is running to bring it down early or
                     make adjustments
                  2) Additional stopping criteria can be used
          Pseudo-code:
              If sim stopped because max number of Steps achieved
                  (Note: user may have set other stopping criteria that would
                         stop this run very quickly thereafter)
                  1) add numSteps inner iteration criteria
                  2) keep running
              Else sim stopped because of other stopping criteria
                      If unsteady:
                          1) Check to see if max physical time enabled & exceeded
                              If so, add number of steps to the clock and run
                      Else:
                          1) disable all enabled stopping criteria for 1/10 of numSteps
                          2) get maxSteps, add number of numSteps
                          3) step simulation 1/10 numSteps
                          4) turn on all other stopping criteria
                          5) run simulation
      */
      // Track &| modify the case starting iteration based on whether it is continuing a completed run
      //   or it is just a crashed case
      ScalarGlobalParameter caseStartItParam = getCaseStartIterationParameter();

      simu.println("  ===============================");
      simu.println("   Simulation stepping reporter: ");
      simu.println("  ===============================");
      simu.println("    JAVA driven automatic stepping: "+autoSimStep);
      if(!autoSimStep){//forcibly drive simulation specified number of steps
          simu.println("      Manual override. Stepping "+numSteps+" steps.");
          getActiveSimulation().getSimulationIterator().step(numSteps,true);
      }else{ // automatically run
          SolverStoppingCriterion maxSteps=simu.getSolverStoppingCriterionManager().getSolverStoppingCriterion("Maximum Steps");
          if(maxSteps.getIsUsed()&&maxSteps.getIsSatisfied()){ //simulation simply stopped due to max steps
              int oldNumberOfSteps = ((StepStoppingCriterion) maxSteps).getMaximumNumberSteps();
              // case was previously completed, we set the new case starting iteration
              caseStartItParam.getQuantity().setDefinition(""+oldNumberOfSteps);
              
              //
              simu.println("      Case was previously stopped due to max steps limit.");
              simu.println("        Old step limit criterion: " + oldNumberOfSteps);
              simu.println("        Increasing max steps by:  " + numSteps);

              //
              int tmpNumSteps=((StepStoppingCriterion) maxSteps).getMaximumNumberSteps();
              ((StepStoppingCriterion) maxSteps).setMaximumNumberSteps(tmpNumSteps+numSteps);
              simu.println("        New steps allowed: "+((StepStoppingCriterion) maxSteps).getMaximumNumberSteps());
              simu.println("        Running...");

              // Relax the solvers to ramp over 500 iterations if this is not a fresh case
              if(true){
                  int nRampSteps=500;
                  try{
                      // Segregated Velocity Ramp
                      SegregatedFlowSolver segregatedFlowSolver_0 = 
                        ((SegregatedFlowSolver) simu.getSolverManager()
                                .getSolver(SegregatedFlowSolver.class));
                      VelocitySolver velocitySolver_0 = 
                        segregatedFlowSolver_0.getVelocitySolver();
                      velocitySolver_0.getRampCalculatorManager().getRampCalculatorOption()
                              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
                      LinearRampCalculator linearRampCalculator_0 = 
                        ((LinearRampCalculator) velocitySolver_0.getRampCalculatorManager()
                                .getCalculator());
                      linearRampCalculator_0.setStartIteration(tmpNumSteps);
                      linearRampCalculator_0.setEndIteration(tmpNumSteps+nRampSteps);
                  }catch(NeoException e){

                  }
                  // Segregated Pressure Ramp
                  try{
                      SegregatedFlowSolver segregatedFlowSolver_0 = 
                        ((SegregatedFlowSolver) simu.getSolverManager()
                                .getSolver(SegregatedFlowSolver.class));
                      PressureSolver pressureSolver_0 = 
                        segregatedFlowSolver_0.getPressureSolver();
                      pressureSolver_0.getRampCalculatorManager().getRampCalculatorOption()
                              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
                      LinearRampCalculator linearRampCalculator_1 = 
                        ((LinearRampCalculator) pressureSolver_0.getRampCalculatorManager()
                                .getCalculator());
                      linearRampCalculator_1.setStartIteration(tmpNumSteps);
                      linearRampCalculator_1.setEndIteration(tmpNumSteps+nRampSteps);
                  }catch(NeoException e){

                  }
                  // Segregated Energy Ramp
                  try{
                      SegregatedEnergySolver segregatedEnergySolver_0 = 
                        ((SegregatedEnergySolver) simu.getSolverManager()
                                .getSolver(SegregatedEnergySolver.class));
                      segregatedEnergySolver_0.getFluidRampCalculatorManager().getRampCalculatorOption()
                              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
                      LinearRampCalculator linearRampCalculator_2 = 
                        ((LinearRampCalculator) segregatedEnergySolver_0.getFluidRampCalculatorManager()
                                .getCalculator());
                      linearRampCalculator_2.setStartIteration(tmpNumSteps);
                      linearRampCalculator_2.setEndIteration(tmpNumSteps+nRampSteps);
                  }catch(NeoException e){

                  }
                  // Turbulence Ramp
                  try{
                      KwTurbSolver kwTurbSolver_0 = 
                        ((KwTurbSolver) simu.getSolverManager().getSolver(KwTurbSolver.class));
                      kwTurbSolver_0.getRampCalculatorManager().getRampCalculatorOption()
                              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
                      LinearRampCalculator linearRampCalculator_3 = 
                        ((LinearRampCalculator) kwTurbSolver_0.getRampCalculatorManager()
                                .getCalculator());
                      linearRampCalculator_3.setStartIteration(tmpNumSteps);
                      linearRampCalculator_3.setEndIteration(tmpNumSteps+nRampSteps);
                  }catch(NeoException e){
                  }
              }
              simu.getSimulationIterator().run(true);

          }else{
              if(CFD_Physics.isUnsteady(allUsedFluidPhysics)){//did case just run out of time?
                  PhysicalTimeStoppingCriterion maxTime = ((PhysicalTimeStoppingCriterion)
                          simu.getSolverStoppingCriterionManager().getSolverStoppingCriterion("Maximum Physical Time"));
                  if(maxTime.inUse()&&maxTime.getIsSatisfied()){
                          simu.println("      Case was previously stopped due to max physical time limit. Increasing time limit and running.");
                          int oldNumberOfSteps = simu.getSimulationIterator().getCurrentIteration();
                          // case was previously completed, we set the new case starting iteration
                          caseStartItParam.getQuantity().setDefinition(""+oldNumberOfSteps);

                          double dT = ((ImplicitUnsteadySolver) simu.getSolverManager().getSolver(ImplicitUnsteadySolver.class)).getTimeStep().getInternalValue(); //sec
                          maxTime.setMaximumTime(maxTime.getMaximumTime().getInternalValue()+numSteps*dT);
                          simu.getSimulationIterator().run(true);
                  }
              }else{ //was case was stopped for some other reason
                  simu.println("      Case was previously stopped for non-max step/time limit. Investigating cause:");
                  // 1) Gather all Criterion, but only care about active use ones
                  Collection<SolverStoppingCriterion> allCriterion = simu.getSolverStoppingCriterionManager().getObjects();
                  Collection<SolverStoppingCriterion> ofInterest = new ArrayList();
                  boolean previousStopByCriterion=false;
                  for(SolverStoppingCriterion tmp:allCriterion){
                      if(tmp.inUse()){
                          ofInterest.add(tmp);
                          if(tmp.getIsSatisfied()&&!previousStopByCriterion){
                              simu.println("        ...simulation previously stopped due to pre-existing alternative criterion");
                              previousStopByCriterion=true;
                          }
                      }
                  }
                  if(previousStopByCriterion){ //whatever criterion stopped it should be temporarily turned off
                      for(SolverStoppingCriterion tmp:ofInterest){ 
                          simu.println("        ...temporarily disabling all in-use stopping criterion");
                          tmp.setIsUsed(false); 
                      }
                      simu.println("        ...stepping simulation 1/10 of prescribed steps");
                      simu.getSimulationIterator().step((numSteps-(numSteps%10))/10,true);
                      simu.println("        ...in-use stopping criterion re-enabled");
                      for(SolverStoppingCriterion tmp:ofInterest){ 
                          tmp.setIsUsed(true);
                      }
                  }else{ //the simulation was brought down for some other manual reason and stopped before any in-use criterion activated
                      simu.println("        ...simulation previously stopped for non-criterion based reason.");
                      int currentStepLevel;
                      if(CFD_Physics.isUnsteady(allUsedFluidPhysics)){
                          currentStepLevel =simu.getSimulationIterator().getCurrentTimeLevel();
                      }else{
                          currentStepLevel = simu.getSimulationIterator().getCurrentIteration();
                      }
                      simu.println("         ..current simulation step level: " +currentStepLevel);
                      int desiredStepLevel = currentStepLevel + numSteps;
                      simu.println("         ..desired simulation step level: " +desiredStepLevel);
                      simu.println("         ..adjusting simulation max step criterion " +desiredStepLevel);
                      ((StepStoppingCriterion) maxSteps).setMaximumNumberSteps(desiredStepLevel);
                      ((StepStoppingCriterion) maxSteps).setIsUsed(true);
                  }
                  simu.println("        ...running simulation.");
                  simu.println("  ===============================");
                  simu.println(" ");
                  simu.println(" ");
                  simu.getSimulationIterator().run(true);
              }
          }
      }
  }
  
  //===================================
  // INITIAL CONDITION TOOLS
  //===================================
  // IC - High kwSST viscosity
  private void highViscosityKwSSTInit(SstKwTurbModel sstKwTurbModel,boolean makeHighlyViscous){
    // Manipulates the turbulence model coefficients to force very high turbulent viscosity near the wall
    //  effectively causing a very sticky air
    double defaultBetaStar = 0.09;
    double defaultBeta2 = 0.0828;
    //
    double highViscosityBetaStar = 0.5;
    double highViscosityBeta2 = 0.2;
    //
    try{
      if(makeHighlyViscous){
        sstKwTurbModel.setBetaStar(highViscosityBetaStar);
        sstKwTurbModel.setBeta2(highViscosityBeta2);        
      }else{
        sstKwTurbModel.setBetaStar(defaultBetaStar);
        sstKwTurbModel.setBeta2(defaultBeta2);
      }
    }catch(NeoException e){
      simu.println("highViscosityKwSSTInit: Turbulence model is not of kw-SST type!");
    }
  }
  // IC - Velocity and Pressure
  private void set_VelocityIC(){
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          tmpPhys.getInitialConditions().get(VelocityProfile.class).setMethod(FunctionVectorProfileMethod.class);
          tmpPhys.getInitialConditions().get(VelocityProfile.class).getMethod(FunctionVectorProfileMethod.class).setFieldFunction(get_VelocityIC_FF());
          tmpPhys.getInitialConditions().get(VelocityProfile.class).setCoordinateSystem(inletCsys);
      }
  }
  private void set_PressureIC() {
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          tmpPhys.getInitialConditions().get(InitialPressureProfile.class).setMethod(FunctionScalarProfileMethod.class);
          tmpPhys.getInitialConditions().get(InitialPressureProfile.class).getMethod(FunctionScalarProfileMethod.class).setFieldFunction(get_PressureIC_FF());
      }
  }
  private UserFieldFunction get_VelocityIC_FF() {
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject("Velocity Initial Condition"));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
      uFF.setPresentationName("Velocity Initial Condition");
      uFF.setFunctionName("vel_ic");
      //uFF.setDefinition("[$WallDistance<0.005?0:$WallDistance>0.5?"+airSpeed+":"+airSpeed+"*($WallDistance-0.005)/.495,0.,0.]");
      uFF.setDefinition("[$WallDistance<0.01?0:$WallDistance>0.75?${Case Airspeed}:${Case Airspeed}*($WallDistance-0.01)/(.49+.25),0.,0.]");
      //uFF.setDefinition("[${Case Airspeed},0.,0.]");
      return uFF;
  }
  private UserFieldFunction get_PressureIC_FF() {
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject("Pressure Initial Condition"));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName("Pressure Initial Condition");
      uFF.setFunctionName("pressure_ic");
      uFF.setDefinition("(0+"+refPressure+")+0.5*"+airDensity+"*"+airSpeed+"*"+airSpeed+"-0.5*"+airDensity+"*mag($$vel_ic)*mag($$vel_ic)");
      return uFF;
  }
  private UserFieldFunction setIterationRamp(int endIt){
      UserFieldFunction uFF;
      String ffName="Iteration Ramp";
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName("it_ramp");
      uFF.setDefinition("($Iteration>="+endIt+"?1:$Iteration/"+endIt+")");
      return uFF;
  }
  // IC - XWind
  private UserFieldFunction get_VelocityXWindIC_FF(){
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject("X Wind Velocity I.C."));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      
      FieldFunction wallDecayFF = getWallDecayFunctionFF();
      UserFieldFunction groundVelocityField = SimTool.getUserFF(simu, "u0_Ground");
      String wallFFname = wallDecayFF.getFunctionName();
      String u0FFname = groundVelocityField.getFunctionName();
      
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
      uFF.setPresentationName("X Wind Velocity I.C.");
      uFF.setFunctionName("xwind_vel_ic");
      uFF.setDefinition("[$$"+u0FFname+"[0]*$"+wallFFname+","+
                         "$$"+u0FFname+"[1]*$"+wallFFname+","+
                         "$$"+u0FFname+"[2]*$"+wallFFname+"]");

      return uFF;
  }
  private FieldFunction getWallDecayFunctionFF(){
    String ffName = " Wall Decay - IC";
    UserFieldFunction uFF;
    try{
      uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
    }catch(NeoException e){
      uFF = simu.getFieldFunctionManager().createFieldFunction();
      uFF.setPresentationName(ffName);
    }
    uFF.setFunctionName("walldecay_ic");
    uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
    uFF.setDefinition("(1.0 + 1.0 * tanh(-${WallDistance}/ ("+nominalChord+"*0.1*0.02) ) )");
    return uFF;
  }
  private UserFieldFunction setOmegaRampFF(double om_x, double om_y, double om_z){
      UserFieldFunction uFF;
      String ffName="Omega Ramp";
      String funcName = "omega_ramp";
      double omega_mag = Math.sqrt(om_x*om_x+om_y*om_y+om_z*om_z);

      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName(funcName);
      uFF.setDefinition(""+omega_mag+"*$it_ramp");
      return uFF;
  }
  private void setXWindVelocityIC(double om_x,double om_y,double om_z){
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          tmpPhys.getInitialConditions().get(VelocityProfile.class)
                  .setMethod(FunctionVectorProfileMethod.class);
          tmpPhys.getInitialConditions().get(VelocityProfile.class)
                  .getMethod(FunctionVectorProfileMethod.class)
                  .setFieldFunction(get_VelocityXWindIC_FF());
          
          tmpPhys.getInitialConditions().get(VelocityProfile.class)
                  .setCoordinateSystem(groundCsys);
      }
  }
  // IC - Temperature
  private void set_TemperatureIC() {
      for(PhysicsContinuum tmpPhys:allUsedFluidPhysics){
          tmpPhys.getInitialConditions().get(StaticTemperatureProfile.class).setValue(refPressure/287.058/airDensity);
      }
  }
  //
  //===================================
  // BLADE ELEMENT METHOD for Rotors
  //===================================
  private GeometryPart getBemVC(VirtualDisk bemDisk){

      // Get extent of radial dimension
      DiskGeometryWithBlades diskGeom=bemDisk.getComponentsManager().get(DiskGeometryWithBlades.class);
      double diskRadius=diskGeom.getDiskOuterRadius().getSIValue();

      // Get blade thickness
      double bladeThickness=diskGeom.getDiskThickness().getSIValue();

      // Get coordinate location
      CartesianCoordinateSystem diskCoord=(CartesianCoordinateSystem)
               bodyCsys.getLocalCoordinateSystemManager().getObject(bemDisk.getPresentationName());

      // Create Cylinder
      Units units_0 = 
        simu.getUnitsManager().getPreferredUnits(new IntVector(new int[] {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      MeshPartFactory meshPF = 
        simu.get(MeshPartFactory.class);
      SimpleCylinderPart diskCylinder;
      try{
          diskCylinder = (SimpleCylinderPart) simu.getGeometryPartManager().getObject("VC_BEM_"+bemDisk.getPresentationName());
      }catch(NeoException e){
          diskCylinder = meshPF.createNewCylinderPart(simu.get(SimulationPartManager.class));
      }


      diskCylinder.setDoNotRetessellate(true);
      diskCylinder.setCoordinateSystem(diskCoord);
      diskCylinder.getRadius().setUnits(units_0);
      diskCylinder.getTessellationDensityOption().setSelected(TessellationDensityOption.Type.FINE);
      diskCylinder.setPresentationName("VC_BEM_"+bemDisk.getPresentationName());

      // End Point
      Coordinate coordinate_0 = 
        diskCylinder.getStartCoordinate();
      coordinate_0.setCoordinateSystem(diskCoord);
      coordinate_0.setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {0.0, 0.0, -1.*bladeThickness}));

      // Start Point
      Coordinate coordinate_1 = 
        diskCylinder.getEndCoordinate();
      coordinate_1.setCoordinateSystem(diskCoord);
      coordinate_1.setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {0.0, 0.0, bladeThickness}));

      // Build remaining of cylinder
      diskCylinder.getRadius().setValue(diskRadius*1.2);
      diskCylinder.rebuildSimpleShapePart();
      diskCylinder.setDoNotRetessellate(false);
      return (GeometryPart) diskCylinder;
  }
  private ArrayList<FileTable> getFileTables(String preStr, String postStr){
    /* Method getBEMTables:
        Reload all duplicate BEM Tables, Import any preStr_*_postStr.csv file
    */
    ArrayList<FileTable> retArr=new ArrayList();
    // Pre-loaded BEM models
    Collection<Table> allTables=simu.getTableManager().getObjects();

    for(Table tmpTable:allTables){
        String tmpTabName = tmpTable.getPresentationName();
        if(tmpTable instanceof FileTable&&
           (tmpTabName.startsWith(preStr)&&tmpTabName.endsWith(postStr))){
            retArr.add((FileTable) tmpTable);
        }
    }
    // Use pwd to get csv file tables with BEM_*_FOIL.csv
    File[] files = new File(simPathName).listFiles();
    ArrayList<File> bemFiles=new ArrayList();
    for (File tmpFile:files){
        String fileName = tmpFile.getName();

        if(fileName.startsWith(preStr)&&fileName.endsWith(".csv")){
          String fileNameNoExt = fileName.substring(0,fileName.length()-4);
          try{
              FileTable bemFileTable = 
                (FileTable) simu.getTableManager().getTable(fileNameNoExt);
              bemFileTable.extract();
              simu.println("BEM MSG: Duplicate .csv found, reloaded table: "+fileNameNoExt);
          }catch(NeoException e){
              bemFiles.add(tmpFile);
              simu.println("BEM MSG: No table for: "+fileName+"found . Loading.");
          }
        }
    }

    //Load in all remaining .csv files
    for(File tmpFile:bemFiles){
        String fileName = tmpFile.getName();
        FileTable tmpFileTable = (FileTable) simu.getTableManager()
                .createFromFile(resolvePath(simPathName+File.separator+fileName));
        retArr.add(tmpFileTable);
    }
    return retArr;
  }
  private VirtualDiskForceReport bemForceReport(VirtualDisk tmpDisk,int inpDir){
        PhysicsContinuum physicsContinuum_0 = 
            ((PhysicsContinuum) simu.getContinuumManager().getContinuum("Air"));
        VirtualDiskForceReport vdFR;
        String aLett="";
        if(inpDir==0){
            aLett="X";
        }else if (inpDir==1){
            aLett="Y";
        }else if (inpDir==2){
            aLett="Z";
        }
        String diskReportName="BEM "+tmpDisk.getPresentationName()+" "+aLett+"-Axis Force";
        try{
          vdFR = (VirtualDiskForceReport) simu.getReportManager().getObject(diskReportName);
        }catch(NeoException e){
          vdFR = simu.getReportManager().createReport(VirtualDiskForceReport.class);  
        }

        vdFR.setVirtualDisk(tmpDisk);
        vdFR.setPresentationName(diskReportName);

        if(inpDir==0){
            vdFR.getForceComponentOption().setSelected(VdmForceComponentOption.Type.X_AXIS_FORCE);
        }else if (inpDir==1){
            vdFR.getForceComponentOption().setSelected(VdmForceComponentOption.Type.Y_AXIS_FORCE);
        }else if (inpDir==2){
            vdFR.getForceComponentOption().setSelected(VdmForceComponentOption.Type.Z_AXIS_FORCE);
        }
        return vdFR;
    }
  private VirtualDiskMomentReport bemMomentReport(VirtualDisk tmpDisk,int inpDir){
        PhysicsContinuum physicsContinuum_0 = 
            ((PhysicsContinuum) simu.getContinuumManager().getContinuum("Air"));
        VirtualDiskMomentReport vdMR;
        String aLett="";
        switch (inpDir) {
            case 0:
                aLett="X";
                break;
            case 1:
                aLett="Y";
                break;
            case 2:
                aLett="Z";
                break;
            default:
                break;
        }
        String diskReportName="BEM "+tmpDisk.getPresentationName()+" "+aLett+"-Axis Moment";
        try{
          vdMR = (VirtualDiskMomentReport) simu.getReportManager().getObject(diskReportName);
        }catch(NeoException e){
          vdMR = simu.getReportManager().createReport(VirtualDiskMomentReport.class);
          vdMR.setPresentationName(diskReportName);
        }
        vdMR.setVirtualDisk(tmpDisk);
        switch (inpDir) {
            case 0:
                vdMR.getMomentComponentOption().setSelected(VdmMomentComponentOption.Type.X_AXIS_MOMENT);
                break;
            case 1:
                vdMR.getMomentComponentOption().setSelected(VdmMomentComponentOption.Type.Y_AXIS_MOMENT);
                break;
            case 2:
                vdMR.getMomentComponentOption().setSelected(VdmMomentComponentOption.Type.Z_AXIS_MOMENT);
                break;
            default:
                break;
        }
        return vdMR;
    }
  private VirtualDisk makeBEMRotor(Collection<PhysicsContinuum> activePhysics, 
        String bemName, CartesianCoordinateSystem axisCoord, double rotRate,
        int radialRes, int azimuthalRes, 
        int nBlades, double innerR, double outerR, double bladeThickness,
        ArrayList<FileTable> foilPerf,
        FileTable chordVSr, FileTable twistVSr) {
    VirtualDisk virtualDisk=null;
    for(PhysicsContinuum tmpPhys:activePhysics){
        try{
            tmpPhys.enable(VirtualDiskModel.class);
        }catch(NeoException e) {
            simu.println("Cannot enable Virtual Disk. Check settings in "+tmpPhys.getPresentationName()+".");
        }

        //Set up virtual disk general properties
        VirtualDiskModel vDM=tmpPhys.getModelManager().getModel(VirtualDiskModel.class);
        try{
            virtualDisk=vDM.getVirtualDiskManager().getVirtualDisk(bemName);
        }catch(NeoException e){
            virtualDisk=vDM.getVirtualDiskManager().createDisk(bemName);
        }
        virtualDisk.setActiveMethod(BladeElementMethod.class);
        virtualDisk.setDisplaySourceTerm(true);                

        VirtualDiskInterpolationGrid virtualDiskInterpolationGrid_0 = 
          virtualDisk.getComponentsManager().get(VirtualDiskInterpolationGrid.class);
        virtualDiskInterpolationGrid_0.getRadialResolution().setValue(radialRes);
        virtualDiskInterpolationGrid_0.getAzimuthalResolution().setValue(azimuthalRes);

        // Define Rotor Geometric Properties
        DiskGeometryWithBlades diskGeometryWithBlades_0 = 
          virtualDisk.getComponentsManager().get(DiskGeometryWithBlades.class);
        diskGeometryWithBlades_0.setNumberOfBlades(nBlades);
        diskGeometryWithBlades_0.getDiskInnerRadius().setValue(innerR);
        diskGeometryWithBlades_0.getDiskOuterRadius().setValue(outerR);
        diskGeometryWithBlades_0.getDiskThickness().setValue(bladeThickness);        
        ((NormalAndCoordinateSystem) diskGeometryWithBlades_0
                .getOrientationSpecification()).setCoordinateSystem(axisCoord);

        //Define Airfoil Section Data
        int foilCnt=0;
        String foilName="";
        String foilFracStr="";
        for(FileTable tmpFileTable:foilPerf){
            String tmpName = tmpFileTable.getPresentationName();
            int cnt=0;
            int subStrLoc=0;
            int oldsubStrLoc=subStrLoc;
            char deLimiter='_';
            boolean gotFoil=false;
            boolean gotFoilFrac=false;
            for(int i=0;i<tmpName.length();i++){
                if(tmpName.charAt(i) == deLimiter){
                    cnt+=1;
                    subStrLoc=i;
                }
                //BEM_NACA4412_1p0_Gen3_FOIL
                if(cnt==2&&!gotFoil){ //hit second delimiter, grab airfoil name
                    foilName=tmpName.substring(4,subStrLoc);
                    gotFoil=true;
                }
                if(cnt==3&&!gotFoilFrac){ //hit 3rd delimiter, grab ratio value
                    foilFracStr=tmpName.substring(oldsubStrLoc+1,subStrLoc);
                    gotFoilFrac=true;
                }
                oldsubStrLoc=subStrLoc;
            }
            AirfoilSection foilSection;
            try{
                foilSection = 
                virtualDisk.getComponentsManager().get(AirfoilSectionManager.class)
                   .getAirfoilSection(foilName);
            }catch(NeoException e){
                foilSection = 
                virtualDisk.getComponentsManager().get(AirfoilSectionManager.class)
                  .createAirfoilSection(foilName);
            }

            foilSection.setPresentationName(foilName);
            double foilFrac = Double.parseDouble(foilFracStr.replace("p","."));
            foilSection.setNormalizedDiskSpan(foilFrac);

            //Expect Mach number
            foilSection.getLiftAndDragFunctionOption().setSelected(AerodynamicCoefficientFunctionOption.Type.MACH_NUMBER_SPECIFIED);

            //Lift Coefficient
            LiftCoefficientAngleAndMachTable liftCoefTab = 
              ((LiftCoefficientAngleAndMachTable) foilSection.getLiftCoefficientTable());
            liftCoefTab.setTable(tmpFileTable);
            liftCoefTab.setMachNumber("Mach");
            liftCoefTab.setAngleOfAttack("AoA");
            liftCoefTab.setLiftCoefficient("Cl");
            //Drag Coefficient
            DragCoefficientAngleAndMachTable dragCoefTab = 
                ((DragCoefficientAngleAndMachTable) foilSection.getDragCoefficientTable());
            dragCoefTab.setTable(tmpFileTable);
            dragCoefTab.setMachNumber("Mach");
            dragCoefTab.setAngleOfAttack("AoA");
            dragCoefTab.setDragCoefficient("Cd");
            //increment airfoil table counter
            foilCnt+=1;

        }
        //define Chord distribution
        if(chordVSr!=null){
            virtualDisk.getComponentsManager().get(ChordBladeProperty.class).setActiveMethod(NormalizedDiskSpanTableMethod.class);
            NormalizedDiskSpanTableMethod normSpanTable= 
                ((NormalizedDiskSpanTableMethod) virtualDisk.getComponentsManager().get(ChordBladeProperty.class).getActiveMethod());
            NormalizedDiskSpanInterpolationTable normSpanInterpTable = 
                normSpanTable.getInterpolationTable();
            normSpanInterpTable.setTable(chordVSr);
            normSpanInterpTable.setData("Chord_length");
            normSpanInterpTable.setDiskSpan("r/R");
        }

        //definte twist distribution
        if(twistVSr!=null){
            virtualDisk.getComponentsManager().get(TwistBladeProperty.class).setActiveMethod(NormalizedDiskSpanTableMethod.class);
            NormalizedDiskSpanTableMethod normSpanTable = 
              ((NormalizedDiskSpanTableMethod) virtualDisk.getComponentsManager().get(TwistBladeProperty.class).getActiveMethod());
            NormalizedDiskSpanInterpolationTable normSpanInterpTable = 
              normSpanTable.getInterpolationTable();
            normSpanInterpTable.setTable(twistVSr);
            normSpanInterpTable.setData("twist");
            normSpanInterpTable.setDiskSpan("r/R");                 
        }

        //define rotation rate
        double rtrSpeed = getRotorSpeed(axisCoord.getPresentationName());
        virtualDisk.getComponentsManager().get(VirtualDiskRampManager.class)
                .getRotationRate().setValue(rtrSpeed);


        virtualDisk.getComponentsManager().get(VirtualDiskRampManager.class)
                .getRampCalculatorOption()
                .setSelected(VirtualDiskRampOption.Type.LINEAR_RAMP);
        VirtualDiskLinearRamp vdSpeedRamp = 
          ((VirtualDiskLinearRamp) virtualDisk.getComponentsManager()
                  .get(VirtualDiskRampManager.class).getRampCalculator());
        vdSpeedRamp.setEndIteration(500);
        vdSpeedRamp.setInitialRampValue(rtrSpeed*1./500.);

    }
    return virtualDisk;
}
  //===================================
  // Time Step Methods
  //===================================
  public double getStopTime(){
      double time = ((PhysicalTimeStoppingCriterion) simu.getSolverStoppingCriterionManager().
              getSolverStoppingCriterion("Maximum Physical Time")).getMaximumTime().getValue();
      return time;
  } 
  public double getCurrentTime(){
      double time = simu.getSolution().getPhysicalTime();
      return time;
  }
  public double getTimeStep(){
      return ((ImplicitUnsteadySolver) simu.getSolverManager().getSolver(ImplicitUnsteadySolver.class)).getTimeStep().getSIValue();
  }

  //===================================
  // CASE SETUP/CONDITIONS
  //===================================
  // Case Name annotation
  private SimpleAnnotation caseNameAnnotation(String caseName) {
    SimpleAnnotation caseAnnotation;
    try{
        caseAnnotation = (SimpleAnnotation) simu.getAnnotationManager().getObject("Case Name");
    }catch(NeoException e){
        caseAnnotation = 
            simu.getAnnotationManager().createSimpleAnnotation();
        caseAnnotation.setPresentationName("Case Name");
    }
    caseAnnotation.setText(caseName);
    caseAnnotation.setDefaultHeight(0.065);
    caseAnnotation.setDefaultPosition(new DoubleVector(new double[] {0.05, 0.0075, 0.0}));
    return caseAnnotation;
  }
  private SimpleAnnotation getTextAnnotation(Simulation tmpSim, String tmpName,
    String txtString,double defH, double[] newPosition,boolean needBckgrnd){
      SimpleAnnotation tmpAnn;
      try{
          tmpAnn = (SimpleAnnotation) tmpSim.getAnnotationManager().getObject(tmpName);
      }catch(NeoException e){
          tmpAnn = tmpSim.getAnnotationManager().createSimpleAnnotation();
          tmpAnn.setPresentationName(tmpName);
      }
      tmpAnn.setText(txtString);
      tmpAnn.setDefaultHeight(defH);
      tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
      tmpAnn.setBackground(needBckgrnd);

      return tmpAnn;
  }
  private ImageAnnotation2D getImgAnnotation(Simulation tmpSim,String tmpName, String imgPath,double defH, double[] newPosition){
      ImageAnnotation2D tmpAnn;
      try{
          tmpAnn = (ImageAnnotation2D) tmpSim.getAnnotationManager().getObject(tmpName);
      }catch(NeoException e){
          tmpAnn = tmpSim.getAnnotationManager().createImageAnnotation2D();
          tmpAnn.setPresentationName(tmpName);
      }
      tmpAnn.setFilePath(imgPath);
      tmpAnn.setDefaultHeight(defH);
      tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
      return tmpAnn;
  }
  
  public void initAnnotations(Simulation tmpSim){
      Annotation tmpAnn;
      //annotations
      String fileSep =File.separator;
      File ventiDir = new File(""+fileSep+"san"+fileSep+"Logos");
      boolean ventiFileExists = ventiDir.exists();
      File winDir = new File("C:"+fileSep+"MakaniCFD"+fileSep+"Logos");
      boolean winDirExists = winDir.exists();

      //Keep the order of the deck correct
      standardAnnotations.clear();
      //
      if(ventiFileExists){
        tmpAnn=getImgAnnotation(tmpSim,"Makani Logo",
          ventiDir+fileSep+"MakaniLogo.png",0.075,new double[]{0.05, 0.81, 0.0});
        standardAnnotations.add(tmpAnn);
      }else if(winDirExists){
        tmpAnn=getImgAnnotation(tmpSim,"Makani Logo",
          winDir+fileSep+"MakaniLogo.png",0.075,new double[]{0.05, 0.81, 0.0});
        standardAnnotations.add(tmpAnn);
      }
      //
      tmpAnn=getTextAnnotation(tmpSim,"Conditions","Conditions:",0.035,new double[]{0.1986, 0.95, 0.0},true);
      standardAnnotations.add(tmpAnn);
      tmpAnn=getTextAnnotation(tmpSim,"Wind Speed","\n\u2022 Wind Speed: "
        +(new DecimalFormat("#.##").format(getKiteSpeed()))+ " m/s",0.06,new double[]{.21, 0.870, 0.0},true);
      tmpAnn.setBackground(false);
      standardAnnotations.add(tmpAnn);
      //
      tmpAnn=getTextAnnotation(tmpSim,"Angle Of Attack","\n\u2022 Angle Of Attack: "
        +(new DecimalFormat("#.#").format(getKiteAlpha()))+" deg",0.06,new double[]{.21, 0.835, 0.0},true);
      tmpAnn.setBackground(false);
      standardAnnotations.add(tmpAnn);
      //
      tmpAnn=getTextAnnotation(tmpSim,"Sideslip Angle","\n\u2022 Angle Of Attack: "
        +(new DecimalFormat("#.#").format(getKiteBeta()))+" deg",0.06,new double[]{.21, 0.800, 0.0},true);
      tmpAnn.setBackground(false);
      standardAnnotations.add(tmpAnn);
      //
      DecimalFormat decimalFormat = new DecimalFormat("#,###");
      tmpAnn=getTextAnnotation(tmpSim,"Reynolds Number","\n\u2022 Reynolds #: " 
        +decimalFormat.format(((int) getKiteRefRe() )).replaceAll(",", " ")+"  ",
        0.06,new double[]{.21, 0.765, 0.0},true);
      tmpAnn.setBackground(false);
      standardAnnotations.add(tmpAnn);

      IterationAnnotation iterationAnnotation = 
          ((IterationAnnotation) tmpSim.getAnnotationManager().getObject("Iteration"));
      iterationAnnotation.setShowTimeStep(true);
      iterationAnnotation.setShowPhysicalTime(true);
      iterationAnnotation.setDefaultHeight(0.15);
      iterationAnnotation.setDefaultPosition(new DoubleVector(new double[] {0.660, 0.825, 0.}));
      tmpAnn.setBackground(false);
      standardAnnotations.add(iterationAnnotation);
      
      //Case Name
  //    tmpAnn = caseNameAnnotation(studyName + ": " + caseName);
      tmpAnn = caseNameAnnotation(studyName + System.lineSeparator() + caseName);
      standardAnnotations.add(tmpAnn);

  }
  // Case parameters
  private void setupCaseFreeStreamParameters(){
    // Reference Area
    ScalarGlobalParameter tmpSP = SimTool.getScalarSimulationParameter(simu, "Reference Area");
    tmpSP.setDimensionsVector(new IntVector(
            new int[] {0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    tmpSP.getQuantity().setValue( referenceArea );
    
    // Density
    tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Density");
    tmpSP.setDimensionsVector(new IntVector(
            new int[] {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0}));
    tmpSP.getQuantity().setValue( airDensity );

    // Airspeed
    tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Airspeed");
    tmpSP.setDimensionsVector(new IntVector(
            new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    tmpSP.getQuantity().setValue( airSpeed );

    // Alpha
    tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Alpha");
    tmpSP.getQuantity().setValue( kiteAlpha );

    // Beta
    tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Beta");
    tmpSP.getQuantity().setValue( kiteBeta );

    // Try to remove any Flight Condition from older cases
    // cases pre-v1.02a
    try{
      ClientServerObjectGroup tmpGroup = ReportTool.getReportGroup(simu, "Flight Conditions");
      for(Report tmpReport: tmpGroup.getObjectsOf(Report.class) ){
        try{
          simu.getReportManager().remove(tmpReport);
        }catch(NeoException j){
        }
      }
      simu.getReportManager().getGroupsManager().remove(tmpGroup);
    }catch(NeoException e){
    }
    
    
  }
  private double[] getCaseFreeStreamParameters(){
    return new double[] {getFreeStreamDensity(), getKiteSpeed(), getKiteAlpha(), getKiteBeta()};
  }
  private double getFreeStreamDensity(){
    //Airspeed
    ScalarGlobalParameter tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Density");
    double tmpRho = tmpSP.getQuantity().getInternalValue();
    return tmpRho;
  }
  private double getKiteSpeed(){
    ScalarGlobalParameter tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Airspeed");
    double tmpSpeed = tmpSP.getQuantity().getInternalValue();   
    return tmpSpeed;
  }
  private double getKiteAlpha(){
    ScalarGlobalParameter tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Alpha");
    double tmpAlpha = tmpSP.getQuantity().getInternalValue(); 
    return tmpAlpha;
  }
  private double getKiteBeta(){
    ScalarGlobalParameter tmpSP = SimTool.getScalarSimulationParameter(simu, "Case Beta");
    double tmpBeta = tmpSP.getQuantity().getInternalValue(); 
    return tmpBeta;
  }
  private double getKiteRefRe(){
    return getFreeStreamDensity()*getKiteSpeed()*nominalChord/airViscosity;
  }
  //
  // Crosswind parameters
  private void setupXWindOmegaReports(double om_x, double om_y, double om_z, double tether_roll_angle){
      //Omega Hat X
      ExpressionReport tmpRep;
      try{
          tmpRep=(ExpressionReport) simu.getReportManager().getObject("Omega Hat X");
      }catch(NeoException e){
          tmpRep=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep.setPresentationName("Omega Hat X");
      }
      tmpRep.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep.setDefinition(""+om_x);        

      //Omega Hat Y
      ExpressionReport tmpRep2;
      try{
          tmpRep2=(ExpressionReport) simu.getReportManager().getObject("Omega Hat Y");
      }catch(NeoException e){
          tmpRep2=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep2.setPresentationName("Omega Hat Y");
      }
      tmpRep2.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep2.setDefinition(""+om_y);      
      //Omega Hat Z
      ExpressionReport tmpRep3;
      try{
          tmpRep3=(ExpressionReport) simu.getReportManager().getObject("Omega Hat Z");
      }catch(NeoException e){
          tmpRep3=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep3.setPresentationName("Omega Hat Z");
      }
      tmpRep3.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep3.setDefinition(""+om_z);        
      //Tether Roll Angle
      ExpressionReport tmpRep4;
      try{
          tmpRep4=(ExpressionReport) simu.getReportManager().getObject("Tether Roll Angle");
      }catch(NeoException e){
          tmpRep4=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep4.setPresentationName("Tether Roll Angle");
      }
      tmpRep4.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep4.setDefinition(""+tether_roll_angle);    
  }
  private double[] getXWindOmegaReports(){
      ExpressionReport tmpRep1;
      //omega hat x
      double tmp1; 
      try{
          tmpRep1=(ExpressionReport) simu.getReportManager().getObject("Omega Hat X");
          tmp1 = tmpRep1.getValue();
      } catch(NeoException e){
          tmp1 = 0.0;
      }
      //omega hat y
      double tmp2; 
      ExpressionReport tmpRep2;
      try{

          tmpRep2=(ExpressionReport) simu.getReportManager().getObject("Omega Hat Y");
          tmp2 = tmpRep2.getValue();      
      } catch(NeoException e){
          tmp2 = 0.0;
      }
      //omega hat z
      double tmp3; 
      ExpressionReport tmpRep3;
      try{
          tmpRep3=(ExpressionReport) simu.getReportManager().getObject("Omega Hat Z");
          tmp3=tmpRep3.getValue();
      } catch(NeoException e){
          tmp3 = 0.0;
      }
      //omega hat z
      double tmp4; 
      ExpressionReport tmpRep4;
      try{
          tmpRep4=(ExpressionReport) simu.getReportManager().getObject("Tether Roll Angle");
          tmp4=tmpRep4.getValue();
      } catch(NeoException e){
          tmp4 = 0.0;
      }
      double[] retVect= {tmp1, tmp2, tmp3, tmp4};
      return retVect;
  }
  private void setupXWindOmegaOriginReports(double om_origin_x, double om_origin_y, double om_origin_z){
      //Omega Origin X
      ExpressionReport tmpRep;
      try{
          tmpRep=(ExpressionReport) simu.getReportManager().getObject("Omega Origin X");
      }catch(NeoException e){
          tmpRep=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep.setPresentationName("Omega Origin X");
      }
      tmpRep.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep.setDefinition(""+om_origin_x);        

      //Omega Origin Y
      ExpressionReport tmpRep2;
      try{
          tmpRep2=(ExpressionReport) simu.getReportManager().getObject("Omega Origin Y");
      }catch(NeoException e){
          tmpRep2=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep2.setPresentationName("Omega Origin Y");
      }
      tmpRep2.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep2.setDefinition(""+om_origin_y);      
      //Omega Origin Z
      ExpressionReport tmpRep3;
      try{
          tmpRep3=(ExpressionReport) simu.getReportManager().getObject("Omega Origin Z");
      }catch(NeoException e){
          tmpRep3=simu.getReportManager().createReport(ExpressionReport.class);
          tmpRep3.setPresentationName("Omega Origin Z");
      }
      tmpRep3.setDimensionsVector(new IntVector(
              new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
      tmpRep3.setDefinition(""+om_origin_z);        
  }
  private double[] getXWindOmegaOriginReports(){
      ExpressionReport tmpRep1;
      //omega origin x
      double tmp1; 
      try{
          tmpRep1=(ExpressionReport) simu.getReportManager().getObject("Omega Origin X");
          tmp1 = tmpRep1.getValue();
      } catch(NeoException e){
          tmp1 = 0.0;
      }
      //omega origin y
      double tmp2; 
      ExpressionReport tmpRep2;
      try{

          tmpRep2=(ExpressionReport) simu.getReportManager().getObject("Omega Origin Y");
          tmp2 = tmpRep2.getValue();      
      } catch(NeoException e){
          tmp2 = 0.0;
      }
      //omega origin z
      double tmp3;
      ExpressionReport tmpRep3;
      try{
          tmpRep3=(ExpressionReport) simu.getReportManager().getObject("Omega Origin Z");
          tmp3=tmpRep3.getValue();
      } catch(NeoException e){
          tmp3 = 0.0;
      }
      double[] retVect= {tmp1,tmp2,tmp3};
      return retVect;
  }
  
  // Tracking for how a case starts
  private SimpleAnnotation getCaseConditionAnnotation(){
    String oldCaseName = "Case Condition";
    try{
      SimpleAnnotation tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().getObject(oldCaseName);  
      return tmpAnn;
    }catch(NeoException e){
      SimpleAnnotation tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().createSimpleAnnotation();
      tmpAnn.setPresentationName(oldCaseName);
      return tmpAnn;
    }
  }
  private String getCaseCondition(){
    return getCaseConditionAnnotation().getText();
  }
  private void setCaseCondition(String caseCondition){
    SimpleAnnotation tmpAnn = getCaseConditionAnnotation();
    tmpAnn.setText(caseCondition);
  }
  //
  private SimpleAnnotation getOldCaseNameAnnotation(){
    String oldCaseName = "Old Case Name";
    try{
      SimpleAnnotation tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().getObject(oldCaseName);  
      return tmpAnn;
    }catch(NeoException e){
      SimpleAnnotation tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().createSimpleAnnotation();
      tmpAnn.setPresentationName(oldCaseName);
      return tmpAnn;
    }
  }
  private String getOldCaseName(){
    return getOldCaseNameAnnotation().getText();
  }
  private void setOldCaseName(String oldCaseName){
    SimpleAnnotation tmpAnn = getOldCaseNameAnnotation();
    tmpAnn.setText(oldCaseName);
  }
  //
  //===========================================
  // ADDITIONAL USER FIELD FUNCTION METHODS
  //===========================================
  private UserFieldFunction get_windVel_FF(){
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject("Wind Axis Velocity"));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
      uFF.setPresentationName("Wind Axis Velocity");
      uFF.setFunctionName("wind_axis_velocity");
      uFF.setDefinition("[${Case Airspeed}*$it_ramp,0,0]");
      return uFF;
  }
  private UserFieldFunction get_RotorRamp_FF(int startRampIts, int FinalRampIts){
      UserFieldFunction uFF;
      try{
          uFF = (UserFieldFunction) simu.getFieldFunctionManager().getObject("Rotor Ramp");
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
          uFF.setPresentationName("Rotor Ramp");
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setFunctionName("rotor_ramp");
      if(FinalRampIts==0){
          uFF.setDefinition("1");    
      }else{
          uFF.setDefinition("($Iteration<"+startRampIts+"?0:($Iteration<"+FinalRampIts+")?($Iteration-"+startRampIts+")/"+startRampIts+":1)");    
      }

      return uFF;
  }
  //
  //=============================================
  // REGIONS Methods
  //=============================================
  // M600 Case Setup Methods
  private Region setupKiteRegion(GeometryPart regPart){
      Region kiteRegion;
      String kiteName = "Kite";
      try{
          kiteRegion = simu.getRegionManager().getObject(kiteName);
      }catch(NeoException e){
          kiteRegion = 
              simu.getRegionManager().createEmptyRegion();
          kiteRegion.setPresentationName(kiteName);
      }
      // Clear existing boundaries
      for(Boundary tmp:kiteRegion.getBoundaryManager().getObjects()){
          kiteRegion.getBoundaryManager().remove(tmp);
      }

      Boundary inletBndy = set_InletBC("000 Inlet");
      Boundary switchBndy = set_InletBC("002 Switch");
      Boundary outletBndy = set_OutletBC();

      // 100s
      String bndyName = "100 Main Wing";
      Boundary wingBndy;
      try{
          wingBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          wingBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      // 200s
      bndyName = "200 Flaps";
      Boundary flapBndy;
      try{
          flapBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          flapBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      // 300s
      bndyName = "300 Actuators";
      Boundary actuatorBndy;
      try{
          actuatorBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          actuatorBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      // 400s
      bndyName = "400 Pylons";
      Boundary pylonBndy;
      try{
          pylonBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          pylonBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      // 500s
      bndyName = "500 Empennage";
      Boundary empBndy;
      try{
          empBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          empBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }

      // 600s
      bndyName = "600 Fuselage";
      Boundary fuseBndy;
      try{
          fuseBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          fuseBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      // 409s
      bndyName = "409 Radiator Interfaces";
      Boundary radIntBndy;
      try{
          radIntBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          radIntBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      radIntBndy.setBoundaryType(SymmetryBoundary.class);
      // 890s
      bndyName = "890 Rotor Interfaces";
      Boundary rotrIntBndy;
      try{
          rotrIntBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          rotrIntBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      rotrIntBndy.setBoundaryType(SymmetryBoundary.class);

      //================================
      // Setup & Associate Geometry
      //================================
      kiteRegion.getPartGroup().setObjects(regPart);
      ArrayList<PartSurface> surf000s = new ArrayList();
      ArrayList<PartSurface> surf001s = new ArrayList();
      ArrayList<PartSurface> surf002s = new ArrayList();
      ArrayList<PartSurface> surf100s = new ArrayList();
      ArrayList<PartSurface> surf200s = new ArrayList();
      ArrayList<PartSurface> surf300s = new ArrayList();
      ArrayList<PartSurface> surf400s = new ArrayList();
      ArrayList<PartSurface> surf409s = new ArrayList();
      ArrayList<PartSurface> surf500s = new ArrayList();
      ArrayList<PartSurface> surf600s = new ArrayList();
      ArrayList<PartSurface> surf890s = new ArrayList();

      for(PartSurface tmp:regPart.getPartSurfaces()){
          //Basic setup
          if(tmp.getPresentationName().startsWith("000")||tmp.getPresentationName().contains(".000")){
              surf000s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("001")||tmp.getPresentationName().contains(".001")){
              surf001s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("002")||tmp.getPresentationName().contains(".002")){
              surf002s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("1")||tmp.getPresentationName().contains(".1")){
              surf100s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("2")||tmp.getPresentationName().contains(".2")){
              surf200s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("3")||tmp.getPresentationName().contains(".3")){
              surf300s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("4")||tmp.getPresentationName().contains(".4")){
              surf400s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("5")||tmp.getPresentationName().contains(".5")){
              surf500s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("6")||tmp.getPresentationName().contains(".6")){
              surf600s.add(tmp);
          }
          //Special Interface surface
          if((tmp.getPresentationName().startsWith("4") &&
             (tmp.getPresentationName().startsWith("3", 2)||tmp.getPresentationName().startsWith("3", 2)))
             || tmp.getPresentationName().contains(".413")
             || tmp.getPresentationName().contains(".423")
             || tmp.getPresentationName().contains(".433")
             || tmp.getPresentationName().contains(".443")
             || tmp.getPresentationName().contains(".414")
             || tmp.getPresentationName().contains(".424")
             || tmp.getPresentationName().contains(".434")
             || tmp.getPresentationName().contains(".444")) {
              surf409s.add(tmp);
          }
          if(tmp.getPresentationName().startsWith("89")||tmp.getPresentationName().contains(".89")){
              surf890s.add(tmp);
          }
      }
      inletBndy.getPartSurfaceGroup().setObjects(surf000s);
      outletBndy.getPartSurfaceGroup().setObjects(surf001s);
      switchBndy.getPartSurfaceGroup().setObjects(surf002s);
      wingBndy.getPartSurfaceGroup().setObjects(surf100s);
      flapBndy.getPartSurfaceGroup().setObjects(surf200s);
      actuatorBndy.getPartSurfaceGroup().setObjects(surf300s);
      pylonBndy.getPartSurfaceGroup().setObjects(surf400s);
      empBndy.getPartSurfaceGroup().setObjects(surf500s);
      fuseBndy.getPartSurfaceGroup().setObjects(surf600s);
      //special assignments get ripped from other boundaries
      radIntBndy.getPartSurfaceGroup().setObjects(surf409s);
      rotrIntBndy.getPartSurfaceGroup().setObjects(surf890s);
      return kiteRegion;
  }
  private ArrayList<Region> setupRadiatorRegions(ArrayList<GeometryPart> radPart){
      ArrayList<Region> allRadReg = new ArrayList();

      for(GeometryPart tmpRad:radPart){
          Region radRegion;
          String radName = tmpRad.getPresentationName();
          try{
              radRegion = simu.getRegionManager().getObject(radName);
              simu.println("Region: "+radName+" exists. Leaving alone.");
          }catch(NeoException e){
              radRegion = 
              simu.getRegionManager().createEmptyRegion();
              radRegion.setPresentationName(radName);
              radRegion.setRegionType(PorousRegion.class);


              //
              // Clear existing boundaries
  //            for(Boundary tmp:radRegion.getBoundaryManager().getObjects()){
  //                radRegion.getBoundaryManager().remove(tmp);
  //            }
              //
              //Inlet Setup
              Boundary inletBndy;
              try{
                  inletBndy = radRegion.getBoundaryManager().getBoundary("710 Inlet");
              }catch (NeoException j){
                  inletBndy = 
                      radRegion.getBoundaryManager().createEmptyBoundary("710 Inlet");
              }    
              inletBndy.setBoundaryType(SymmetryBoundary.class);
              Boundary sidesBndy;
              try{
                  sidesBndy = radRegion.getBoundaryManager().getBoundary("700 Sides");
              }catch (NeoException j){
                  sidesBndy = 
                      radRegion.getBoundaryManager().createEmptyBoundary("700 Sides");
              }
              Boundary outletBndy;
              try{
                  outletBndy = radRegion.getBoundaryManager().getBoundary("720 Outlet");
              }catch (NeoException j){
                  outletBndy = 
                      radRegion.getBoundaryManager().createEmptyBoundary("720 Outlet");
              }
              outletBndy.setBoundaryType(SymmetryBoundary.class);
              radRegion.getPartGroup().setObjects(tmpRad);
              ArrayList<PartSurface> surf700s = new ArrayList();
              ArrayList<PartSurface> surf710s = new ArrayList();
              ArrayList<PartSurface> surf720s = new ArrayList();
              for(PartSurface tmp:tmpRad.getPartSurfaces()){
                  //Basic setup
                  if(tmp.getPresentationName().startsWith("7")||tmp.getPresentationName().contains(".7")){
                      surf700s.add(tmp);
                  }
                  if(tmp.getPresentationName().startsWith("71")||tmp.getPresentationName().contains(".71")){
                      surf710s.add(tmp);
                  }
                  if(tmp.getPresentationName().startsWith("72")||tmp.getPresentationName().contains(".72")){
                      surf720s.add(tmp);
                  }
              }

              sidesBndy.getPartSurfaceGroup().setObjects(surf700s);
              inletBndy.getPartSurfaceGroup().setObjects(surf710s);
              outletBndy.getPartSurfaceGroup().setObjects(surf720s);

              // setup radiator region coordinate system
              if(radName.contains("Lower")){
                  CartesianCoordinateSystem regCsys =
                          (CartesianCoordinateSystem) bodyCsys.getLocalCoordinateSystemManager().getObject("Lower Radiators");
                  setupRadiatorResistance(radRegion,regCsys,224.0,10000.0, 10000.0,178.61,10000.0,10000.0);
              }else {
                  CartesianCoordinateSystem regCsys =
                          (CartesianCoordinateSystem) bodyCsys.getLocalCoordinateSystemManager().getObject("Upper Radiators");
                  setupRadiatorResistance(radRegion,regCsys,224.0,10000.0, 10000.0,178.61,10000.0,10000.0);
              }

          }
          allRadReg.add(radRegion);
      }

      return allRadReg;
  }
  private ArrayList<Region> setupRotorRegions(ArrayList<GeometryPart> rtrParts){
      ArrayList<Region> allRotorReg = new ArrayList();
      for(GeometryPart tmpRtr:rtrParts){

          Region rotorRegion;
          String rotorName = tmpRtr.getPresentationName().substring(7);
          simu.println(rotorName);
          try{
              rotorRegion = simu.getRegionManager().getRegion(rotorName);

          }catch(NeoException e){
              rotorRegion = 
          simu.getRegionManager().createEmptyRegion();
          rotorRegion.setPresentationName(rotorName);
          }
          rotorRegion.setRegionType(FluidRegion.class);
          //
          // Clear existing boundaries
          for(Boundary tmp:rotorRegion.getBoundaryManager().getObjects()){
              rotorRegion.getBoundaryManager().remove(tmp);
          }
          //
          //Inlet Setup
          Boundary rotorBndy;
          try{
              rotorBndy = rotorRegion.getBoundaryManager().getBoundary("800 Rotors");
          }catch (NeoException e){
              rotorBndy = 
                  rotorRegion.getBoundaryManager().createEmptyBoundary("800 Rotors");
          }    
          Boundary hubBndy;
          try{
              hubBndy = rotorRegion.getBoundaryManager().getBoundary("860 Hub");
          }catch (NeoException e){
              hubBndy = 
                  rotorRegion.getBoundaryManager().createEmptyBoundary("860 Hub");
          }
          Boundary intBndy;
          try{
              intBndy = rotorRegion.getBoundaryManager().getBoundary("890 Rotor Interface");
          }catch (NeoException e){
              intBndy = 
                  rotorRegion.getBoundaryManager().createEmptyBoundary("890 Rotor Interface");
          }
          intBndy.setBoundaryType(SymmetryBoundary.class);
          rotorRegion.getPartGroup().setObjects(tmpRtr);
          ArrayList<PartSurface> allSurfs = new ArrayList();
          ArrayList<PartSurface> surf800s = new ArrayList();
          ArrayList<PartSurface> surf860s = new ArrayList();
          ArrayList<PartSurface> surf890s = new ArrayList();
          for(PartSurface tmp:tmpRtr.getPartSurfaces()){
              //Basic setup
              if(tmp.getPresentationName().startsWith("8")||tmp.getPresentationName().contains(".8")){
                  surf800s.add(tmp);
              }
              if(tmp.getPresentationName().startsWith("86")||tmp.getPresentationName().contains(".86")){
                  surf860s.add(tmp);
              }
              if(tmp.getPresentationName().startsWith("89")||tmp.getPresentationName().contains(".89")){
                  surf890s.add(tmp);
              }

              allSurfs.add(tmp);
          }

          //add everything into the hub boundary (in case Mesh Op pieces from kite)
          hubBndy.getPartSurfaceGroup().setObjects(allSurfs);
          rotorBndy.getPartSurfaceGroup().setObjects(surf800s);
          intBndy.getPartSurfaceGroup().setObjects(surf890s);
          allRotorReg.add(rotorRegion);
          applyMRFMotions(rotorRegion);
      }

      return allRotorReg;
  }
  //
  // Boundary Conditions
  // inlets
  private Boundary set_InletBC(String bndyName){
      Region kiteRegion=simu.getRegionManager().getRegion("Kite");
      //000 Inlet Setup
      Boundary inletBndy;
      try{
          inletBndy = kiteRegion.getBoundaryManager().getBoundary(bndyName);
      }catch (NeoException e){
          inletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary(bndyName);
          inletBndy.setPresentationName(bndyName);
      }
      inletBndy.setBoundaryType(InletBoundary.class);
      inletBndy.getConditions().get(FlowDirectionOption.class).setSelected(FlowDirectionOption.Type.COMPONENTS);
      FlowDirectionProfile flowDirectionProfile = 
        inletBndy.getValues().get(FlowDirectionProfile.class);
      flowDirectionProfile.setCoordinateSystem(inletCsys);
      
      //Set velocity at inlet

      VelocityMagnitudeProfile vMP = 
        inletBndy.getValues().get(VelocityMagnitudeProfile.class);
      vMP.setMethod(ConstantScalarProfileMethod.class);
      vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setDefinition("${Case Airspeed}");

      return inletBndy;
  }
  private Boundary set_InletBC(){
      Region kiteRegion=simu.getRegionManager().getRegion("Kite");
      //000 Inlet Setup
      Boundary inletBndy;
      try{
          inletBndy = kiteRegion.getBoundaryManager().getBoundary("000 Inlet");
      }catch (NeoException e){
          inletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("000 Inlet");
          inletBndy.setPresentationName("000 Inlet");
      }
      inletBndy.setBoundaryType(InletBoundary.class);
      inletBndy.getConditions().get(FlowDirectionOption.class).setSelected(FlowDirectionOption.Type.COMPONENTS);
      FlowDirectionProfile flowDirectionProfile = 
        inletBndy.getValues().get(FlowDirectionProfile.class);
      flowDirectionProfile.setCoordinateSystem(inletCsys);
      //Set velocity at inlet
      VelocityMagnitudeProfile vMP = 
        inletBndy.getValues().get(VelocityMagnitudeProfile.class);
      vMP.setMethod(ConstantScalarProfileMethod.class);
      vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setDefinition("${Case Airspeed}");
      
      //002 Switch Setup -- Added for XWind Cases to Avoid B.C. impingement
      // always assume we are setting up for a normal no-omega case
      try{
          inletBndy = kiteRegion.getBoundaryManager().getBoundary("002 Switch");
      }catch (NeoException e){
          inletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("002 Switch");
          inletBndy.setPresentationName("002 Switch");
      }
      inletBndy.setBoundaryType(InletBoundary.class);
      inletBndy.getConditions().get(FlowDirectionOption.class).setSelected(FlowDirectionOption.Type.COMPONENTS);
      flowDirectionProfile.setCoordinateSystem(inletCsys);
      //Set velocity at inlet
      vMP.setMethod(ConstantScalarProfileMethod.class);
      vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setDefinition("${Case Airspeed}");
//      vMP.getMethod(FunctionScalarProfileMethod.class).setFieldFunction(userFieldFunction_0);

      return inletBndy;
  }
  private Boundary setInletToLabZero_BC(){
      Region kiteRegion=simu.getRegionManager().getRegion("Kite");
      //000 Inlet Setup
      Boundary inletBndy;
      try{
          inletBndy = kiteRegion.getBoundaryManager().getBoundary("000 Inlet");
      }catch (NeoException e){
          inletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("000 Inlet");
          inletBndy.setPresentationName("000 Inlet");
      }
      inletBndy.setBoundaryType(InletBoundary.class);
      inletBndy.getConditions().get(FlowDirectionOption.class).setSelected(FlowDirectionOption.Type.COMPONENTS);
      FlowDirectionProfile flowDirectionProfile = 
        inletBndy.getValues().get(FlowDirectionProfile.class);
      flowDirectionProfile.setCoordinateSystem(inletCsys);
      VelocityMagnitudeProfile velocityMagnitudeProfile_0 =
      inletBndy.getValues().get(VelocityMagnitudeProfile.class);
      velocityMagnitudeProfile_0.setMethod(ConstantScalarProfileMethod.class);
      velocityMagnitudeProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(0.0);

      return inletBndy;
  }

private Boundary setInletToWind_BC(CartesianCoordinateSystem groundCsys,
        double speedMag, double omega_ground_x,double omega_ground_y,double omega_ground_z,
        double r_BwrtGround_x,  double r_BwrtGround_y,  double r_BwrtGround_z,
        double alpha, double beta, double phi){
    
      Region kiteRegion=simu.getRegionManager().getRegion("Kite");
      //000 Inlet Setup
      Boundary inletBndy;
      try{
          inletBndy = kiteRegion.getBoundaryManager().getBoundary("000 Inlet");
      }catch (NeoException e){
          inletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("000 Inlet");
          inletBndy.setPresentationName("000 Inlet");
      }
      inletBndy.setBoundaryType(InletBoundary.class);
      inletBndy.getConditions().get(InletVelocityOption.class).setSelected(InletVelocityOption.Type.COMPONENTS);
      
      //001 Outlet Setup
      Boundary outletBndy;
      try{
          outletBndy = kiteRegion.getBoundaryManager().getBoundary("001 Outlet");
      }catch (NeoException e){
          outletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("001 Outlet");
          outletBndy.setPresentationName("001 Outlet");
      }
      outletBndy.setBoundaryType(InletBoundary.class);
      outletBndy.getConditions().get(InletVelocityOption.class).setSelected(InletVelocityOption.Type.COMPONENTS);

      // WIND INFORMATION
      // Body rotation matrix is defined relative to the unperturbed body coordinate system; it is left alone
      double[][] omega_mat = new double[3][3];
      omega_mat[0][0] =             0.0; omega_mat[0][1] = -omega_ground_z; omega_mat[0][2] =  omega_ground_y;
      omega_mat[1][0] =  omega_ground_z; omega_mat[1][1] =      0.0; omega_mat[1][2] = -omega_ground_x;
      omega_mat[2][0] = -omega_ground_y; omega_mat[2][1] =  omega_ground_x; omega_mat[2][2] =      0.0;
      
      // The origin r vec is the rBodywrtGround
      double[] origin_r_vec  = {r_BwrtGround_x, r_BwrtGround_y, r_BwrtGround_z};
      double[] kiteInertialVelocity = SimTool.transformVector(omega_mat, origin_r_vec);
      simu.println("Kite Velocity In Ground Frame: " 
              + kiteInertialVelocity[0] + "," +kiteInertialVelocity[1] + "," + kiteInertialVelocity[2]);

      // Wind is always in the X-component of the wind axis
      // which is defined relative to perturbed body, this is aligned with aero
      // 'x'
      //
      // Apparent wind vector is always defined relative to Body
      // must be converted to CCM Coordinates *then* to Ground Coordinates
      //
      double[] windVec = {speedMag, 0.0, 0.0};
      double[][] b2sa = getR_Y(-alpha*Math.PI/180.);
      double[][] stab2aw = getR_Z(beta*Math.PI/180.);
      double[][] aw2inlet = getR_Y(Math.PI);
      double[][] lab2ground = getR_X(-1.*phi*Math.PI/180.);

      double[] inletwind_in_aw = SimTool.transformVector(SimTool.get3x3Transpose(aw2inlet),windVec); // inverse is transpose
      simu.println("Apparent Wind: " + inletwind_in_aw[0] + "," +inletwind_in_aw[1] + "," +inletwind_in_aw[2]);
      double[] inletwind_in_stab = SimTool.transformVector(SimTool.get3x3Transpose(stab2aw),inletwind_in_aw);
      simu.println("Apparent Wind In Stability: " + inletwind_in_stab[0] + "," +inletwind_in_stab[1] + "," +inletwind_in_stab[2]);
      double[] inletwind_in_body = SimTool.transformVector(SimTool.get3x3Transpose(b2sa),inletwind_in_stab);
      simu.println("Apparent Wind In Body: " + inletwind_in_body[0] + "," +inletwind_in_body[1] + "," +inletwind_in_body[2]);
      double[] inletwind_in_ground = SimTool.transformVector(SimTool.get3x3Transpose(lab2ground),inletwind_in_body);
      simu.println("Apparent Wind In Ground: " + inletwind_in_ground[0] + "," +inletwind_in_ground[1] + "," +inletwind_in_ground[2]);
      
      
      // This is the ground Wind in the non-perturbed system
      double[] groundWind = {inletwind_in_ground[0] + kiteInertialVelocity[0],
                             inletwind_in_ground[1] + kiteInertialVelocity[1],
                             inletwind_in_ground[2] + kiteInertialVelocity[2]};

      simu.println("Ground Wind: " + groundWind[0] + "," +groundWind[1] + "," +groundWind[2]);
      
      // Rotating motion velocity vector field functions
      UserFieldFunction u0_ground_FF = SimTool.getUserVectorFF(simu,"u0_Ground");
      u0_ground_FF.setFunctionName("u0_ground");
      u0_ground_FF.setDefinition("["+groundWind[0]+","+groundWind[1]+","+groundWind[2]+"]");
      
      UserFieldFunction u0_lab_FF = SimTool.getUserVectorFF(simu,"u0_Lab");
      u0_lab_FF.setFunctionName("u0_lab");
      u0_lab_FF.setDefinition("[$${u0_ground}[0],"
              + "$${u0_ground}[1]*cos($phi*$pi/180) - $${u0_ground}[2]*sin($phi*$pi/180),"
              + "$${u0_ground}[1]*sin($phi*$pi/180) + $${u0_ground}[2]*cos($phi*$pi/180)]");
      
      
      // Background wind velocity vector field functions
      UserFieldFunction u0_body_FF = SimTool.getUserVectorFF(simu,"u0_Body");
      u0_body_FF.setFunctionName("u0_body");
      u0_body_FF.setDefinition("$$u0_ground(@CoordinateSystem(\""
          + labCsys.getPresentationName() + "." + bodyCsys.getPresentationName()
          + "\"))");

      // Background wind velocity vector field functions
      UserFieldFunction windErrorFF = SimTool.getUserVectorFF(simu,"Wind Error ");
      windErrorFF.setFunctionName("wind_error");
      windErrorFF.setDefinition("($$Velocity(@CoordinateSystem(\""
          + labCsys.getPresentationName() + "." + groundCsys.getPresentationName()
          + "\"))-$$u0_ground)/(mag($$u0_ground)+1e-20)");

      // Background wind velocity vector field functions
      UserFieldFunction windErrorPctFF = SimTool.getUserVectorFF(simu, "Wind Error Percentage");
      windErrorPctFF.setFunctionName("wind_error_pct");
      windErrorPctFF.setDefinition("($$Velocity(@CoordinateSystem(\""
          + labCsys.getPresentationName() + "." + groundCsys.getPresentationName()
          + "\"))-$$u0_ground)/(mag($$u0_ground)+1e-20)*100.0");

      // Wind Error Metric
      // We use the Xwind Ground Lab coordinate system, which is an inertial frame
      // INLET
      VelocityProfile velocityProfile = inletBndy.getValues().get(VelocityProfile.class);
      velocityProfile.setCoordinateSystem(groundCsys);
      velocityProfile.setMethod(FunctionVectorProfileMethod.class);
      velocityProfile.getMethod(FunctionVectorProfileMethod.class).setFieldFunction(u0_ground_FF);

      // OUTLET
      velocityProfile = outletBndy.getValues().get(VelocityProfile.class);
      velocityProfile.setCoordinateSystem(groundCsys);
      velocityProfile.setMethod(FunctionVectorProfileMethod.class);
      velocityProfile.getMethod(FunctionVectorProfileMethod.class).setFieldFunction(u0_ground_FF);

      return inletBndy;
  }


 private double[][] getR_X(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] = 1; retMat[0][1] =                0; retMat[0][2] =               0;
   retMat[1][0] = 0; retMat[1][1] =  Math.cos(angle); retMat[1][2] = Math.sin(angle);
   retMat[2][0] = 0; retMat[2][1] = -Math.sin(angle); retMat[2][2] = Math.cos(angle);
   return retMat;
 }
 
 private double[][] getR_Y(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] = Math.cos(angle); retMat[0][1] = 0; retMat[0][2] = -Math.sin(angle);
   retMat[1][0] =               0; retMat[1][1] = 1; retMat[1][2] =                0;
   retMat[2][0] = Math.sin(angle); retMat[2][1] = 0; retMat[2][2] =  Math.cos(angle);
   return retMat;
 }
 
  private double[][] getR_Z(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] =  Math.cos(angle); retMat[0][1] =  Math.sin(angle); retMat[0][2] = 0;
   retMat[1][0] = -Math.sin(angle); retMat[1][1] =  Math.cos(angle); retMat[1][2] = 0;
   retMat[2][0] =                0; retMat[2][1] =                0; retMat[2][2] = 1;
   return retMat;
 }


  private Boundary setSwitchToOutlet_BC(){
    Region kiteRegion=simu.getRegionManager().getRegion("Kite");
    //Outlet Setup
    Boundary switchBndy;
    try{
        switchBndy = kiteRegion.getBoundaryManager().getBoundary("002 Switch");
    }catch (NeoException e){
        switchBndy = 
            kiteRegion.getBoundaryManager().createEmptyBoundary("002 Switch");
    }
    switchBndy.setBoundaryType(PressureBoundary.class);
    //switchBndy.getValues().get(StaticTemperatureProfile.class).setValue(refPressure/287.058/airDensity);

    // Outlet pressure field functions
    UserFieldFunction pressBC_FF = SimTool.getUserScalarFF(simu,"MRF Press BC");
    pressBC_FF.setFunctionName("mrf_press_bc");
    pressBC_FF.setDefinition(" $Density*($$omega_lab[2] * $$u0_lab[1] - $$omega_lab[1] * $$u0_lab[2])*$$Position[0]"
                           + "+$Density*($$omega_lab[0] * $$u0_lab[2] - $$omega_lab[2] * $$u0_lab[0])*$$Position[1]"
                           + "+$Density*($$omega_lab[1] * $$u0_lab[0] - $$omega_lab[0] * $$u0_lab[1])*$$Position[2]");

    StaticPressureProfile staticPressureProfile = 
      switchBndy.getValues().get(StaticPressureProfile.class);
    staticPressureProfile.setMethod(FunctionScalarProfileMethod.class);
    staticPressureProfile.getMethod(FunctionScalarProfileMethod.class)
        .setFieldFunction(pressBC_FF);
    
    // Backflow specification
    BackflowSpecification backflowSpecification = 
      switchBndy.getConditions().get(BackflowSpecification.class);
    backflowSpecification.getBackflowVelocityDirectionOption().setSelected(BackflowVelocityDirectionOption.Type.COMPONENTS);
    backflowSpecification.getReversedFlowPressureOption().setSelected(ReversedFlowPressureOption.Type.ENVIRONMENTAL);

    FlowDirectionProfile backFlowDirection = 
      switchBndy.getValues().get(FlowDirectionProfile.class);
    
    backFlowDirection.setCoordinateSystem(groundCsys);
    
    backFlowDirection.setMethod(FunctionVectorProfileMethod.class);
   
    UserFieldFunction userFF = SimTool.getUserFF(simu, "u0_Ground");
    backFlowDirection.getMethod(FunctionVectorProfileMethod.class).setFieldFunction(userFF);
    
    return switchBndy;
  }
    

  
  // outlet
  private Boundary set_OutletBC(){
      Region kiteRegion=simu.getRegionManager().getRegion("Kite");
      //Outlet Setup
      Boundary outletBndy;
      try{
          outletBndy = kiteRegion.getBoundaryManager().getBoundary("001 Outlet");
      }catch (NeoException e){
          outletBndy = 
              kiteRegion.getBoundaryManager().createEmptyBoundary("001 Outlet");
      }
      outletBndy.setBoundaryType(PressureBoundary.class);
      try{
          outletBndy.getValues().get(StaticTemperatureProfile.class).setValue(refPressure/287.058/airDensity);
      }catch(NeoException e){
        
      }

      return outletBndy;
  }
  // Utility
  private Collection<NamedObject> allWallParts(Region kiteRegion){
      ArrayList<NamedObject> retList = new ArrayList();
      for(Boundary tmp:kiteRegion.getBoundaryManager().getBoundaries()){
          if(tmp.getBoundaryType() instanceof WallBoundary){
              retList.add(tmp);
          }
      }
      return retList;
  }
  
  private Collection<NamedObject> allWallParts(Collection<Region> allRegions){
      ArrayList<NamedObject> retList = new ArrayList();

      for(Region tmpReg:allRegions){
          for(Boundary tmp:tmpReg.getBoundaryManager().getBoundaries()){
              if(tmp.getBoundaryType() instanceof WallBoundary){
                  retList.add(tmp);
              }
          }
      }
      return retList;
  }    
  //
  // Physics/Reference Frame Methods
  private void applyLabRefFrame(Collection<Region> regList){
      for(Region tmpReg:regList){
          //Apply Lab Reference Frame
          MotionSpecification motionSpecification_0 = 
            tmpReg.getValues().get(MotionSpecification.class);
          LabReferenceFrame labRefFrame = 
            ((LabReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("Lab Reference Frame"));
          motionSpecification_0.setReferenceFrame(labRefFrame);
      }
  }
  private RotatingMotion applyMRFMotions(Region rotorReg){
      String rotorName = rotorReg.getPresentationName();
      RotatingMotion rotMotion=getRotatingMotion(rotorName);

      rotMotion.getAxisDirection().setComponents(0.0, 0.0, -1.0);
      CartesianCoordinateSystem motionCsys = 
        ((CartesianCoordinateSystem) bodyCsys.getLocalCoordinateSystemManager().getObject(rotorName));
      rotMotion.setCoordinateSystem(motionCsys);

      //assign mRF
      MotionSpecification motionSpecification_0 = 
        rotorReg.getValues().get(MotionSpecification.class);
      RotatingReferenceFrame mRF = 
        ((RotatingReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("ReferenceFrame for "+rotorName));
      motionSpecification_0.setReferenceFrame(mRF);
      return rotMotion;
  }
  
  private double[][] getBodyToStabilityRotMat(double alphad){
    
    // alpha to rad
    double alpha = alphad*Math.PI/180.0;
    
    // Alpha rotation matrix
    double[][] alphaMat = new double[3][3];
    alphaMat[0][0] =  Math.cos(alpha); alphaMat[0][1] = 0.0; alphaMat[0][2] = Math.sin(alpha);
    alphaMat[1][0] =              0.0; alphaMat[1][1] = 1.0; alphaMat[1][2] = 0.0;
    alphaMat[2][0] = -Math.sin(alpha); alphaMat[2][1] = 0.0; alphaMat[2][2] = Math.cos(alpha);
    
    return alphaMat;
  }
  
    private double[][] getStabilityToWindRotMat(double betad){
    
    // alpha to rad
    double beta = betad*Math.PI/180.0;
    
    // Alpha rotation matrix
    double[][] betaMat = new double[3][3];
    betaMat[0][0] =  Math.cos(beta); betaMat[0][1] = Math.sin(beta); betaMat[0][2] = 0.0;
    betaMat[1][0] = -Math.sin(beta); betaMat[1][1] = Math.cos(beta); betaMat[1][2] = 0.0;
    betaMat[2][0] =             0.0; betaMat[2][1] = 0.0;            betaMat[2][2] = 1;
    
    return betaMat;
  }
  
  
  private RotatingMotion setXWindRotatingMotion(CartesianCoordinateSystem groundCsys,
          double omega_ground_x, double omega_ground_y, double omega_ground_z){
    
    // The expectation for a crosswind MRF case is that the body coordinate
    // system is aligned with the Laboratory coordinate system and is at the
    // origin of the lab coordinate system, or (0,0,0). The omega vector
    // is expressed in the Body Coordinate system. Therefore, we need to first
    // express the velocity vector of the kite (which is in the wind coordiante
    // system) into the body coordinate system. We can then calculate the
    // location of the origin of the MRF relative to the zero origin, which is
    // why the negative sign appears in the final origin location.
    RotatingMotion rotMotion = getRotatingMotion("Kite XWind MRF");
    
    // We use the Ground Coordinate system as our origin and because this
    // coordinate system is pre-set at the correct CCM lab origin location
    // and orientation, the origin of the motion is at the origin of the
    // Ground coordinate system.
    rotMotion.setCoordinateSystem(groundCsys);
    rotMotion.getOriginVector().setComponents(0.0, 0.0, 0.0);

    // Set up the MRF Motion for the kite in crosswind, using the unperturbed body orientation
    rotMotion.getAxisDirection().setComponents(omega_ground_x, omega_ground_y, omega_ground_z);

    // set magnitude of omega
    rotMotion.getRotationRate().setDefinition("$omega_mag");

    return rotMotion;
  }

  private void applyXwindRotatingReferenceFrame(Collection<Region> regList){
      for(Region tmpReg:regList){
          //assign transAndRotRF
          MotionSpecification motionSpecification_0 = 
            tmpReg.getValues().get(MotionSpecification.class);
          RotatingReferenceFrame rotAndTransRefFrame = 
          ((RotatingReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("ReferenceFrame for Kite XWind MRF"));
          motionSpecification_0.setReferenceFrame(rotAndTransRefFrame);
      }
  }
  
  private void applyTransAndRotRefFrame(Collection<Region> regList){
      for(Region tmpReg:regList){
          //assign transAndRotRF
          MotionSpecification motionSpecification_0 = 
            tmpReg.getValues().get(MotionSpecification.class);
          RotatingAndTranslatingReferenceFrame rotAndTransRefFrame = 
          ((RotatingAndTranslatingReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("ReferenceFrame for Kite Wind Axis XWind"));
          motionSpecification_0.setReferenceFrame(rotAndTransRefFrame);
      }
  }

  private boolean doesCaseNeedTransitionModel(double apparentWindAlpha){
    // The alpha=3.0 value comes from the physics and wind tunnel tests
    // the alpha=18.0 would be clearly a post-stall condition.
    if(apparentWindAlpha >= 3.0 && apparentWindAlpha <= 16.0) {
      return true;
    }else {
      return false;
    }
  }
  // Interfaces Methods
  private void setupRadiatorInterfaces(Region kiteRegion,ArrayList<Region> allRadRegions){
      simu.println("These all exist");
      simu.println(simu.getInterfaceManager().getObjects());

      for(Region tmpReg:allRadRegions){
          simu.println("In radiator interfaces for: "+tmpReg.getPresentationName());
          //Inlet
          BoundaryInterface bndyInt;
          String inletIntName = tmpReg.getPresentationName()+" Inlet";
          try{
              simu.println("Checking "+inletIntName+" exists...");
              bndyInt=((BoundaryInterface) simu.getInterfaceManager().getInterface(inletIntName));
              simu.println("...confirmed "+inletIntName+" interface exists.");
          }catch(NeoException e){
              simu.println("...exception, "+inletIntName+" does not exist.");
              Boundary masterBndy = 
                  kiteRegion.getBoundaryManager().getBoundary("409 Radiator Interfaces");
              Boundary bndy2 = 
                  tmpReg.getBoundaryManager().getBoundary("710 Inlet");
              bndyInt = 
              simu.getInterfaceManager().createBoundaryInterface(
                      masterBndy,bndy2 , "Interface");
              bndyInt.setPresentationName(inletIntName);
          }

          // Outlet
          BoundaryInterface bndyInt2;
          String outletIntName = tmpReg.getPresentationName()+" Outlet";
          try{
              simu.println("Checking "+outletIntName+" interface exists...");
              //simu.getInterfaceManager().getObject(outletIntName);
              bndyInt2 = ((BoundaryInterface) simu.getInterfaceManager().getInterface(outletIntName));
              simu.println("...confirmed "+outletIntName+" interface exists.");
          }catch(NeoException e){
              simu.println("...exception, "+outletIntName+" does not exist.");
              Boundary masterBndy = 
                  kiteRegion.getBoundaryManager().getBoundary("409 Radiator Interfaces");
              Boundary bndy3 = 
              tmpReg.getBoundaryManager().getBoundary("720 Outlet");
              bndyInt2 = 
              simu.getInterfaceManager().createBoundaryInterface(
                      masterBndy,bndy3 , "Interface");
              bndyInt2.setPresentationName(outletIntName);
          }
      }
      simu.println("done w/ setup Radiator interfaces");
  }
  private void setupRotorInterfaces(Region kiteRegion,Collection<Region> allRotorRegions){
      Boundary masterBndy = 
        kiteRegion.getBoundaryManager().getBoundary("890 Rotor Interfaces");
      for(Region tmpReg:allRotorRegions){
          Boundary bndy2 = 
              tmpReg.getBoundaryManager().getBoundary("890 Rotor Interface");
          BoundaryInterface bndyInt = 
              simu.getInterfaceManager().createBoundaryInterface(masterBndy,bndy2 , "Interface");
          bndyInt.setPresentationName(tmpReg.getPresentationName()+" Interface");
      }
  }
  private void setupRadiatorResistance(Region radRegion, CoordinateSystem regCsys,double xxIres, double yyIres, double zzIres,double xxVres, double yyVres, double zzVres){
      // Inertial Resistance Terms
      PorousInertialResistance porousInertialResistance_0 = 
        radRegion.getValues().get(PorousInertialResistance.class);
      //x-axis
      VectorProfile vectorProfile_0 = 
        porousInertialResistance_0.getMethod(PrincipalTensorProfileMethod.class).getXAxis();
      vectorProfile_0.getMethod(ConstantVectorProfileMethod.class).getQuantity().setComponents(1.0, 0.0, 0.0);
      vectorProfile_0.setCoordinateSystem(regCsys);
      //y-axis
      VectorProfile vectorProfile_1 = 
        porousInertialResistance_0.getMethod(PrincipalTensorProfileMethod.class).getYAxis();
      vectorProfile_1.getMethod(ConstantVectorProfileMethod.class).getQuantity().setComponents(0.0, 1.0, 0.0);
      ScalarProfile scalarProfile_0 = 
          porousInertialResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(0);
      scalarProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue( xxIres);
      ScalarProfile scalarProfile_1 = 
        porousInertialResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(1);
      scalarProfile_1.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(yyIres);
      ScalarProfile scalarProfile_2 = 
        porousInertialResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(2);
      scalarProfile_2.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(zzIres);

      // Viscous resistance terms
      PorousViscousResistance porousViscousResistance_0 = 
        radRegion.getValues().get(PorousViscousResistance.class);
      VectorProfile vectorProfile_2 = 
        porousViscousResistance_0.getMethod(PrincipalTensorProfileMethod.class).getXAxis();
      vectorProfile_2.getMethod(ConstantVectorProfileMethod.class).getQuantity().setComponents(1.0, 0.0, 0.0);
      vectorProfile_2.setCoordinateSystem(regCsys);
      VectorProfile vectorProfile_3 = 
        porousViscousResistance_0.getMethod(PrincipalTensorProfileMethod.class).getYAxis();
      vectorProfile_3.getMethod(ConstantVectorProfileMethod.class).getQuantity().setComponents(0.0, 1.0, 0.0);
      ScalarProfile scalarProfile_3 = 
        porousViscousResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(0);
      scalarProfile_3.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue( xxVres);
      ScalarProfile scalarProfile_4 = 
        porousViscousResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(1);
      scalarProfile_4.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue( yyVres);
      ScalarProfile scalarProfile_5 = 
        porousViscousResistance_0.getMethod(PrincipalTensorProfileMethod.class).getProfile(2);
      scalarProfile_5.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue( zzVres);
  }

  //=============================================
  // Aero Toolkit Objects
  //=============================================
	public void makeKitePitots() {
    String pitotName;
    double xPos=0;
    double yPos=0;
    double zPos=0;
    CartesianCoordinateSystem htailCsys=(CartesianCoordinateSystem) bodyCsys.
            getLocalCoordinateSystemManager().
            getObject("CAD H Tail Axis");
			
    Collection<Region> tmpRegs = gatherBodyRegions();
    tmpRegs.addAll(gatherRotatingBodyRegions());
			
			
    // Mass Balance Pitot
    pitotName = "MassBalance";
    xPos=   3.269;
    yPos=   0.0;
    zPos=   0.446;
    allPitots.add(new PitotProbe(simu, bodyCsys,tmpRegs,pitotName, new double[] {xPos, yPos, zPos}));

    // Port Pitot
    pitotName = "PortTip";
    xPos=   0.108;
    yPos= -12.802;
    zPos=  -0.220;
    allPitots.add(new PitotProbe(simu, bodyCsys,tmpRegs,pitotName, new double[] {xPos, yPos, zPos}));

    // Stbd Pitot
    pitotName = "StbdTip";
    xPos=   0.302;
    yPos=  12.802;
    zPos=   0.394;
    allPitots.add(new PitotProbe(simu, bodyCsys,tmpRegs,pitotName, new double[] {xPos, yPos, zPos}));

    // HTail Pitot
    pitotName = "HTail";
    xPos=  -6.010;
    yPos=  -0.032;
    zPos=   0.817;
    allPitots.add(new PitotProbe(simu, bodyCsys,tmpRegs,pitotName, new double[] {xPos, yPos, zPos}));

    // Rotor Pitots
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_000", new double[] {	1.613	,	4.214	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_045", new double[] {	1.591720912	,	4.045586399	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_090", new double[] {	1.582906825	,	3.639	,	2.171211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_135", new double[] {	1.591720912	,	3.232413601	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_180", new double[] {	1.613	,	3.064	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_225", new double[] {	1.634279088	,	3.232413601	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_270", new double[] {	1.643093175	,	3.639	,	1.022788018	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.50_az_315", new double[] {	1.634279088	,	4.045586399	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_000", new double[] {	1.613	,	4.559	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_045", new double[] {	1.578953459	,	4.289538239	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_090", new double[] {	1.56485092	,	3.639	,	2.515739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_135", new double[] {	1.578953459	,	2.988461761	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_180", new double[] {	1.613	,	2.719	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_225", new double[] {	1.647046541	,	2.988461761	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_270", new double[] {	1.66114908	,	3.639	,	0.678260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbo_r_0.80_az_315", new double[] {	1.647046541	,	4.289538239	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_000", new double[] {	1.613	,	1.788	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_045", new double[] {	1.591720912	,	1.619586399	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_090", new double[] {	1.582906825	,	1.213	,	2.171211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_135", new double[] {	1.591720912	,	0.8064136008	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_180", new double[] {	1.613	,	0.638	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_225", new double[] {	1.634279088	,	0.8064136008	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_270", new double[] {	1.643093175	,	1.213	,	1.022788018	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.50_az_315", new double[] {	1.634279088	,	1.619586399	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_000", new double[] {	1.613	,	2.133	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_045", new double[] {	1.578953459	,	1.863538239	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_090", new double[] {	1.56485092	,	1.213	,	2.515739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_135", new double[] {	1.578953459	,	0.5624617613	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_180", new double[] {	1.613	,	0.293	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_225", new double[] {	1.647046541	,	0.5624617613	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_270", new double[] {	1.66114908	,	1.213	,	0.678260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sbi_r_0.80_az_315", new double[] {	1.647046541	,	1.863538239	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_000", new double[] {	1.613	,	-0.638	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_045", new double[] {	1.591720912	,	-0.8064136008	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_090", new double[] {	1.582906825	,	-1.213	,	2.171211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_135", new double[] {	1.591720912	,	-1.619586399	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_180", new double[] {	1.613	,	-1.788	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_225", new double[] {	1.634279088	,	-1.619586399	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_270", new double[] {	1.643093175	,	-1.213	,	1.022788018	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.50_az_315", new double[] {	1.634279088	,	-0.8064136008	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_000", new double[] {	1.613	,	-0.293	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_045", new double[] {	1.578953459	,	-0.5624617613	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_090", new double[] {	1.56485092	,	-1.213	,	2.515739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_135", new double[] {	1.578953459	,	-1.863538239	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_180", new double[] {	1.613	,	-2.133	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_225", new double[] {	1.647046541	,	-1.863538239	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_270", new double[] {	1.66114908	,	-1.213	,	0.678260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbi_r_0.80_az_315", new double[] {	1.647046541	,	-0.5624617613	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_000", new double[] {	1.613	,	-3.064	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_045", new double[] {	1.591720912	,	-3.232413601	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_090", new double[] {	1.582906825	,	-3.639	,	2.171211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_135", new double[] {	1.591720912	,	-4.045586399	,	2.003029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_180", new double[] {	1.613	,	-4.214	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_225", new double[] {	1.634279088	,	-4.045586399	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_270", new double[] {	1.643093175	,	-3.639	,	1.022788018	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.50_az_315", new double[] {	1.634279088	,	-3.232413601	,	1.190970813	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_000", new double[] {	1.613	,	-2.719	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_045", new double[] {	1.578953459	,	-2.988461761	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_090", new double[] {	1.56485092	,	-3.639	,	2.515739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_135", new double[] {	1.578953459	,	-4.289538239	,	2.246646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_180", new double[] {	1.613	,	-4.559	,	1.597	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_225", new double[] {	1.647046541	,	-4.289538239	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_270", new double[] {	1.66114908	,	-3.639	,	0.678260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pbo_r_0.80_az_315", new double[] {	1.647046541	,	-2.988461761	,	0.9473533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_000", new double[] {	1.96	,	-3.064	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_045", new double[] {	1.938720912	,	-3.232413601	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_090", new double[] {	1.929906825	,	-3.639	,	-0.6417880175	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_135", new double[] {	1.938720912	,	-4.045586399	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_180", new double[] {	1.96	,	-4.214	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_225", new double[] {	1.981279088	,	-4.045586399	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_270", new double[] {	1.990093175	,	-3.639	,	-1.790211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.50_az_315", new double[] {	1.981279088	,	-3.232413601	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_000", new double[] {	1.96	,	-2.719	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_045", new double[] {	1.925953459	,	-2.988461761	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_090", new double[] {	1.91185092	,	-3.639	,	-0.297260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_135", new double[] {	1.925953459	,	-4.289538239	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_180", new double[] {	1.96	,	-4.559	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_225", new double[] {	1.994046541	,	-4.289538239	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_270", new double[] {	2.00814908	,	-3.639	,	-2.134739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pto_r_0.80_az_315", new double[] {	1.994046541	,	-2.988461761	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_000", new double[] {	1.96	,	-0.638	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_045", new double[] {	1.938720912	,	-0.8064136008	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_090", new double[] {	1.929906825	,	-1.213	,	-0.6417880175	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_135", new double[] {	1.938720912	,	-1.619586399	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_180", new double[] {	1.96	,	-1.788	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_225", new double[] {	1.981279088	,	-1.619586399	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_270", new double[] {	1.990093175	,	-1.213	,	-1.790211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.50_az_315", new double[] {	1.981279088	,	-0.8064136008	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_000", new double[] {	1.96	,	-0.293	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_045", new double[] {	1.925953459	,	-0.5624617613	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_090", new double[] {	1.91185092	,	-1.213	,	-0.297260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_135", new double[] {	1.925953459	,	-1.863538239	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_180", new double[] {	1.96	,	-2.133	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_225", new double[] {	1.994046541	,	-1.863538239	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_270", new double[] {	2.00814908	,	-1.213	,	-2.134739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Pti_r_0.80_az_315", new double[] {	1.994046541	,	-0.5624617613	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_000", new double[] {	1.96	,	1.788	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_045", new double[] {	1.938720912	,	1.619586399	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_090", new double[] {	1.929906825	,	1.213	,	-0.6417880175	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_135", new double[] {	1.938720912	,	0.8064136008	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_180", new double[] {	1.96	,	0.638	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_225", new double[] {	1.981279088	,	0.8064136008	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_270", new double[] {	1.990093175	,	1.213	,	-1.790211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.50_az_315", new double[] {	1.981279088	,	1.619586399	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_000", new double[] {	1.96	,	2.133	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_045", new double[] {	1.925953459	,	1.863538239	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_090", new double[] {	1.91185092	,	1.213	,	-0.297260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_135", new double[] {	1.925953459	,	0.5624617613	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_180", new double[] {	1.96	,	0.293	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_225", new double[] {	1.994046541	,	0.5624617613	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_270", new double[] {	2.00814908	,	1.213	,	-2.134739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sti_r_0.80_az_315", new double[] {	1.994046541	,	1.863538239	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_000", new double[] {	1.96	,	4.214	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_045", new double[] {	1.938720912	,	4.045586399	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_090", new double[] {	1.929906825	,	3.639	,	-0.6417880175	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_135", new double[] {	1.938720912	,	3.232413601	,	-0.8099708133	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_180", new double[] {	1.96	,	3.064	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_225", new double[] {	1.981279088	,	3.232413601	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_270", new double[] {	1.990093175	,	3.639	,	-1.790211982	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.50_az_315", new double[] {	1.981279088	,	4.045586399	,	-1.622029187	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_000", new double[] {	1.96	,	4.559	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_045", new double[] {	1.925953459	,	4.289538239	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_090", new double[] {	1.91185092	,	3.639	,	-0.297260828	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_135", new double[] {	1.925953459	,	2.988461761	,	-0.5663533014	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_180", new double[] {	1.96	,	2.719	,	-1.216	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_225", new double[] {	1.994046541	,	2.988461761	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_270", new double[] {	2.00814908	,	3.639	,	-2.134739172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Rotor_Sto_r_0.80_az_315", new double[] {	1.994046541	,	4.289538239	,	-1.865646699	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_000", new double[] {	1.679	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_045", new double[] {	1.239660172	,	5.5	,	1.060660172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_090", new double[] {	0.179	,	5.5	,	1.5	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_135", new double[] {	-0.8816601718	,	5.5	,	1.060660172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_180", new double[] {	-1.321	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_225", new double[] {	-0.8816601718	,	5.5	,	-1.060660172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_270", new double[] {	0.179	,	5.5	,	-1.5	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_1.5_az_315", new double[] {	1.239660172	,	5.5	,	-1.060660172	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_000", new double[] {	3.179	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_045", new double[] {	2.300320344	,	5.5	,	2.121320344	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_090", new double[] {	0.179	,	5.5	,	3	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_135", new double[] {	-1.942320344	,	5.5	,	2.121320344	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_180", new double[] {	-2.821	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_225", new double[] {	-1.942320344	,	5.5	,	-2.121320344	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_270", new double[] {	0.179	,	5.5	,	-3	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_3.0_az_315", new double[] {	2.300320344	,	5.5	,	-2.121320344	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_000", new double[] {	5.179	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_045", new double[] {	3.714533906	,	5.5	,	3.535533906	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_090", new double[] {	0.179	,	5.5	,	5	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_135", new double[] {	-3.356533906	,	5.5	,	3.535533906	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_180", new double[] {	-4.821	,	5.5	,	0	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_225", new double[] {	-3.356533906	,	5.5	,	-3.535533906	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_270", new double[] {	0.179	,	5.5	,	-5	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"Wing_Circle_X_0.18_Y_5.5_r_5.0_az_315", new double[] {	3.714533906	,	5.5	,	-3.535533906	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"pitot_center_port", new double[] {3.213, 0,	0.443	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"pitot_0.15m_fwd", new double[] {3.36279, 0,	0.450845	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"pitot_0.30m_fwd", new double[] {3.51258, 0,	0.45869	}));
    allPitots.add(new PitotProbe(simu,bodyCsys,tmpRegs,"pitot_0.50m_fwd", new double[] {3.7123, 0,	0.46915	}));
  }
  private ArrayList<PitotProbe> getAllPitotTubeObjects(){
    ArrayList<PitotProbe> retList = new ArrayList();
    Collection<Part> allDerivedParts = simu.getPartManager().getObjects();
    String preFix = "Pitot_";
    for(Part tmpPart:allDerivedParts){
      String pitotName = tmpPart.getPresentationName();
      if (tmpPart instanceof PointPart && pitotName.startsWith("Pitot")){
        simu.println("  Found PitotProbe: "+pitotName);
        long initTime = System.nanoTime();
        PitotProbe tmpPitot = new PitotProbe(simu, bodyCsys,tmpPart.getInputPartsCollection(),
                                pitotName.substring(preFix.length()-1),
                                ((PointPart)tmpPart).getPoint().toDoubleArray());
        long endTime = System.nanoTime();
        simu.println("     ...time to generate: "+(endTime-initTime)/1E9);
        // Add this pitot to the list
        tmpPitot.sortReports();
        tmpPitot.sortMonitors();
      
        // Add Pitot Tube to List of Pitot Tubes
        retList.add(tmpPitot);
      
      } // pitot setup complete
    }
    simu.println("  Found: "+retList.size()+ " pitot objects.");
    
    return retList;
  }
  //
  //=============================================
  // Derived Parts Methods
  //=============================================
  private void setupDerivedParts(Collection<Region> myRegs){
    // Create Threshold Parts to Potentially ID bad areas
    VolumeMeshTool.makeMeshQualityDerivedParts(simu,myRegs);
    
    DerivedPartTool.singlePlane(simu,myRegs,"Body - Center Cut",bodyCsys,zeroOrigin,yOnlyAxis);
    DerivedPartTool.singlePlane(simu,myRegs,"Body - Horizontal Cut",bodyCsys,zeroOrigin,zOnlyAxis);
    //multiPlane(myRegs,"Center Multislice",bodyCsys,yOnlyAxis,zeroOrigin, 23,-11.,11.);
    

    //
    //MakePitot Tubes
    //makeKitePitots();

    //Make Wing Histogram

  }
  private PrimitiveFieldFunction getCaseVelocityReferenceFramePrimitiveFF(boolean useMRFFrame){
    PrimitiveFieldFunction pFF = 
      ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
    if(useMRFFrame){
      // Reversed Flow IsoSurface
      RotatingReferenceFrame caseReferenceFrame = (RotatingReferenceFrame) simu.get(ReferenceFrameManager.class)
              .getObject("ReferenceFrame for Kite XWind MRF");
      pFF = (PrimitiveFieldFunction) pFF.getFunctionInReferenceFrame(caseReferenceFrame);
    }
    return pFF;
  }
  private VectorComponentFieldFunction getCaseInletVelocityComponent(boolean useMRFFrame, int componentDirection){
    VectorComponentFieldFunction vCFF;
    PrimitiveFieldFunction pFF = getCaseVelocityReferenceFramePrimitiveFF(useMRFFrame);
    vCFF = ((VectorComponentFieldFunction) pFF.getFunctionInCoordinateSystem(inletCsys)
            .getComponentFunction(componentDirection));
    return vCFF;
  }
  private IsoPart getReversedFlowPart(Collection<Region> myRegs){
    VectorComponentFieldFunction vCFF = getCaseInletVelocityComponent(isCrossWindMRFCase, 0);
    // Reversed Flow IsoSurface Part
    IsoPart reversedFlowPart = DerivedPartTool.getSingleIsoSurfPart(simu, "Reversed Flow", myRegs, vCFF, -1.0);
    return reversedFlowPart;
  }
  private ArrayList<StreamPart> getM600MainWingStreamLines(Collection<Region> myRegs){
    // M600 specific streamline parts
    ArrayList<StreamPart> retParts = new ArrayList();
    Units mUnit = ((Units) simu.getUnitsManager().getObject("m"));
    StreamPart thisSL;
    PrimitiveFieldFunction pFF = getCaseVelocityReferenceFramePrimitiveFF(isCrossWindMRFCase);
    Region kiteRegion = simu.getRegionManager().getRegion("Kite");

    // Port Tip
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Port Tip");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0, -12.3, -0.75}, new double[] {2.0, -12.3, 0.4}, 5);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    // Stbd Tip
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Stbd Tip");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0,  12.3, -0.75}, new double[] {2.0,  12.3, 0.4}, 5);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    // Main Wing
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Main Wing");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {3.0, -13.0, 0.0}, new double[] {3.0, 13.0, 0.0}, 100);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    // Pylon 1 Horizontal Streamlines
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Pylon 1 Horizontal");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0, -3.0, 0.1}, new double[] {2.0, -3.75, 0.1}, 10);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    //Pylon 2 Horizontal Streamlines
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Pylon 2 Horizontal");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0, -2.75, 0.1}, new double[] {2.0, -2.0, 0.1}, 10);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    //Pylon 3 Horizontal Streamlines
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Pylon 3 Horizontal");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0, 3.0, 0.1}, new double[] {2.0, 3.75, 0.1}, 10);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    //Pylon 4 Horizontal Streamlines
    thisSL = DerivedPartTool.getStreamlinePart(simu, "SL - Pylon 4 Horizontal");
    DerivedPartTool.setStreamlineLineSeed(thisSL, mUnit, new double[] {2.0, -2.75, 0.1}, new double[] {2.0, -2.0, 0.1}, 10);
    thisSL.getSecondOrderIntegrator().setDirection(IntegrationDirection.BOTH);
    thisSL.getInputParts().setObjects(kiteRegion);
    thisSL.setFieldFunction(pFF);
    retParts.add(thisSL);

    return retParts;
  }
  private PlaneSection getPlane(String planeName){
      return ((PlaneSection) simu.getPartManager().getObject(planeName));
  }
  private IsoPart getIso(String planeName){
      return ((IsoPart) simu.getPartManager().getObject(planeName));
  }
  private PlaneSection multiPlane(Collection<Region> myRegs, String planeName, CoordinateSystem myCsys,double[] newOrigin, double[] newNormal, int nSec, double minVal,double maxVal){
      PlaneSection myPlane;
      try{
          myPlane=(PlaneSection) simu.getPartManager().getObject(planeName);
      }catch(NeoException e){
          myPlane=
            (PlaneSection) simu.getPartManager().createImplicitPart(new NeoObjectVector(new Object[] {}), new DoubleVector(new double[] {0.0, 0.0, 1.0}), new DoubleVector(new double[] {0.0, 0.0, 0.0}), 0, 1, new DoubleVector(new double[] {0.0}));
      }
      myPlane.setCoordinateSystem(myCsys);

      Coordinate myOrientation = 
        myPlane.getOrientationCoordinate();
      Units units_0 = 
        ((Units) simu.getUnitsManager().getObject("m"));
      myOrientation.setCoordinate(units_0, units_0, units_0, new DoubleVector(newNormal));
      Coordinate myOrigin =myPlane.getOriginCoordinate();
      myOrigin.setCoordinate(units_0, units_0, units_0, new DoubleVector(newOrigin));
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

  //
  //=============================================
  // Reports Methods
  //=============================================
  // Kite object
  private void makeKiteReports(Region kiteRegion, Collection<Region> allRadReg,boolean wantMon){
        ForceCoefficientReport aeroX;
        ForceCoefficientReport aeroY;
        ForceCoefficientReport aeroZ;
        ForceCoefficientReport bodyX;
        ForceCoefficientReport bodyY;
        ForceCoefficientReport bodyZ;
        //=============================
        // TOTAL FORCES ON KITE BODY
        //=============================
        aeroX=makeForceCoef("Aero Kite Cd",inletCsys,xOnlyAxis, allWallParts(kiteRegion));
        aeroY=makeForceCoef("Aero Kite Cy",inletCsys,yOnlyAxis, allWallParts(kiteRegion));
        aeroZ=makeForceCoef("Aero Kite Cl",inletCsys,zOnlyAxis, allWallParts(kiteRegion));
        bodyX=makeForceCoef("Body Kite CX", bodyCsys,xOnlyAxis, allWallParts(kiteRegion));
        bodyY=makeForceCoef("Body Kite CY", bodyCsys,yOnlyAxis, allWallParts(kiteRegion));
        bodyZ=makeForceCoef("Body Kite CZ", bodyCsys,zOnlyAxis, allWallParts(kiteRegion));
        
        if(wantMon){
            Monitor aeroXMon = MonitorTool.reportIterationMonitor(simu, aeroX, maxSteadySteps, 1, 1);
            Monitor aeroYMon = MonitorTool.reportIterationMonitor(simu, aeroY, maxSteadySteps, 1, 1);
            Monitor aeroZMon = MonitorTool.reportIterationMonitor(simu, aeroZ, maxSteadySteps, 1, 1);
            Monitor bodyXMon = MonitorTool.reportIterationMonitor(simu, bodyX, maxSteadySteps, 1, 1);
            Monitor bodyYMon = MonitorTool.reportIterationMonitor(simu, bodyY, maxSteadySteps, 1, 1);
            Monitor bodyZMon = MonitorTool.reportIterationMonitor(simu, bodyZ, maxSteadySteps, 1, 1);
            MonitorTool.getMonitorGroup(simu, "Kite No Radiator Data").getGroupsManager()
              .groupObjects("Kite No Radiator Data",
                new NeoObjectVector(new Object[] {aeroXMon, aeroYMon, aeroZMon, bodyXMon, bodyYMon, bodyZMon}), true);
        }
        // Make Radiator Contributions
        SumReport tmpAeroRadX; SumReport tmpAeroRadY; SumReport tmpAeroRadZ;
        SumReport tmpBodyRadX; SumReport tmpBodyRadY; SumReport tmpBodyRadZ;
        ArrayList<Object> tmpAllRadReports=new ArrayList();
        // Make Radiator Contributions
        ArrayList<SumReport> aeroRadX=new ArrayList();
        ArrayList<SumReport> aeroRadY=new ArrayList();
        ArrayList<SumReport> aeroRadZ=new ArrayList();
        ArrayList<SumReport> bodyRadX=new ArrayList();
        ArrayList<SumReport> bodyRadY=new ArrayList();
        ArrayList<SumReport> bodyRadZ=new ArrayList();
        ArrayList<SumReport> allRadReports=new ArrayList();
        //Performance
        ExpressionReport tmpRadPerfReport;
        //
        for(Region tmp:allRadReg){
            tmpAeroRadX=makeRadiatorIntForceReport(tmp,inletCsys, 0,true); // Aero drag
            tmpAeroRadY=makeRadiatorIntForceReport(tmp,inletCsys, 1,true); // Aero lateral
            tmpAeroRadZ=makeRadiatorIntForceReport(tmp,inletCsys, 2,true); // Aero lift
            tmpBodyRadX=makeRadiatorIntForceReport(tmp,bodyCsys, 0,true); // Body drag
            tmpBodyRadY=makeRadiatorIntForceReport(tmp,bodyCsys, 1,true); // Body lateral
            tmpBodyRadZ=makeRadiatorIntForceReport(tmp,bodyCsys, 2,true); // Body lift
            
            tmpRadPerfReport=makeRadiatorPerformanceReport(tmp,true);
            //
            aeroRadX.add(tmpAeroRadX);aeroRadY.add(tmpAeroRadY);aeroRadZ.add(tmpAeroRadZ);
            bodyRadX.add(tmpBodyRadX);bodyRadY.add(tmpBodyRadY);bodyRadZ.add(tmpBodyRadZ);
            //
            tmpAllRadReports.add(tmpAeroRadX);tmpAllRadReports.add(tmpAeroRadY);tmpAllRadReports.add(tmpAeroRadZ);
            tmpAllRadReports.add(tmpBodyRadX);tmpAllRadReports.add(tmpBodyRadY);tmpAllRadReports.add(tmpBodyRadZ);
            ReportTool.getReportGroup(simu, tmp.getPresentationName()+" Data").getGroupsManager()
                    .groupObjects(tmp.getPresentationName()+" Data", new NeoObjectVector(tmpAllRadReports.toArray()), true);
            ReportTool.getReportGroup(simu, tmp.getPresentationName()+" Data").getGroupsManager()
                    .groupObjects(tmp.getPresentationName()+" Data", new NeoObjectVector(new Object[] {tmpRadPerfReport}), true);

            tmpAllRadReports.clear();
        }
        // Make total expression reports for aero
        ExpressionReport aeroTotalDrag = makeTotalForceCoefficient("Aero Kite Total Cd",aeroX,aeroRadX);
        ExpressionReport aeroTotalLatl = makeTotalForceCoefficient("Aero Kite Total Cy",aeroY,aeroRadY);
        ExpressionReport aeroTotalLift = makeTotalForceCoefficient("Aero Kite Total Cl",aeroZ,aeroRadZ);
        // Make total expression reports for body
        ExpressionReport bodyTotalCX = makeTotalForceCoefficient("Body Kite Total CX", bodyX, bodyRadX);
        ExpressionReport bodyTotalCY = makeTotalForceCoefficient("Body Kite Total CY", bodyY, bodyRadY);
        ExpressionReport bodyTotalCZ = makeTotalForceCoefficient("Body Kite Total CZ", bodyZ, bodyRadZ);        

        if(wantMon){
            Monitor aeroXMon=MonitorTool.reportIterationMonitor(simu,aeroTotalDrag,maxSteadySteps,1,1);
            Monitor aeroYMon=MonitorTool.reportIterationMonitor(simu,aeroTotalLatl,maxSteadySteps,1,1);
            Monitor aeroZMon=MonitorTool.reportIterationMonitor(simu,aeroTotalLift,maxSteadySteps,1,1);
            Monitor bodyXMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCX,maxSteadySteps,1,1);
            Monitor bodyYMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCY,maxSteadySteps,1,1);
            Monitor bodyZMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCZ,maxSteadySteps,1,1);
            MonitorTool.getMonitorGroup(simu, "Kite Total Data").getGroupsManager().groupObjects("Kite Total Data",
                new NeoObjectVector(new Object[] {aeroXMon,aeroYMon,aeroZMon,bodyXMon,bodyYMon,bodyZMon}), true);
        }

        //==========================================
        // TOTAL MOMENT ON KITE BODY
        //==========================================
        MomentCoefficientReport bodyCmX;
        MomentCoefficientReport bodyCmY;
        MomentCoefficientReport bodyCmZ;
        bodyCmX=makeMomentCoef("Body Kite CmX",bodyCsys,xOnlyAxis,zeroOrigin,wingSpanNorm, allWallParts(kiteRegion));
        bodyCmY=makeMomentCoef("Body Kite CmY",bodyCsys,yOnlyAxis,zeroOrigin,nominalChord, allWallParts(kiteRegion));
        bodyCmZ=makeMomentCoef("Body Kite CmZ",bodyCsys,zOnlyAxis,zeroOrigin,wingSpanNorm, allWallParts(kiteRegion));
        //
        MomentCoefficientReport aeroCmX;
        MomentCoefficientReport aeroCmY;
        MomentCoefficientReport aeroCmZ;
        aeroCmX=makeMomentCoef("Aero Kite CmX",inletCsys,xOnlyAxis,zeroOrigin,wingSpanNorm, allWallParts(kiteRegion));
        aeroCmY=makeMomentCoef("Aero Kite CmY",inletCsys,yOnlyAxis,zeroOrigin,nominalChord, allWallParts(kiteRegion));
        aeroCmZ=makeMomentCoef("Aero Kite CmZ",inletCsys,zOnlyAxis,zeroOrigin,wingSpanNorm, allWallParts(kiteRegion));
        //
        
        if(wantMon){
            Monitor aeroCXMon=MonitorTool.reportIterationMonitor(simu,bodyCmX,maxSteadySteps,1,1);
            Monitor aeroCYMon=MonitorTool.reportIterationMonitor(simu,bodyCmY,maxSteadySteps,1,1);
            Monitor aeroCZMon=MonitorTool.reportIterationMonitor(simu,bodyCmZ,maxSteadySteps,1,1);
            Monitor bodyCXMon=MonitorTool.reportIterationMonitor(simu,aeroCmX,maxSteadySteps,1,1);
            Monitor bodyCYMon=MonitorTool.reportIterationMonitor(simu,aeroCmY,maxSteadySteps,1,1);
            Monitor bodyCZMon=MonitorTool.reportIterationMonitor(simu,aeroCmZ,maxSteadySteps,1,1);
            MonitorTool.getMonitorGroup(simu, "Kite No Radiator Data").getGroupsManager()
              .groupObjects("Kite No Radiator Data",
                new NeoObjectVector(new Object[] {aeroCXMon,aeroCYMon,aeroCZMon,
                    bodyCXMon,bodyCYMon,bodyCZMon}), true);
        }
        
        
        // Make Radiator Contributions
        SumReport tmpBodyRadCmX;
        SumReport tmpBodyRadCmY;
        SumReport tmpBodyRadCmZ;
        ArrayList<SumReport> bodyRadCmX=new ArrayList();
        ArrayList<SumReport> bodyRadCmY=new ArrayList();
        ArrayList<SumReport> bodyRadCmZ=new ArrayList();
        ArrayList<SumReport> tmpBodyAllRadCmReports=new ArrayList();
        SumReport tmpAeroRadCmX;
        SumReport tmpAeroRadCmY;
        SumReport tmpAeroRadCmZ;
        ArrayList<SumReport> aeroRadCmX=new ArrayList();
        ArrayList<SumReport> aeroRadCmY=new ArrayList();
        ArrayList<SumReport> aeroRadCmZ=new ArrayList();
        ArrayList<SumReport> tmpAeroAllRadCmReports=new ArrayList();
        for(Region tmp:allRadReg){
          tmpBodyRadCmX=makeRadiatorIntMomentReport(tmp,bodyCsys,zeroOrigin, 0); // Body CmX
          tmpBodyRadCmY=makeRadiatorIntMomentReport(tmp,bodyCsys,zeroOrigin, 1); // Body CmY
          tmpBodyRadCmZ=makeRadiatorIntMomentReport(tmp,bodyCsys,zeroOrigin, 2); // Body CmZ
          bodyRadCmX.add(tmpBodyRadCmX);bodyRadCmY.add(tmpBodyRadCmY);bodyRadCmZ.add(tmpBodyRadCmZ);
          tmpBodyAllRadCmReports.add(tmpBodyRadCmX);tmpBodyAllRadCmReports.add(tmpBodyRadCmY);tmpBodyAllRadCmReports.add(tmpBodyRadCmZ);
          ReportTool.groupReports(simu, tmp.getPresentationName()+" Data",
            new NeoObjectVector(tmpBodyAllRadCmReports.toArray()));
          tmpBodyAllRadCmReports.clear();
          //
          tmpAeroRadCmX=makeRadiatorIntMomentReport(tmp,inletCsys,zeroOrigin, 0); // Aero CmX
          tmpAeroRadCmY=makeRadiatorIntMomentReport(tmp,inletCsys,zeroOrigin, 1); // Aero CmY
          tmpAeroRadCmZ=makeRadiatorIntMomentReport(tmp,inletCsys,zeroOrigin, 2); // Aero CmZ
          aeroRadCmX.add(tmpAeroRadCmX);aeroRadCmY.add(tmpAeroRadCmY);aeroRadCmZ.add(tmpAeroRadCmZ);
          tmpAeroAllRadCmReports.add(tmpAeroRadCmX);tmpAeroAllRadCmReports.add(tmpAeroRadCmY);tmpAeroAllRadCmReports.add(tmpAeroRadCmZ);
          ReportTool.groupReports(simu, tmp.getPresentationName()+" Data",
            new NeoObjectVector(tmpAeroAllRadCmReports.toArray()));
          tmpAeroAllRadCmReports.clear();
          //
        }
        // Make total expression reports for body
        //makeTotalMomentCoefficient(String repName,ForceCoefficientReport momentCoef, ArrayList<SumReport> radCmRepts,double refRad)
        ExpressionReport bodyTotalCmX = makeTotalMomentCoefficient("Body Kite Total CmX", bodyCmX, bodyRadCmX,25.663);
        ExpressionReport bodyTotalCmY = makeTotalMomentCoefficient("Body Kite Total CmY", bodyCmY, bodyRadCmY, 1.283);
        ExpressionReport bodyTotalCmZ = makeTotalMomentCoefficient("Body Kite Total CmZ", bodyCmZ, bodyRadCmZ,25.663);  

        ExpressionReport aeroTotalCmX = makeTotalMomentCoefficient("Aero Kite Total CmX", aeroCmX, aeroRadCmX,25.663);
        ExpressionReport aeroTotalCmY = makeTotalMomentCoefficient("Aero Kite Total CmY", aeroCmY, aeroRadCmY, 1.283);
        ExpressionReport aeroTotalCmZ = makeTotalMomentCoefficient("Aero Kite Total CmZ", aeroCmZ, aeroRadCmZ,25.663);  

        if(wantMon){
            Monitor bodyCmXMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCmX,maxSteadySteps,1,1);
            Monitor bodyCmYMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCmY,maxSteadySteps,1,1);
            Monitor bodyCmZMon=MonitorTool.reportIterationMonitor(simu,bodyTotalCmZ,maxSteadySteps,1,1);
            ArrayList<Monitor> tmpList = new ArrayList(Arrays.asList(bodyCmXMon, bodyCmYMon, bodyCmZMon));
            MonitorTool.groupMonitors(simu, "Kite Total Data", tmpList);
            //
            Monitor aeroCmXMon=MonitorTool.reportIterationMonitor(simu,aeroTotalCmX,maxSteadySteps,1,1);
            Monitor aeroCmYMon=MonitorTool.reportIterationMonitor(simu,aeroTotalCmY,maxSteadySteps,1,1);
            Monitor aeroCmZMon=MonitorTool.reportIterationMonitor(simu,aeroTotalCmZ,maxSteadySteps,1,1);
            tmpList = new ArrayList(Arrays.asList(aeroCmXMon,aeroCmYMon,aeroCmZMon));
            MonitorTool.groupMonitors(simu, "Kite Total Data", tmpList);
            //
        }

    }
  private void makeRotorReports(Collection<Region> allRotorReg,boolean wantMon){
        ForceReport fX; ForceReport fY; ForceReport rotorThrust;
        MomentReport mX; MomentReport mY; MomentReport rotorTorque;
        for(Region tmp:allRotorReg){
            String prePend = tmp.getPresentationName();
            CoordinateSystem tmpCsys = bodyCsys.getLocalCoordinateSystemManager().getObject(tmp.getPresentationName());
            //FX
            fX=makeForceReport(prePend+" FX",tmpCsys,xOnlyAxis, allWallParts(tmp));
            //FY
            fY=makeForceReport(prePend+" FY",tmpCsys,yOnlyAxis, allWallParts(tmp));
            //FZ
            rotorThrust=makeForceReport(prePend+" Thrust",tmpCsys,zOnlyAxis, allWallParts(tmp));
            //MX
            mX=makeMomentReport(prePend+" MX",tmpCsys,xOnlyAxis,zeroOrigin, allWallParts(tmp));
            //MY
            mY=makeMomentReport(prePend+" MY",tmpCsys,yOnlyAxis,zeroOrigin, allWallParts(tmp));
            //MZ
            rotorTorque=makeMomentReport(prePend+" Torque",tmpCsys,zOnlyAxis,zeroOrigin, allWallParts(tmp));
            ReportTool.groupReports(simu, tmp.getPresentationName()+" Data",
              new NeoObjectVector(new Object[] {rotorThrust,rotorTorque}));
            
            if(wantMon){
                Monitor fXMon=MonitorTool.reportIterationMonitor(simu,fX,maxSteadySteps,1,1);
                Monitor fYMon=MonitorTool.reportIterationMonitor(simu,fY,maxSteadySteps,1,1);
                Monitor rotorThrustMon=MonitorTool.reportIterationMonitor(simu,rotorThrust,maxSteadySteps,1,1);
                Monitor mXMon=MonitorTool.reportIterationMonitor(simu,mX,maxSteadySteps,1,1);
                Monitor mYMon=MonitorTool.reportIterationMonitor(simu,mY,maxSteadySteps,1,1);
                Monitor rotorTorqueMon=MonitorTool.reportIterationMonitor(simu,rotorTorque,maxSteadySteps,1,1);
                MonitorTool.getMonitorGroup(simu, prePend+" Data").getGroupsManager().groupObjects(prePend+" Data",
                    new NeoObjectVector(new Object[] {fXMon,fYMon,rotorThrustMon,mXMon,mYMon,rotorTorqueMon}), true);
            }
        }
    }
  // Generic
  private void makePartCoefficientReports(Region kiteRegion,String prePend,String firstPart,int nParts,boolean wantMon){
    ForceCoefficientReport aeroX;
    ForceCoefficientReport aeroY;
    ForceCoefficientReport aeroZ;
    ForceCoefficientReport bodyX;
    ForceCoefficientReport bodyY;
    ForceCoefficientReport bodyZ;
    //=============================
    // TOTAL FORCES ON PART_i BODY
    //=============================
    Collection<GeometryPart> allRegPart = kiteRegion.getPartGroup().getObjects();
    GeometryPart regPart=(GeometryPart) allRegPart.toArray()[0];

    for(int i=1;i<=nParts;i++){
      ArrayList<PartSurface> surfnX0s = new ArrayList();
      for(PartSurface tmp:regPart.getPartSurfaces()){
        if(nParts>1){
          if(tmp.getPresentationName().startsWith(firstPart+i)||tmp.getPresentationName().contains("."+firstPart+i)){
              surfnX0s.add(tmp);
          }
        }else{
          if(tmp.getPresentationName().startsWith(firstPart)||tmp.getPresentationName().contains("."+firstPart)){
              surfnX0s.add(tmp);
          }
        }
      }
      String iStr = ""+i+" ";
      if(nParts==1){
          iStr ="";
      }
      aeroX=makeForceCoef("Aero "+prePend+iStr+"Cd",inletCsys,xOnlyAxis,surfnX0s);
      aeroY=makeForceCoef("Aero "+prePend+iStr+"Cy",inletCsys,yOnlyAxis,surfnX0s);
      aeroZ=makeForceCoef("Aero "+prePend+iStr+"Cl",inletCsys,zOnlyAxis,surfnX0s);
      bodyX=makeForceCoef("Body "+prePend+iStr+"CX", bodyCsys,xOnlyAxis,surfnX0s);
      bodyY=makeForceCoef("Body "+prePend+iStr+"CY", bodyCsys,yOnlyAxis,surfnX0s);
      bodyZ=makeForceCoef("Body "+prePend+iStr+"CZ", bodyCsys,zOnlyAxis,surfnX0s);
      ArrayList<Report> tmpRepList = new ArrayList(
        Arrays.asList(aeroX, aeroY, aeroZ, bodyX, bodyY, bodyZ));
      ReportTool.groupReports(simu, prePend+iStr+"Data", tmpRepList);


      if(wantMon){
        Monitor aeroXMon=MonitorTool.reportIterationMonitor(simu,aeroX,maxSteadySteps,1,1);
        Monitor aeroYMon=MonitorTool.reportIterationMonitor(simu,aeroY,maxSteadySteps,1,1);
        Monitor aeroZMon=MonitorTool.reportIterationMonitor(simu,aeroZ,maxSteadySteps,1,1);
        Monitor bodyXMon=MonitorTool.reportIterationMonitor(simu,bodyX,maxSteadySteps,1,1);
        Monitor bodyYMon=MonitorTool.reportIterationMonitor(simu,bodyY,maxSteadySteps,1,1);
        Monitor bodyZMon=MonitorTool.reportIterationMonitor(simu,bodyZ,maxSteadySteps,1,1);
        ArrayList<Monitor> tmpMonList = new ArrayList(
            Arrays.asList(aeroXMon,aeroYMon,aeroZMon,bodyXMon,bodyYMon,bodyZMon));
        MonitorTool.groupMonitors(simu, prePend+iStr+"Data", tmpMonList);

      }

    }
  }
  private void makePartMomentReports(Region kiteRegion, String prePend, String firstPart, 
          int nStart, int nParts, boolean wantMon){
    MomentReport aeroMX;
    MomentReport aeroMY;
    MomentReport aeroMZ;
    MomentReport bodyMX;
    MomentReport bodyMY;
    MomentReport bodyMZ;

    //=============================
    // TOTAL MOMENTS ON PART_i BODY
    //=============================
    Collection<GeometryPart> allRegPart = kiteRegion.getPartGroup().getObjects();
    GeometryPart regPart=(GeometryPart) allRegPart.toArray()[0];

    for(int i=1; i<= nParts; i++){
      ArrayList<PartSurface> surfnX0s = new ArrayList();
      for(PartSurface tmp:regPart.getPartSurfaces()){
        if(nParts > 1){
          if(tmp.getPresentationName().startsWith(firstPart+i) ||
                  tmp.getPresentationName().contains("."+firstPart+i)){
              surfnX0s.add(tmp);
          }
        }else{
          if(tmp.getPresentationName().startsWith(firstPart) ||
                  tmp.getPresentationName().contains("."+firstPart)){
              surfnX0s.add(tmp);
          }
        }
      }
      String iStr = ""+i+" ";
      if(nParts==1){
          iStr ="";
      }

      aeroMX=makeMomentReport("Aero "+prePend+iStr+"MX", inletCsys, xOnlyAxis, zeroOrigin, surfnX0s);
      aeroMY=makeMomentReport("Aero "+prePend+iStr+"MY", inletCsys, yOnlyAxis, zeroOrigin, surfnX0s);
      aeroMZ=makeMomentReport("Aero "+prePend+iStr+"MZ", inletCsys, zOnlyAxis, zeroOrigin, surfnX0s);
      bodyMX=makeMomentReport("Body "+prePend+iStr+"MX", bodyCsys, xOnlyAxis, zeroOrigin, surfnX0s);
      bodyMY=makeMomentReport("Body "+prePend+iStr+"MY", bodyCsys, yOnlyAxis, zeroOrigin, surfnX0s);
      bodyMZ=makeMomentReport("Body "+prePend+iStr+"MZ", bodyCsys, zOnlyAxis, zeroOrigin, surfnX0s);
      ArrayList<Report> tmpRepList = new ArrayList(
        Arrays.asList(aeroMX, aeroMY, aeroMZ, bodyMX, bodyMY, bodyMZ));
      ReportTool.groupReports(simu, prePend+iStr+"Data", tmpRepList);

      if(wantMon){
        Monitor aeroXMon=MonitorTool.reportIterationMonitor(simu,aeroMX,maxSteadySteps,1,1);
        Monitor aeroYMon=MonitorTool.reportIterationMonitor(simu,aeroMY,maxSteadySteps,1,1);
        Monitor aeroZMon=MonitorTool.reportIterationMonitor(simu,aeroMZ,maxSteadySteps,1,1);
        Monitor bodyXMon=MonitorTool.reportIterationMonitor(simu,bodyMX,maxSteadySteps,1,1);
        Monitor bodyYMon=MonitorTool.reportIterationMonitor(simu,bodyMY,maxSteadySteps,1,1);
        Monitor bodyZMon=MonitorTool.reportIterationMonitor(simu,bodyMZ,maxSteadySteps,1,1);
        ArrayList<Monitor> tmpMonList = new ArrayList(
            Arrays.asList(aeroXMon,aeroYMon,aeroZMon,bodyXMon,bodyYMon,bodyZMon));
        MonitorTool.groupMonitors(simu, prePend+iStr+"Data", tmpMonList);
     }
    }
  }
  private void makePartMomentCoefficientReports(Region kiteRegion,
          String prePend, String firstPart, 
          int nStart, int nParts, boolean wantMon){
    MomentCoefficientReport aeroCMX;
    MomentCoefficientReport aeroCMY;
    MomentCoefficientReport aeroCMZ;
    MomentCoefficientReport bodyCMX;
    MomentCoefficientReport bodyCMY;
    MomentCoefficientReport bodyCMZ;

    //=============================
    // TOTAL MOMENTS ON PART_i BODY
    //=============================
    Collection<GeometryPart> allRegPart = kiteRegion.getPartGroup().getObjects();
    GeometryPart regPart=(GeometryPart) allRegPart.toArray()[0];

    for(int i=1; i<= nParts; i++){
      ArrayList<PartSurface> surfnX0s = new ArrayList();
      for(PartSurface tmp:regPart.getPartSurfaces()){
        if(nParts > 1){
          if(tmp.getPresentationName().startsWith(firstPart+i) ||
                  tmp.getPresentationName().contains("."+firstPart+i)){
              surfnX0s.add(tmp);
          }
        }else{
          if(tmp.getPresentationName().startsWith(firstPart) ||
                  tmp.getPresentationName().contains("."+firstPart)){
              surfnX0s.add(tmp);
          }
        }
      }
      String iStr = ""+i+" ";
      if(nParts==1){
          iStr ="";
      }
      
      Collection<NamedObject> tmpColl = new ArrayList();
      for(PartSurface tmpSurf : surfnX0s){
        tmpColl.add(tmpSurf);
      }
      
      aeroCMX=makeMomentCoef("Aero "+prePend+iStr+"CmX", inletCsys, xOnlyAxis, zeroOrigin, wingSpanNorm, tmpColl);
      aeroCMY=makeMomentCoef("Aero "+prePend+iStr+"CmY", inletCsys, yOnlyAxis, zeroOrigin, nominalChord, tmpColl);
      aeroCMZ=makeMomentCoef("Aero "+prePend+iStr+"CmZ", inletCsys, zOnlyAxis, zeroOrigin, wingSpanNorm, tmpColl);

      bodyCMX=makeMomentCoef("Body "+prePend+iStr+"CmX", bodyCsys, xOnlyAxis, zeroOrigin, wingSpanNorm, tmpColl);
      bodyCMY=makeMomentCoef("Body "+prePend+iStr+"CmY", bodyCsys, yOnlyAxis, zeroOrigin, nominalChord, tmpColl);
      bodyCMZ=makeMomentCoef("Body "+prePend+iStr+"CmZ", bodyCsys, zOnlyAxis, zeroOrigin, wingSpanNorm, tmpColl);

      ArrayList<Report> tmpRepList = new ArrayList(
        Arrays.asList(aeroCMX, aeroCMY, aeroCMZ, bodyCMX, bodyCMY, bodyCMZ));
      ReportTool.groupReports(simu, prePend+iStr+"Data", tmpRepList);

      if(wantMon){
        Monitor aeroXMon=MonitorTool.reportIterationMonitor(simu,aeroCMX,maxSteadySteps,1,1);
        Monitor aeroYMon=MonitorTool.reportIterationMonitor(simu,aeroCMY,maxSteadySteps,1,1);
        Monitor aeroZMon=MonitorTool.reportIterationMonitor(simu,aeroCMZ,maxSteadySteps,1,1);
        Monitor bodyXMon=MonitorTool.reportIterationMonitor(simu,bodyCMX,maxSteadySteps,1,1);
        Monitor bodyYMon=MonitorTool.reportIterationMonitor(simu,bodyCMY,maxSteadySteps,1,1);
        Monitor bodyZMon=MonitorTool.reportIterationMonitor(simu,bodyCMZ,maxSteadySteps,1,1);
        ArrayList<Monitor> tmpMonList = new ArrayList(
            Arrays.asList(aeroXMon,aeroYMon,aeroZMon,bodyXMon,bodyYMon,bodyZMon));
        MonitorTool.groupMonitors(simu, prePend+iStr+"Data", tmpMonList);
     }
    }
  }
  private void makePartLocalMomentReport(Region kiteRegion,String prePend,String firstPart,int nStart,int nParts,CoordinateSystem repCsys,double[] axisVector,double[] localOrigin,boolean wantMon){
    MomentReport localMomX;
    //=============================
    // TOTAL FORCES ON PART_i BODY
    //=============================
    Collection<GeometryPart> allRegPart = kiteRegion.getPartGroup().getObjects();
    GeometryPart regPart=(GeometryPart) allRegPart.toArray()[0];

    for(int i=nStart;i<=(nStart+nParts-1);i++){
      ArrayList<PartSurface> surfnX0s = new ArrayList();
      for(PartSurface tmp:regPart.getPartSurfaces()){
        if(nParts>1){
          if(tmp.getPresentationName().startsWith(firstPart+i)||tmp.getPresentationName().contains("."+firstPart+i)){
            surfnX0s.add(tmp);
          }
        }else{
          if(tmp.getPresentationName().startsWith(firstPart)||tmp.getPresentationName().contains("."+firstPart)){
            surfnX0s.add(tmp);
          }
        }
      }
      String iStr = ""+i+" ";
      if(nParts==1){
        iStr ="";
      }
      localMomX=makeMomentReport("Local "+prePend+iStr+"Moment", repCsys,axisVector,localOrigin,surfnX0s);
      ArrayList<Report> localRep = new ArrayList();
      localRep.add(localMomX);
      ReportTool.groupReports(simu, prePend+iStr+"Data", localRep );

      if(wantMon){
        Monitor localMomXMon=MonitorTool.reportIterationMonitor(simu,localMomX,maxSteadySteps,1,1);
        ArrayList<Monitor> localMon = new ArrayList();
        localMon.add(localMomXMon);
        MonitorTool.groupMonitors(simu, prePend+iStr+"Data", localMon);
      }
    }
  }
  private ForceCoefficientReport makeForceCoef(String repName, CoordinateSystem repCsys,double[] repDirection,Collection<? extends NamedObject> repParts){
        ForceCoefficientReport fRepC;
        try{
            fRepC = (ForceCoefficientReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            fRepC = simu.getReportManager().createReport(ForceCoefficientReport.class); 
            fRepC.setPresentationName(repName);
        }
        fRepC.setCoordinateSystem(repCsys);
        fRepC.getDirection().setComponents(repDirection[0], repDirection[1],repDirection[2]);
        fRepC.getReferenceDensity().setDefinition("${Case Density}");
        fRepC.getReferenceVelocity().setDefinition("${Case Airspeed}");
        fRepC.getReferenceArea().setDefinition("${Reference Area}");
        fRepC.setRepresentation(simProxy);
        //fRepC.setObjects(repParts);
        fRepC.getParts().setObjects(repParts);
        return fRepC;
    }
  private MomentCoefficientReport makeMomentCoef(String repName, CoordinateSystem repCsys,double[] repAxis, double[] repOrigin,double refRadius,Collection<NamedObject> repParts){
        MomentCoefficientReport mRepC;
        try{
            mRepC = (MomentCoefficientReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            mRepC = simu.getReportManager().createReport(MomentCoefficientReport.class); 
            mRepC.setPresentationName(repName);
        }
        mRepC.getReferenceRadius().setValue(2.0);
        mRepC.getReferenceArea().setDefinition("${Reference Area}");
        mRepC.getReferenceVelocity().setDefinition("${Case Airspeed}");
        mRepC.getReferenceDensity().setDefinition("${Case Density}");
        mRepC.getDirection().setComponents(repAxis[0], repAxis[1], repAxis[2]);
        mRepC.getOrigin().setComponents(repOrigin[0], repOrigin[1], repOrigin[2]);
        mRepC.getReferenceRadius().setValue(refRadius);
        mRepC.setCoordinateSystem(repCsys);
        mRepC.setRepresentation(simProxy);
        mRepC.setObjects(repParts);
        return mRepC;
    }
  private ForceReport makeForceReport(String repName, CoordinateSystem repCsys, double[] repDirection,Collection<NamedObject> repParts){
        ForceReport fRepC;
        try{
            fRepC = (ForceReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            fRepC = simu.getReportManager().createReport(ForceReport.class); 
            fRepC.setPresentationName(repName);
        }
        fRepC.setCoordinateSystem(repCsys);
        fRepC.getDirection().setComponents(repDirection[0], repDirection[1],repDirection[2]);
        fRepC.setRepresentation(simProxy);
        fRepC.setObjects(repParts);
        return fRepC;
    }
  private MomentReport makeMomentReport(String repName, CoordinateSystem repCsys,double[] repAxis, double[] repOrigin,Collection<? extends NamedObject> repParts){
        MomentReport mRepC;
        try{
            mRepC = (MomentReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            mRepC = simu.getReportManager().createReport(MomentReport.class); 
            mRepC.setPresentationName(repName);
        }
        mRepC.getDirection().setComponents(repAxis[0], repAxis[1], repAxis[2]);
        mRepC.getOrigin().setComponents(repOrigin[0], repOrigin[1], repOrigin[2]);
        mRepC.setCoordinateSystem(repCsys);
        mRepC.setRepresentation(simProxy);
        //mRepC.setObjects(repParts);
        mRepC.getParts().setObjects(repParts);
        return mRepC;
    }
  private SurfaceIntegralReport makeSurfIntReport(String repName, FieldFunction tmpFF, Collection<? extends Boundary> tmpBnd){
        SurfaceIntegralReport retRep;
        try{
            retRep = (SurfaceIntegralReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            retRep = simu.getReportManager().createReport(SurfaceIntegralReport.class); 
            retRep.setPresentationName(repName);
        }
        retRep.setRepresentation(simProxy);
        retRep.setScalar(tmpFF);
        retRep.getParts().setObjects(tmpBnd);
        return retRep;
    }
  //
  private ExpressionReport makeTotalForceCoefficient(String repName,ForceCoefficientReport forceCoef, ArrayList<SumReport> radRepts){
        ExpressionReport expRep;
        
        try{
            expRep=(ExpressionReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            expRep=(ExpressionReport) simu.getReportManager().createReport(ExpressionReport.class);
            expRep.setPresentationName(repName);
        }
        
        String tmpRad="(";
        for(SumReport tmp:radRepts){
            if(tmpRad.equals("(")){
                tmpRad=tmpRad+"$"+tmp.getPresentationName().replaceAll("\\s","")+"Report";
            }else{
                tmpRad=tmpRad+"+$"+tmp.getPresentationName().replaceAll("\\s","")+"Report";
            }
        }
        tmpRad=tmpRad+")";
        String tmpFCoefName=forceCoef.getPresentationName().replaceAll("\\s","")+"Report";
        expRep.setDefinition("$"+tmpFCoefName+"+"+tmpRad+
          "*2.0/(${Case Density}*${Case Airspeed}*${Case Airspeed}*${Reference Area})");
        return expRep;
    }
  private ExpressionReport makeTotalMomentCoefficient(String repName,MomentCoefficientReport momentCoef, ArrayList<SumReport> radCmRepts,double refRad){
        ExpressionReport expRep;
        try{
            expRep=(ExpressionReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            expRep=(ExpressionReport) simu.getReportManager().createReport(ExpressionReport.class);
            expRep.setPresentationName(repName);
        }
        
        String tmpRad="(";
        for(SumReport tmp:radCmRepts){
            if(tmpRad.equals("(")){
                tmpRad=tmpRad+"$"+tmp.getPresentationName().replaceAll("\\s","")+"Report";
            }else{
                tmpRad=tmpRad+"+$"+tmp.getPresentationName().replaceAll("\\s","")+"Report";
            }
        }
        tmpRad=tmpRad+")";
        String tmpCmCoefName=momentCoef.getPresentationName().replaceAll("\\s","")+"Report";
        expRep.setDefinition("$"+tmpCmCoefName+"+"+tmpRad+"*2.0/(${Case Density}*${Case Airspeed}*${Case Airspeed}*${Reference Area}*"+refRad+")");
        return expRep;
    }
  //
  private SumReport makeRadiatorIntForceReport(Region radRegion,CoordinateSystem repCsys,int repDir,boolean needMon){
        SumReport sRep;
        String repName = radRegion.getPresentationName();
        String prePend;
        String appEnd="";
        switch (repCsys.getPresentationName()) {
            case "Velocity Inlet":
                prePend="Aero ";
                if(repDir==0){
                    appEnd = " Fd";
                }else if(repDir==1){
                    appEnd = " Fy";
                }else if(repDir==2){
                    appEnd = " Fl";
                }else{
                    appEnd = " ERR";
                }    break;
            case "Body Csys":
                prePend="Body ";
                if(repDir==0){
                    appEnd = " FX";
                }else if(repDir==1){
                    appEnd = " FY";
                }else if(repDir==2){
                    appEnd = " FZ";
                }else{
                    appEnd = " ERR";
                }    break;
            default:
                prePend = "UKWN";
                break;
        }
        try{
            sRep = (SumReport) simu.getReportManager().getObject(prePend+repName+appEnd);
        }catch(NeoException e){
          sRep = simu.getReportManager().createReport(SumReport.class);
          sRep.setPresentationName(prePend+repName+appEnd);
        }
        sRep.setRepresentation(simProxy);
        UserFieldFunction uFF ;
        
        try{
            //force vector in desired coordinate system
            uFF=((UserFieldFunction) simu.getFieldFunctionManager().getObject("Radiator Force"));
        }catch(NeoException e){
            uFF=((UserFieldFunction) simu.getFieldFunctionManager().createFieldFunction());
            uFF.setPresentationName("Radiator Force");
        }
        uFF.setFunctionName("radiator_force");
        uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
        uFF.setDefinition("-($Pressure*$$Area+alternateValue($Density,1.17)*$$Velocity*dot($$Velocity,$$Area)) ");
        VectorComponentFieldFunction vFF = 
          ((VectorComponentFieldFunction) uFF.getFunctionInCoordinateSystem(repCsys).getComponentFunction(repDir));
        sRep.setFieldFunction(vFF);
        ArrayList<Boundary> radIntParts = new ArrayList();
        for(Boundary tmp:radRegion.getBoundaryManager().getBoundaries()){
            if(tmp instanceof InterfaceBoundary){
                radIntParts.add(tmp);
            }
        }
        sRep.getParts().setObjects(radIntParts);
        
        if(needMon&&appEnd.contains("Fd")){
            Monitor radFx=MonitorTool.reportIterationMonitor(simu,sRep,maxSteadySteps,1,1);
            ArrayList<Monitor> radFxList = new ArrayList();
            radFxList.add(radFx);
            MonitorTool.groupMonitors(simu, radRegion.getPresentationName()+" Data", radFxList);
        }
        
        
        return sRep;
    }
  private SumReport makeRadiatorIntMomentReport(Region radRegion,CoordinateSystem repCsys,double[] repOrigin,int repDir){
        SumReport sRep;
        String cSysName=repCsys.getPresentationName();
        String repName = radRegion.getPresentationName();
        String prePend;
        String appEnd="";
        switch (repCsys.getPresentationName()) {
            case "Velocity Inlet":
                prePend="Aero ";
                if(repDir==0){
                    appEnd = " CmX";
                }else if(repDir==1){
                    appEnd = " CmY";
                }else if(repDir==2){
                    appEnd = " CmZ";
                }else{
                    appEnd = " ERR";
                }    break;
            case "Body Csys":
                prePend="Body ";
                if(repDir==0){
                    appEnd = " CmX";
                }else if(repDir==1){
                    appEnd = " CmY";
                }else if(repDir==2){
                    appEnd = " CmZ";
                }else{
                    appEnd = " ERR";
                }    break;
            default:
                prePend = "UKWN ";
                break;
        }
        try{
            sRep = (SumReport) simu.getReportManager().getObject(prePend+repName+appEnd);
        }catch(NeoException e){
          sRep = simu.getReportManager().createReport(SumReport.class);
          sRep.setPresentationName(prePend+repName+appEnd);
        }
        sRep.setRepresentation(simProxy);

        // Calculate Moment Vector
        UserFieldFunction uFF ;
        try{
            //force vector in desired coordinate system
            uFF=((UserFieldFunction) simu.getFieldFunctionManager().getObject("Radiator Moment"));
        }catch(NeoException e){
            uFF=((UserFieldFunction) simu.getFieldFunctionManager().createFieldFunction());
            uFF.setPresentationName("Radiator Moment");
        }
        String tmpOrigin = "["+repOrigin[0]+","+repOrigin[1]+","+repOrigin[2]+"]";
        
        uFF.setFunctionName("radiator_moment");
        uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
        uFF.setDefinition("cross($$Position-"+tmpOrigin+",$$radiator_force)");
        VectorComponentFieldFunction vFF = 
          ((VectorComponentFieldFunction) uFF.getFunctionInCoordinateSystem(repCsys).getComponentFunction(repDir));
        sRep.setFieldFunction(vFF);
        ArrayList<Boundary> radIntParts = new ArrayList();
        for(Boundary tmp:radRegion.getBoundaryManager().getBoundaries()){
            if(tmp instanceof InterfaceBoundary){
                radIntParts.add(tmp);
            }
        }
        sRep.getParts().setObjects(radIntParts);
        return sRep;
    }
  private ExpressionReport makeRadiatorPerformanceReport(Region radRegion,boolean wantMon){

        String repName = radRegion.getPresentationName()+" Performance";
        String prePend;
        String appEnd="";

        ExpressionReport retRep;
        try{
            retRep = (ExpressionReport) simu.getReportManager().getObject(repName);
        }catch(NeoException e){
            retRep = simu.getReportManager().createReport(ExpressionReport.class); 
            retRep.setPresentationName(repName);
        }
        
        //Velocity field function
        PrimitiveFieldFunction pFF = 
          ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
        VectorMagnitudeFieldFunction vFF = 
          ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
        
        ArrayList<Boundary> radIntParts = new ArrayList();
        for(Boundary tmp:radRegion.getBoundaryManager().getBoundaries()){
            if(tmp instanceof InterfaceBoundary 
                    && ( tmp.getPresentationName().startsWith("71")
                      || tmp.getPresentationName().contains(".71") ) ){
                radIntParts.add(tmp);
//                radIntParts.add(tmp.getBoundary());
            }
        }
        //get surface integral report
        String tmpName = radRegion.getPresentationName()+" V In";
        SurfaceIntegralReport tmpRep = makeSurfIntReport(tmpName,vFF,radIntParts);
        ArrayList<Report> tmpList = new ArrayList();
        tmpList.add(tmpRep);
        ReportTool.groupReports(simu, radRegion.getPresentationName()+" Data", tmpList);
        String tmpVarName=tmpRep.getPresentationName().replaceAll("\\s","")+"Report";
        retRep.setDefinition("$"+tmpVarName+"/${Case Airspeed}");
        
        if(wantMon){
          Monitor localMonRep=MonitorTool.reportIterationMonitor(simu,retRep,maxSteadySteps,1,1);
          ArrayList<Monitor> tmpMonList = new ArrayList();
          tmpMonList.add(localMonRep);
          MonitorTool.groupMonitors(simu, radRegion.getPresentationName()+" Data", tmpMonList);
        }
      return retRep;
    }
  //
  // HPC Reports, Monitors, and Plots
  private void makeHPCReports(int itPerMemCheck){
    String hpcGroupName = "HPC Data";
    //
    CumulativeElapsedTimeReport wallTime;
    IteratorElapsedTimeReport itPerSec;
    //
    IterationMaximumMemoryReport resMem;
    IterationMaximumMemoryReport virtMem;
    IterationMaximumMemoryReport resWatermark;
    IterationMaximumMemoryReport virtWatermark;
    //
    Units memUnits = 
      ((Units) simu.getUnitsManager().getObject("GiB"));
    
    ArrayList<Report> tmpReps = new ArrayList();
    //Wall time
    try{
        wallTime=(CumulativeElapsedTimeReport) simu.getReportManager().getReport("Wall Time");
    }catch(NeoException e){
        wallTime = 
          simu.getReportManager().createReport(CumulativeElapsedTimeReport.class);
        wallTime.setPresentationName("Wall Time");
    }
    wallTime.setUnits((Units) simu.getUnitsManager().getObject("hr"));
    tmpReps.add(wallTime);
    //
    // Seconds per Iteration
    try{
        itPerSec = (IteratorElapsedTimeReport) simu.getReportManager().getReport("sec/it");
    }catch(NeoException e){
        itPerSec = 
          simu.getReportManager().createReport(IteratorElapsedTimeReport.class);
        itPerSec.setPresentationName("sec/it");
    }

    tmpReps.add(itPerSec);
    // Memory Management
    try{
        resMem =(IterationMaximumMemoryReport) simu.getReportManager().getReport("RES Memory");
    }catch(NeoException e){
        resMem = 
          simu.getReportManager().createReport(IterationMaximumMemoryReport.class);
        resMem.setPresentationName("RES Memory");
    }
    resMem.getMemoryReportMetricOption().setSelected(MemoryReportMetricOption.Type.RESIDENT);
    resMem.setUnits(memUnits);
    resMem.setSamplingFrequency(itPerMemCheck);
    tmpReps.add(resMem);
    //
    try{
        virtMem =(IterationMaximumMemoryReport) simu.getReportManager().getReport("VIRT Memory");
    }catch(NeoException e){
        virtMem = 
          simu.getReportManager().createReport(IterationMaximumMemoryReport.class);
        virtMem.setPresentationName("VIRT Memory");
    }
    virtMem.getMemoryReportMetricOption().setSelected(MemoryReportMetricOption.Type.VIRTUAL);
    virtMem.setUnits(memUnits);
    virtMem.setSamplingFrequency(itPerMemCheck);
    tmpReps.add(virtMem);
    //
    try{
        resWatermark =(IterationMaximumMemoryReport) simu.getReportManager().getReport("RES Watermark Memory");
    }catch(NeoException e){
        resWatermark = 
          simu.getReportManager().createReport(IterationMaximumMemoryReport.class);
        resWatermark.setPresentationName("RES Watermark Memory");
    }
    resWatermark.getMemoryReportMetricOption().setSelected(MemoryReportMetricOption.Type.RESIDENTHWM);
    resWatermark.setUnits(memUnits);
    resWatermark.setSamplingFrequency(itPerMemCheck);
    tmpReps.add(resWatermark);
    //
    try{
        virtWatermark =(IterationMaximumMemoryReport) simu.getReportManager().getReport("VIRT Watermark Memory");
    }catch(NeoException e){
        virtWatermark = 
          simu.getReportManager().createReport(IterationMaximumMemoryReport.class);
        virtWatermark.setPresentationName("VIRT Watermark Memory");
    }
    virtWatermark.getMemoryReportMetricOption().setSelected(MemoryReportMetricOption.Type.VIRTUALHWM);
    virtWatermark.setUnits(memUnits);
    virtWatermark.setSamplingFrequency(itPerMemCheck);
    tmpReps.add(virtWatermark);
    
    //Group all HPC Reports
    ReportTool.groupReports(simu, hpcGroupName, tmpReps);

    // Monitors
    ArrayList<Monitor> tmpMons = new ArrayList(); 
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, wallTime, maxSteadySteps, itPerMemCheck, 1));
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, itPerSec, maxSteadySteps, itPerMemCheck, 1));
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, resMem, maxSteadySteps, itPerMemCheck, 1));
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, virtMem, maxSteadySteps, itPerMemCheck, 1));
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, resWatermark, maxSteadySteps, itPerMemCheck, 1));
    tmpMons.add(MonitorTool.reportIterationMonitor(simu, virtWatermark, maxSteadySteps, itPerMemCheck, 1));
    for(Monitor tmpMon:tmpMons){
      ((ReportMonitor) tmpMon).setPlotLimit(itPerMemCheck*10);
    }
    MonitorTool.groupMonitors(simu, hpcGroupName, tmpMons);
    //Plots
    ArrayList<StarPlot> hpcPlots = new ArrayList();
    int counter=0;
    for(Monitor hpcMon:tmpMons){
      String hpcName = hpcMon.getPresentationName();
      MonitorPlot tmpPlot = PlotTool.getMonitorPlot(simu,hpcName,hpcName);
      MonitorDataSet tmpDat = PlotTool.addDataSet(simu, tmpPlot, hpcName, hpcName);
      PlotTool.setDataLines(tmpDat, hpcName, 2, PlotTool.forLoopColor(counter));
      hpcPlots.add(tmpPlot);
      counter++;
    }
    PlotTool.groupPlots(simu,hpcGroupName, hpcPlots);
  }
  private ArrayList<StarPlot> postProcessHPCData(Simulation simu, String hpcPlotPath, String hpcCSVPath){
    // method postProcessHPCData puts the HPC reports into the appropriate folders and returns the list of
    //   post processed plots that have been exported
    // 
    // System I/O check
    SystemTool.touchDirectory(hpcPlotPath);
    SystemTool.touchDirectory(hpcCSVPath);
    // Name of HPC Group
    String hpcGroupName = "HPC Data";
    // PLOTS
    ArrayList<StarPlot> retPlots = new ArrayList();
    for(StarPlot tmpPlot:PlotTool.getPlotGroup(simu, hpcGroupName).getObjectsOf(StarPlot.class)){
      String noSpaceName = tmpPlot.getPresentationName().replaceAll("\\s","");
      PlotTool.setPlotXAxis(tmpPlot, "Iteration", 0, maxSteadySteps, 1000, 2);
      // Export to Plot and CSV folder
      tmpPlot.export(hpcCSVPath + noSpaceName+".csv");
      tmpPlot.encode(hpcPlotPath + noSpaceName+".png","png", outputRes[0], outputRes[1],true);
      retPlots.add(tmpPlot);
    } 
    return retPlots;
  }

  //=============================================
  // Monitor Methods
  //=============================================
  private void groupResidualMonitors(){
    ArrayList<Monitor> retList=new ArrayList();
    for(Monitor tmp:simu.getMonitorManager().getMonitors()){
        if(tmp instanceof ResidualMonitor){
            retList.add(tmp);
        }
    }
    MonitorTool.groupMonitors(simu, "Residuals", retList);
  }
  private void setOutputMonitorDisplay(){
    ArrayList<String> wantMonitors= new ArrayList( Arrays.asList(
      "Continuity","X-momentum","Y-momentum","Z-momentum","Energy","Tke","Sdr"));
    //wantMonitors.add("Aero Kite Total Cl - It");
    //wantMonitors.add("Aero Kite Total Cd - It");
    //wantMonitors.add("Aero Kite Total Cy - It");
    //wantMonitors.add("Body Kite Total CmX - It");
    //wantMonitors.add("Body Kite Total CmY - It");
    //wantMonitors.add("Body Kite Total CmZ - It");
    wantMonitors.add("Body Kite CZ - It"); // Should be faster to track versus aero and total
    wantMonitors.add("Body Kite CX - It"); // Should be faster to track versus aero and total
    //wantMonitors.add("Wall Time - It");
    wantMonitors.add("sec/it - It");
    //wantMonitors.add("RES Memory - It");
    //wantMonitors.add("VIRT Memory - It");
    //wantMonitors.add("RES Watermark Memory - It");
    //wantMonitors.add("VIRT Watermark Memory - It");

    // this seems a bit inefficient
    ArrayList<Monitor> dispMonitors = new ArrayList();
    simu.getMonitorManager().setPrintedMonitors();
    for(String wantedName : wantMonitors){
      try{
        Monitor tmpMon = simu.getMonitorManager().getObject(wantedName);
        dispMonitors.add(tmpMon);
      }catch(NeoException e){
        simu.println("M600 setOutputMonitorDisplay(): "+wantedName+" not found.");
      }
    }
    simu.getMonitorManager().setPrintedMonitors(dispMonitors);
  }
  private void setupUnsteadyMonitors(){
        /* Loop through the monitor groupings,
           duplicate the monitors into unsteady monitors,
           place unsteady monitor into appropriate group.
        */
        simu.println("in setupUnsteadyMonitors");
        for(ClientServerObjectManager<ClientServerObject> tmpGroupList:simu.getMonitorManager().getGroupsManager().getObjects()){
            String groupName=tmpGroupList.getPresentationName();
            //Exception groups
            if(groupName.equals("HPC Data")||groupName.equals("Residuals")){
                    continue;
                }
            for(Monitor tmpMon:tmpGroupList.getObjectsOf(Monitor.class)){
                String itMonName=tmpMon.getPresentationName();
                if(    !(tmpMon instanceof SimulationIteratorTimeReportMonitor)
                    && !tmpMon.getPresentationName().endsWith("UNS")){

                    String unsName=itMonName.substring(0,itMonName.length()-2)+"UNS";
                    //does this already exist
                    ReportMonitor unsRepMon;
                    try{
                        unsRepMon = (ReportMonitor) simu.getMonitorManager().getObject(unsName);
                    }catch(NeoException e){
                        //simu.println("Creating Monitor:" + unsName);
                        unsRepMon = ((ReportMonitor) tmpMon).getReport().createMonitor();
                        unsRepMon.setPresentationName(unsName);
                    }
                    unsRepMon.copyProperties(tmpMon);
                    unsRepMon.getStarUpdate().getUpdateModeOption()
                        .setSelected(StarUpdateModeOption.Type.TIMESTEP);
                    unsRepMon.getStarUpdate().getTimeStepUpdateFrequency().setTimeSteps(1);

                    //add new unsteady Monitor into Group
                    MonitorTool.getMonitorGroup(simu, groupName).add(unsRepMon.getObjectKey().getObject());
                    
            //        getMonGroup("Kite No Radiator Data").getGroupsManager().groupObjects("Kite No Radiator Data",
            //    new NeoObjectVector(new Object[] {aeroXMon,aeroYMon,aeroZMon,bodyXMon,bodyYMon,bodyZMon}), true);
                    
                }else{
                    if(!tmpMon.getPresentationName().endsWith("UNS")){
                        simu.println("No Report Monitor: "+itMonName);
                    }
                }
            }
        }
        /*
        for(Monitor tmp:simu.getMonitorManager().getObjects()){
            if(tmp instanceof ReportMonitor && !(tmp instanceof SimulationIteratorTimeReportMonitor)){
                //simu.println("It monitor name: "+tmp);
                makeUnsteadyReportMonitor(tmp.getPresentationName());
                
            }
        }
        */
        //simu.println("Done");
    }

  //=============================================
  // Plots Methods
  //=============================================
  private void setupInitialPlots(){
        // Line Style Options:
        //
        //   None, Solid, Dot, Dash, Dash Dot, Dash Dot Dot
        //
        // Symbol Style Options:
        //
        //  None, Filled Square, Empty Square, Filled Circle, Empty Circle,
        //    Filled Triangle, Empty Triangle, Cross, Plus, Star, Filled Diamond,
        //    Horizontal Line, Vertical Line
        // Header
        MonitorPlot tmpPlot;
        String tmpName;
        MonitorDataSet tmpMonData;

        //Residual Plot
        adjustResidualPlot();
        //
        //Kite body only
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Only Cd","Aero Kite Only Cd");
          //
          tmpName="Aero Kite Cd - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Cd");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",0.05,0.15,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("blue"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("blue"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Only Cy","Aero Kite Only Cy");
          tmpName="Aero Kite Cy - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Cy");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.2,0.2,0.02,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("orange"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("orange"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Only Cl","Aero Kite Only Cl");
          tmpName="Aero Kite Cl - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Cl");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",1.0,2.0,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("darkgn"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("darkgn"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Only CX","Body Kite Only CX");
          tmpName="Body Kite CX - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite CX");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",0.05,0.15,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("blue"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("blue"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Only CY","Body Kite Only CY");
          tmpName="Body Kite CY - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite CY");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.2,0.2,0.02,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("orange"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("orange"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Only CZ","Body Kite Only CZ");
          tmpName="Body Kite CZ - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Cl");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",1.0,2.0,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("darkgn"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("darkgn"));
        //===================
        //  Kite TOTAL
        //===================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Total Cd","Aero Kite Total Cd");
          tmpName="Aero Kite Total Cd - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Total Cd");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",0.05,0.15,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("blue"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("blue"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Total Cy","Aero Kite Total Cy");
          tmpName="Aero Kite Total Cy - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Total Cy");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.2,0.2,0.02,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("orange"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("orange"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Aero Kite Total Cl","Aero Kite Total Cl");
          tmpName="Aero Kite Total Cl - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Aero Kite Total Cl");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",1.0,2.0,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("darkgn"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("darkgn"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CX","Body Kite Total CX");
          tmpName="Body Kite Total CX - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CX");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.18,0.0,0.02,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("blue"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("blue"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CY","Body Kite Total CY");
          tmpName="Body Kite Total CY - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CY");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.18,0.18,0.04,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("orange"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("orange"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CZ","Body Kite Total CZ");
          tmpName="Body Kite Total CZ - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CZ");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-2.0,-1.0,0.1,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("darkgn"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("darkgn"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CmX","Body Kite Total CmX");
          tmpName="Body Kite Total CmX - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CmX");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.01,0.01,0.01,4);
          PlotTool.setDataLines(tmpMonData,"Dash",2,PlotTool.getColor("blue"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("blue"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CmY","Body Kite Total CmY");
          tmpName="Body Kite Total CmY - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CmY");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-0.25,0.25,0.025,4);
          PlotTool.setDataLines(tmpMonData,"Dash",2,PlotTool.getColor("orange"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("orange"));
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Body Kite Total CmZ","Body Kite Total CmZ");
          tmpName="Body Kite Total CmZ - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Body Kite Total CmZ");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-.01,.01,0.01,4);
          PlotTool.setDataLines(tmpMonData,"Dash",2,PlotTool.getColor("darkgn"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("darkgn"));
        //===============================================================================
        //  HTAIL Control Surface Information
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Local H Tail Moment","Local H Tail Moment");
          tmpName="Local H Tail Moment - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Local H Tail Moment");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-100.0,0.0,10.0,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("teal"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("teal"));
        //===============================================================================
        //  VTAIL Control Surface Information
        //===============================================================================
        tmpPlot=PlotTool.getMonitorPlot(simu,"Local Rudder Moment","Local Rudder Moment");
          tmpName="Local Rudder Moment - It";
          PlotTool.addDataSet(simu,tmpPlot, tmpName,"Local Rudder Moment");
          tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(tmpName);
          PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,maxSteadySteps,1000.,4);
          PlotTool.setPlotYAxis(tmpPlot,"",-5.0,5.0,0.5,4);
          PlotTool.setDataLines(tmpMonData,"Solid",2,PlotTool.getColor("purple"));
          PlotTool.setDataSymbols(tmpMonData,"None",10,1,1,PlotTool.getColor("purple"));
    }
  private void postProcessPlots(String appEnd,boolean printPlotsNow){
    // postProcessingPlots both creates and prints out plots for post-processing
    //   String appEnd indicates whether a case is " - It" or " - UNS", i.e. steady or unsteady
    //   boolean printPlotsNow indicates whether to print out the plots or not
    // 
    MonitorDataSet tmpMonData;
    ArrayList<StarPlot> printedPlots = new ArrayList();
        
    //Residual Plot
    MonitorPlot tmpResPlot = adjustResidualPlot();
    String resPlotName = tmpResPlot.getPresentationName();
    if(printPlotsNow){
      try{
        // goes to both user and CFD folders
        tmpResPlot.encode(userPlots + resPlotName + ".png", "png", outputRes[0], outputRes[1],true);
        tmpResPlot.encode(cfdPlots + resPlotName + ".png", "png", outputRes[0], outputRes[1],true);
        printedPlots.add(tmpResPlot);
      }catch(NeoException e){
        simu.println("Unable to print out residual plots for some reason.");
      }
    }

    // HPC Data
    // Goes to CFD postprocessing area
    String hpcCSV = cfdCSV+File.separator+"HPC"+File.separator;
    String hpcPlots = cfdPlots+File.separator+"HPC"+File.separator;
    if(printPlotsNow){
      printedPlots.addAll(postProcessHPCData(simu,hpcCSV,hpcPlots));
    }

    // Kite post-processing
    String kiteCSV = userCSV+File.separator+"Kite"+File.separator;
    String kitePlots = userPlots+File.separator+"Kite"+File.separator;
    if(printPlotsNow){
      printedPlots.addAll(postProcessKitePlots(simu, kitePlots, kiteCSV, "Aero", appEnd));
      printedPlots.addAll(postProcessKitePlots(simu, kitePlots, kiteCSV, "Body", appEnd));
    }

    // All left-over Monitor plots (must be user-created; put into misc folder
    String miscCSV = userCSV+File.separator+"MISC"+File.separator;
    String miscPlots = userPlots+File.separator+"MISC"+File.separator;
    if(printPlotsNow){
      printedPlots.addAll(postProcessMiscMonitorPlots(simu, miscPlots, miscCSV, printedPlots));
    }
    // All left-over XY Plots (must be user-created; put into misc folder)
    miscCSV = userCSV+File.separator+"MISC"+File.separator;
    miscPlots = userPlots+File.separator+"MISC"+File.separator;
    if(printPlotsNow){
      printedPlots.addAll(postProcessMiscXYPlots(simu, miscPlots, miscCSV, printedPlots));
    }
    

  }
  //  
  // Plots - Residual
  private void setResidualsToAbsolute(){
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("Continuity"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("X-momentum"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("Y-momentum"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("Z-momentum"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    try{
      ((ResidualMonitor) simu.getMonitorManager().getMonitor("Energy"))
        .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    }catch(NeoException e){
      simu.println("Energy monitor not found.");
    }
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("Sdr"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
    ((ResidualMonitor) simu.getMonitorManager().getMonitor("Tke"))
      .getNormalizeOption().setSelected(MonitorNormalizeOption.Type.OFF);
  }
  private MonitorPlot adjustResidualPlot(){
    MonitorPlot tmpResPlot;
    String tmpName;
    MonitorDataSet tmpDataSet;
    //
    tmpResPlot = getResidualPlot("Residuals","Absolute Residuals");
    PlotTool.setPlotXAxis(tmpResPlot, "Iteration", 0.0, maxSteadySteps,1000.,3);
    PlotTool.setPlotXToLog(tmpResPlot, true);
    //
    tmpName = "Continuity";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("red"));
    PlotTool.setDataSymbols(tmpDataSet, "None", 2, 1, 1, PlotTool.getColor("red"));
    //
    tmpName = "X-momentum";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("blue"));
    PlotTool.setDataSymbols(tmpDataSet, "None", 2, 1, 1, PlotTool.getColor("blue"));
    //
    tmpName = "Y-momentum";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("orange"));
    PlotTool.setDataSymbols(tmpDataSet, "None", 2, 1, 1, PlotTool.getColor("orange"));
    //
    tmpName = "Z-momentum";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("darkgrn"));
    PlotTool.setDataSymbols(tmpDataSet, "None", 2, 1, 1, PlotTool.getColor("darkgrn"));
    //
    try{
      tmpName = "Energy";
      tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
      PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("drkred"));
      PlotTool.setDataSymbols(tmpDataSet, "None", 2, 1, 1, PlotTool.getColor("drkred"));  
    }catch(NeoException e){
      simu.println("Energy not found.");
    }
    
    //
    tmpName = "Sdr";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("cyan"));
    PlotTool.setDataSymbols(tmpDataSet, "Filled Square", 2, 1, 1, PlotTool.getColor("cyan"));
    //
    tmpName = "Tke";
    tmpDataSet = (MonitorDataSet) tmpResPlot.getDataSetManager().getDataSet(tmpName);
    PlotTool.setDataLines(tmpDataSet, "Solid", 2, PlotTool.getColor("magenta"));
    PlotTool.setDataSymbols(tmpDataSet, "Filled Square", 2, 1, 1, PlotTool.getColor("magenta"));
    //
    return tmpResPlot;
  }
  private ResidualPlot getResidualPlot(String plotName, String plotTitle){
    ResidualPlot retPlot;
    retPlot = (ResidualPlot) simu.getPlotManager().getObject(plotName);
    retPlot.setPresentationName(plotName);
    retPlot.setTitle(plotTitle);
    retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 0, 18));
    retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 1, 18));
    MultiColLegend tmpLegend = 
        retPlot.getLegend();
    tmpLegend.getChartPositionOption().setSelected(ChartPositionOption.Type.NORTH_EAST);
    return retPlot;
  }
  
  // Plots - Kite Body
  private ArrayList<StarPlot> postProcessKitePlots(Simulation simu, 
    String plotPath, String csvPath, String inFrame, String appEnd){
    // method postProcessHPCData puts the HPC reports into the appropriate folders and returns the list of
    //   post processed plots that have been exported
    //
    // String plotPath is the path for the plots
    // String csvPath is the path for the csv files from the monitors
    // String inFrame is either "Aero" or "Body"
    // String appEnd is indicating Iteration or Unsteady
    // 
    // System I/O check
    SystemTool.touchDirectory(plotPath);
    SystemTool.touchDirectory(csvPath);

    // PLOTS by Group List
    ArrayList<Monitor> kiteMonitors = new ArrayList();
    Collection<ClientServerObjectManager<ClientServerObject>> allMonGroupList 
      = simu.getMonitorManager().getGroupsManager().getObjects();
    for(ClientServerObjectManager<ClientServerObject> tmpMonGroup:allMonGroupList){
      for(Monitor grpMon:tmpMonGroup.getObjectsOf(Monitor.class)){
        String grpMonName = grpMon.getPresentationName();
        if(grpMonName.contains("Kite") && grpMonName.startsWith("Aero") && (grpMon instanceof ReportMonitor) ){
          kiteMonitors.add(grpMon);
        }
      }
    }

    ArrayList<StarPlot> kitePlots = new ArrayList();
    for(Monitor tmpMon:kiteMonitors){
      String monName = tmpMon.getPresentationName();
      double[] colorArray;
      //assign colors
      if(monName.contains("Cd")||monName.contains("CX")||monName.contains("CmX")){
        colorArray = PlotTool.getColor("blue");
      }else if(monName.contains("Cy")||monName.contains("CY")||monName.contains("CmY")){
        colorArray = PlotTool.getColor("orange");
      }else if(monName.contains("Cl")||monName.contains("CL")||monName.contains("CmZ")){
        colorArray = PlotTool.getColor("dkgrn");
      }else{
        colorArray = PlotTool.getColor("black");
      }

      // Create/get Plot
      MonitorPlot tmpPlot=PlotTool.getMonitorPlot(simu,monName,monName);
      String repName=((ReportMonitor) tmpMon).getReport().getPresentationName();
      String repMonName=repName+" - "+appEnd;
      
      // Set up plot
      PlotTool.addDataSet(simu, tmpPlot, repMonName,repName);
      MonitorDataSet tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(monName);
      double xAxisRange;
      //setup x-axis
      if(appEnd.equals("UNS")){
        if(getCurrentTime()<1e-3){
            xAxisRange=1.0;
        }else{
            xAxisRange=getCurrentTime();
        }
        PlotTool.setPlotXAxis(tmpPlot,"Time (s)",0.0,xAxisRange,nDataSamples*getTimeStep(),1);
      }else{
        if(simu.getSimulationIterator().getCurrentIteration()<10){
            xAxisRange=100;
            PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,xAxisRange,10,1);
        }else{
            xAxisRange=simu.getSimulationIterator().getCurrentIteration();
            PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,xAxisRange,nDataSamples,1);
        }
      }
      
      //setup y-axis
      double meanVal=MonitorTool.getLastMonitorSamplesMean(simu,monName,nDataSamples);
      double stDevVal=MonitorTool.getLastMonitorSamplesStDev(simu,monName,nDataSamples,meanVal);
      PlotTool.autoFitYAxisToYData(simu, tmpPlot, meanVal, stDevVal);

      //setup data lines
      String lineChoice="Solid";
      int lineThickness=2;
      String symbChoice="None";
      int symbThickness=10;
      if(monName.contains("CmX")||monName.contains("CmY")||monName.contains("CmZ")){
          lineChoice="Dash";
      }else if(monName.contains("Local")&&monName.contains("Moment")){
          lineChoice="Dash";
      }
      PlotTool.setDataLines(tmpMonData,lineChoice,lineThickness,colorArray);
      PlotTool.setDataSymbols(tmpMonData,symbChoice,symbThickness,2,2,colorArray);
      
      //Add Plot to Group
      kitePlots.add(tmpPlot);
            
    }
    //Group all these plots
    PlotTool.groupPlots(simu, inFrame+" Kite", kitePlots);
    
    return kitePlots;
  }
  
  // Plots - MISC User
  private ArrayList<StarPlot> postProcessMiscMonitorPlots(Simulation simu, String plotPath, String csvPath, 
                                           ArrayList<StarPlot> donePlots){
    // method postProcessMiscPlots take remaining plots not captured in previous post processing outputs
    //   and outputs them to the user misc folder
    //
    // String plotPath is the path for the plots
    // String csvPath is the path for the csv files from the monitors
    // ArrayList<StarPlot> donePlots is all previously post-process plots
    // 
    // System I/O check  
    SystemTool.touchDirectory(plotPath);
    SystemTool.touchDirectory(csvPath);

    //
    ArrayList<StarPlot> completedPlots = new ArrayList();
    
    //Post Process Remaining Plots
    Collection<StarPlot> allPlots = simu.getPlotManager().getObjects();
    allPlots.removeAll(donePlots);
    for(StarPlot tmpPlot:allPlots){
      if(!(tmpPlot instanceof MonitorPlot)){
        allPlots.remove(tmpPlot);
      }
    }

    simu.println("Post processing remaining misc monitor plots");
    for(StarPlot tmpPlot:allPlots){
      String monName = tmpPlot.getPresentationName();
      double[] colorArray;

      //assign colors
      if(monName.contains("Cd")||monName.contains("CX")||monName.contains("CmX")){
        colorArray = PlotTool.getColor("blue");
      }else if(monName.contains("Cy")||monName.contains("CY")||monName.contains("CmY")){
        colorArray = PlotTool.getColor("orange");
      }else if(monName.contains("Cl")||monName.contains("CL")||monName.contains("CmZ")){
        colorArray = PlotTool.getColor("darkgrn");
      }else{ //non aero monitor plots
        colorArray = PlotTool.getColor("blue");
      }

      try{
        MonitorDataSet tmpMonData=(MonitorDataSet) tmpPlot.getDataSetManager().getDataSet(monName);
        double xAxisRange;
        //setup x-axis
        if(monName.endsWith("UNS")){
          if(getCurrentTime()<1e-3){
              xAxisRange=1.0;
          }else{
              xAxisRange=getCurrentTime();
          }
          PlotTool.setPlotXAxis(tmpPlot,"Time (s)",0.0,xAxisRange,nDataSamples*getTimeStep(),1);
        }else{
          if(simu.getSimulationIterator().getCurrentIteration()<10){
              xAxisRange=100;
              PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,xAxisRange,10,1);
          }else{
              xAxisRange=simu.getSimulationIterator().getCurrentIteration();
              PlotTool.setPlotXAxis(tmpPlot,"Iteration",0.0,xAxisRange,nDataSamples,1);
          }
        }

        //setup y-axis
        double meanVal=MonitorTool.getLastMonitorSamplesMean(simu,monName,nDataSamples);
        double stDevVal=MonitorTool.getLastMonitorSamplesStDev(simu,monName,nDataSamples,meanVal);
        PlotTool.autoFitYAxisToYData(simu, tmpPlot, meanVal, stDevVal);

        //setup data lines
        String lineChoice="Solid";
        int lineThickness=2;
        String symbChoice="None";
        int symbThickness=10;
        if(monName.contains("CmX")||monName.contains("CmY")||monName.contains("CmZ")){
            lineChoice="Dash";
        }else if(monName.contains("Local")&&monName.contains("Moment")){
            lineChoice="Dash";
        }
        PlotTool.setDataLines(tmpMonData,lineChoice,lineThickness,colorArray);
        PlotTool.setDataSymbols(tmpMonData,symbChoice,symbThickness,2,2,colorArray);
      }catch(NeoException e){
        simu.println("Something broke while setting up Plot: "+tmpPlot.getPresentationName());
      }
      try{
        tmpPlot.encode(plotPath + tmpPlot.getPresentationName()+".png","png", outputRes[0], outputRes[1],true);
        tmpPlot.close();
        completedPlots.add(tmpPlot);
      }catch(NeoException e){
        simu.println("unable to render Plot: "+tmpPlot.getPresentationName());
      }
    }
    return completedPlots;
  }
  private ArrayList<StarPlot> postProcessMiscXYPlots(Simulation simu, String plotPath, String csvPath, 
                                           ArrayList<StarPlot> donePlots){
    // method postProcessMiscXYPlots take remaining plots not captured in previous post processing outputs
    //   and outputs them to the user misc folder
    //
    // String plotPath is the path for the plots
    // String csvPath is the path for the csv files from the monitors
    // ArrayList<StarPlot> donePlots is all previously post-process plots
    // 
    // System I/O check  
    SystemTool.touchDirectory(plotPath);
    SystemTool.touchDirectory(csvPath);

    //
    ArrayList<StarPlot> completedPlots = new ArrayList();
    
    //Post Process Remaining Plots
    Collection<StarPlot> allPlots = simu.getPlotManager().getObjects();
    allPlots.removeAll(donePlots);
    for(StarPlot tmpPlot:allPlots){
      if(!(tmpPlot instanceof XYPlot)){
        allPlots.remove(tmpPlot);
      }
    }
    simu.println("Post processing remaining misc plots");
    for(StarPlot tmpPlot:allPlots){
      try{
        tmpPlot.encode(plotPath + tmpPlot.getPresentationName()+".png","png", outputRes[0], outputRes[1],true);
        tmpPlot.close();
        completedPlots.add(tmpPlot);
      }catch(NeoException e){
        simu.println("unable to render Plot: "+tmpPlot.getPresentationName());
      }
    }

    return completedPlots;
  }
  //
  //=============================================
  // Scenes Methods
  //=============================================
  // setup important scenes
  private ArrayList<Scene> setupKiteScenes(boolean needRunRotors){
    Scene tmpScene;
    PartDisplayer tmpPD;
    ScalarDisplayer tmpSD;
    Collection<Part> tmpColl = new ArrayList();
    // Auto-range options:
    // "Min and Max Values"
    // "Min Value"
    // "Max Value"
    // Clip options
    // "Below Min, Above Max"
    // "Below Min"
    // "Above Max"
    double[] axisLoc = {0.,0.,0.05,0.2}; //moving axis on scenes
    VisView near = fieldViews.get(0);
    simu.println("******near view for kite scenes is: "+near.getPresentationName());
    ArrayList<Scene> bodyScenes=new ArrayList();
    simu.println("M600 PreProc: Setting up Kite Scenes....");
    //============================================
    // Kite-specific - Reference Frame Invariant
    //============================================
    // Skin Friction Coefficient
    simu.println("M600 PreProc:   ...Skin Friction Coeff");
    tmpScene = SceneTool.getScene(simu,"Kite - Skin Friction Coefficient");
    if(needRunRotors){
      tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
      SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Skin Fric Coef",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",0.0,0.005,simProxy);
    SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    bodyScenes.add(tmpScene);
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    //Initialize solver for wall y+ scenes
    simu.initializeSolution();
    // Viscous Sublayer
    simu.println("M600 PreProc:   ...Viscous Layer");
    tmpScene=SceneTool.getScene(simu,"Kite - Wall Y+ Viscous Sublayer");
    if(needRunRotors){
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",0.0,5.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"water",5,6,false,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    bodyScenes.add(tmpScene);
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    // Buffer Layer
    simu.println("M600 PreProc:   ...Buffer Layer");
    tmpScene=SceneTool.getScene(simu,"Kite - Wall Y+ Buffer Layer");
    if(needRunRotors){
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",5.0,100.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    bodyScenes.add(tmpScene);
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    // Log Layer
    simu.println("M600 PreProc:   ...Log Layer");
    tmpScene=SceneTool.getScene(simu,"Kite - Wall Y+ Log Layer");
    if(needRunRotors){
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",100.0,300.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,false,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    bodyScenes.add(tmpScene);    
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    // Cp_Low
    simu.println("M600 PreProc:   ...cP Low Pressure Side");
    tmpScene=SceneTool.getScene(simu,"Kite - Cp Low");
    if(needRunRotors){
      tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
      SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cp low",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient","Off","Off",-5.0,0.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"blue-red",32,6,false,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    bodyScenes.add(tmpScene);
    for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    // Cp_High
    simu.println("M600 PreProc:   ...cP High Pressure Side");
    tmpScene=SceneTool.getScene(simu,"Kite - Cp High");
    if(needRunRotors){
      tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
      SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cp high",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient","Off","Off",0.0,1.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"blue-red",32,6,false,"Left Side");
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    bodyScenes.add(tmpScene);
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    
    //=======================================
    // 
    //=======================================
    // Streamlines
    simu.println("M600 PreProc:   ...Streamline Images");
    tmpScene = SceneTool.getScene(simu,"Kite - Main Wing Streamlines");

    // geometry displayers
    tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
    if(needRunRotors){
      tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",
              allWallParts(allRotorRegs),simProxy);
      SceneTool.setPartDisplayerField(tmpPD, false, false, false ,true,0,0.5);
    }
    SceneTool.setPartDisplayerField(tmpPD, false, false, false ,true,0,0.5);

    // streamline displayers
    ArrayList<StreamPart> m600Streamlines = 
            getM600MainWingStreamLines(allBodyRegs);
    PrimitiveFieldFunction pFF = 
            getCaseVelocityReferenceFramePrimitiveFF(isCrossWindMRFCase);
    VectorMagnitudeFieldFunction vMFF = 
            ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
    StreamDisplayer tmpStreamDisp = SceneTool
            .getStreamLineDisplayer(tmpScene, "M600 Streamlines",
                    m600Streamlines);
    tmpStreamDisp.getScalarDisplayQuantity().setFieldFunction(vMFF);
    SceneTool.setStreamScalarDisplayerProperties(tmpStreamDisp, "Off", "Off",
            40.0, 80.0, simProxy); // kite speed is nominally 60 m/s
    SceneTool.setStreamColormap(tmpStreamDisp,"cool-warm",32,6,false,"Left Side");
    tmpStreamDisp.getInputParts().setObjects(m600Streamlines);
    bodyScenes.add(tmpScene);
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);

    // Wind Error
    if(isCrossWindMRFCase){
      simu.println("M600 PreProc:   ...Cross Wind Error");
      tmpScene = SceneTool.getScene(simu,"Wind Error Percentage");
      // geometry displayers
      SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
      if(needRunRotors){
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD, false, false, false ,true,0,0.5);
      }
      PlaneSection bodyZSlice = DerivedPartTool.singlePlane(simu, allBodyRegs,
          "Field Slice Body Z=0", bodyCsys, zeroOrigin, zOnlyAxis);
      tmpSD = SceneTool.getScalarDisplayer(tmpScene, "Wind Error Percentage",
              Collections.singleton(bodyZSlice));
      
      SceneTool.setScalarDisplayerField(tmpSD,"wind_error_pct", "Off", "Off",
              0.0, 5.0, simProxy);
      bodyScenes.add(tmpScene);
      for(Annotation tmpAnn:standardAnnotations){
        SceneTool.addAnnotation(tmpScene,tmpAnn);
      }
      SceneTool.removeDefaultLogo(tmpScene);
      tmpScene.close(true);
    }

    //Reversed Flow - no skin fric
    tmpScene = setupKiteCfWithReversedFlow(needRunRotors);
    bodyScenes.add(tmpScene);
    tmpScene.close(true);
    return  bodyScenes;
  }
  
  private Scene setupKiteCfWithReversedFlow(boolean needRunRotors){
    Scene tmpScene;
    PartDisplayer tmpPD;
    ScalarDisplayer tmpSD;
    ArrayList<Part> tmpColl = new ArrayList();
    double[] axisLoc = {0.,0.,0.05,0.2};
    //Reversed Flow - with skin fric
    simu.println("M600 PreProc:   ...Reversed Flow With Skin Friction");
    tmpScene = SceneTool.getScene(simu,"Reversed Flow with Cf");
    if(needRunRotors){
      tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
      SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.5);
    }
    tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
    SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
    tmpPD.Delete(); // don't need to have this in the skin friction scene
    tmpColl.add((Part) getReversedFlowPart(allBodyRegs));
    tmpPD = SceneTool.getPartDisplayer(tmpScene,"Reversed Flow",tmpColl,simProxy);
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Skin Fric Coef",allWallParts(allBodyRegs));
    SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",0.0,0.005,simProxy);
    SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
    SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
    SceneTool.axisLocation(tmpScene, axisLoc, true);
    tmpScene.getCurrentView().setView(fieldViews.get(0));
    for(Annotation tmpAnn:standardAnnotations){
      SceneTool.addAnnotation(tmpScene,tmpAnn);
    }
    SceneTool.removeDefaultLogo(tmpScene);
    tmpScene.close(true);
    return tmpScene;
  }
  
  private ArrayList<Scene> setupFieldScenes(boolean needRunRotors){
        Scene tmpScene;
        // Auto-range options
        // "Min and Max Values"
        // "Min Value"
        // "Max Value"
        // Clip options
        // "Below Min, Above Max"
        // "Below Min"
        // "Above Max"
        //Post
        ArrayList<Scene> fieldScenes=new ArrayList();
        
        //===========================
        simu.println("setupFieldTKE");
        tmpScene = setupFieldTKE(needRunRotors);
        fieldScenes.add(tmpScene);
        tmpScene.close(true);
        simu.println("setupFieldVelocityIn");
        tmpScene = setupFieldVelocityIn(needRunRotors);
        fieldScenes.add(tmpScene);
        tmpScene.close(true);
        simu.println("setupFieldVorticityIN");
        tmpScene = setupFieldVorticityIn(needRunRotors);
        fieldScenes.add(tmpScene);
        tmpScene.close(true);
        simu.println("setupFieldTVR");
        tmpScene = setupFieldTVR(needRunRotors);
        fieldScenes.add(tmpScene);
        tmpScene.close(true);
        simu.println("setupFieldVortMag");
        tmpScene = setupFieldVortMag(needRunRotors);
        fieldScenes.add(tmpScene);
        tmpScene.close(true);
        simu.println("done fieldScenes");
        return fieldScenes;
        
    }
  private Scene setupFieldTKE(boolean needRunRotors){
    //Method to set up TKE scene
        Scene tmpScene;
        PartDisplayer tmpPD;
        ScalarDisplayer tmpSD;
        double[] axisLoc = {0.,0.,0.05,0.2};
        ArrayList dispPlane = new ArrayList();
        dispPlane.add(getPlane("Body - Center Cut"));
    
        simu.println("Setting up tke");
        tmpScene=SceneTool.getScene(simu,"Field - Turbulent Kinetic Energy");
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        if(needRunRotors){
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
            SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        }
        tmpSD = SceneTool.getScalarDisplayer(tmpScene,"tke",dispPlane);
        SceneTool.setScalarDisplayerField(tmpSD,"TurbulentKineticEnergy","Off","Below Min",0.0,10.0,simProxy);
        SceneTool.setScalarColormap(tmpSD,"blue-red",11,6,false,"Left Side");
        tmpScene.getCurrentView().setView(fieldViews.get(0));
        SceneTool.axisLocation(tmpScene, axisLoc, true);
        SceneTool.removeDefaultLogo(tmpScene);
        for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
        }
        tmpScene.close(true);
    return tmpScene;
  }
  private Scene setupFieldVelocityIn(boolean needRunRotors){
        Scene tmpScene;
        PartDisplayer tmpPD;
        ScalarDisplayer tmpSD;
        double[] axisLoc = {0.,0.,0.05,0.2};
        ArrayList horzPlane = new ArrayList();
        horzPlane.add(getPlane("Body - Horizontal Cut"));

        simu.println("Setting up velocity");
        tmpScene=SceneTool.getScene(simu,"Field - Velocity - Inlet I");
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        if(needRunRotors){
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
            SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        }
        PrimitiveFieldFunction pFF = ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
        //PrimitiveFieldFunction pFF = getCaseVelocityReferenceFramePrimitiveFF(isCrossWindMRFCase);
        simu.println("pff is: "+pFF.getPresentationName());
        VectorComponentFieldFunction vCFF;
        if(isCrossWindMRFCase){
         RotatingReferenceFrame rotRF = 
            ((RotatingReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("ReferenceFrame for Kite XWind MRF"));
         vCFF = ((VectorComponentFieldFunction) pFF.getFunctionInReferenceFrame(rotRF
            ).getFunctionInCoordinateSystem(inletCsys).getComponentFunction(0));
        }else{
             vCFF=((VectorComponentFieldFunction)
                pFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(0));
        }
        
        tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Velocity Inlet",horzPlane);
        double tmpAirSpeed = getKiteSpeed();
        simu.println("air speed is: "+tmpAirSpeed);
        SceneTool.setScalarVectorComponent(tmpSD,"Velocity",inletCsys,0,"Off","Off",tmpAirSpeed*0.5,tmpAirSpeed*1.5,simProxy);
        tmpSD.getScalarDisplayQuantity().setFieldFunction(vCFF);
        tmpSD.getScalarDisplayQuantity().setAutoRange(false);
        tmpSD.getScalarDisplayQuantity().setRange(new double[] {tmpAirSpeed*0.5,tmpAirSpeed*1.5});
        SceneTool.setScalarColormap(tmpSD,"cool-warm",32,6,false,"Left Side");
        tmpScene.getCurrentView().setView(fieldViews.get(0));
        SceneTool.axisLocation(tmpScene, axisLoc, true);
        SceneTool.removeDefaultLogo(tmpScene);
        for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
        }
        tmpScene.close(true);
        return tmpScene;
  }
  private Scene setupFieldVorticityIn(boolean needRunRotors){
        Scene tmpScene;
        PartDisplayer tmpPD;
        ScalarDisplayer tmpSD;
        double[] axisLoc = {0.,0.,0.05,0.2};
        ArrayList horzPlane = new ArrayList();
        horzPlane.add(getPlane("Body - Horizontal Cut"));
               
        tmpScene=SceneTool.getScene(simu,"Field - Vorticity - Inlet J");
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        if(needRunRotors){
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
            SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        }
        tmpSD = SceneTool.getScalarDisplayer(tmpScene,"VorticityVector",horzPlane);
        SceneTool.setScalarVectorComponent(tmpSD,"VorticityVector",inletCsys,1,"Off","Off",-1000.0,1000.0,simProxy);
        SceneTool.setScalarColormap(tmpSD,"cool-warm",32,6,false,"Left Side");
        tmpScene.getCurrentView().setView(fieldViews.get(0));
        SceneTool.axisLocation(tmpScene, axisLoc, true);
        SceneTool.removeDefaultLogo(tmpScene);
        for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
        }
        tmpScene.close(true);
        return tmpScene;
  }
  private Scene setupFieldTVR(boolean needRunRotors){
        Scene tmpScene;
        PartDisplayer tmpPD;
        ScalarDisplayer tmpSD;
        double[] axisLoc = {0.,0.,0.05,0.2};
        ArrayList dispPlane = new ArrayList();
        dispPlane.add(getPlane("Body - Center Cut"));
      
        tmpScene=SceneTool.getScene(simu,"Field - Turbulent Viscosity Ratio");
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        if(needRunRotors){
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
            SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        }
        tmpSD = SceneTool.getScalarDisplayer(tmpScene,"TVR",dispPlane);
        SceneTool.setScalarDisplayerField(tmpSD,"TurbulentViscosityRatio","Off","Off",0,750.0,simProxy);
        SceneTool.setScalarColormap(tmpSD,"blue-red",32,6,false,"Left Side");
        tmpScene.getCurrentView().setView(fieldViews.get(0));
        SceneTool.axisLocation(tmpScene, axisLoc, true);
        SceneTool.removeDefaultLogo(tmpScene);
        for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
        }
        tmpScene.close(true);
        return tmpScene;
  }
  private Scene setupFieldVortMag(boolean needRunRotors){
        Scene tmpScene;
        PartDisplayer tmpPD;
        ScalarDisplayer tmpSD;
        double[] axisLoc = {0.,0.,0.05,0.2};
        VectorMagnitudeFieldFunction vMFF;
        
        simu.println("Setting up Vorticity Magnutide Derived Parts");
        ResampledVolumePart nearFieldResampledVolumePart = 
            DerivedPartTool.getResampledVolumePart(simu, "Resampled Volume - Near Field");
        DerivedPartTool.setResampledVolumeCorners(nearFieldResampledVolumePart,
                new double[] {3.0,-16.0, -3.5}, new double[] {-15.0, 16.0, 3.0});
        nearFieldResampledVolumePart.setCellToVoxelRatio(5.0);
        nearFieldResampledVolumePart.getMinimumVoxelSize().setValue(0.05);
        ResampledVolumePart farFieldResampledVolumePart = 
                DerivedPartTool.getResampledVolumePart(simu, "Resampled Volume - Far Field");
        if(isCrossWindMRFCase){
          DerivedPartTool.setResampledVolumeCorners(farFieldResampledVolumePart,
                  new double[] {-15.0, 13.0, -8.0}, new double[] {-45.0, -50.0, 10.0});
        }else{
          DerivedPartTool.setResampledVolumeCorners(farFieldResampledVolumePart,
            new double[] {-15.0, 20.0, -8.0}, new double[] {-55.0, -20.0, 10.0});
        }
        farFieldResampledVolumePart.setCellToVoxelRatio(5.0);
        farFieldResampledVolumePart.getMinimumVoxelSize().setValue(0.05);

        ArrayList<ResampledVolumePart> omegaVRParts = new ArrayList();
        nearFieldResampledVolumePart.getInputParts().setObjects(allBodyRegs);
        farFieldResampledVolumePart.getInputParts().setObjects(allBodyRegs);
        omegaVRParts.add(nearFieldResampledVolumePart);
        omegaVRParts.add(farFieldResampledVolumePart);
        
        simu.println("Setting up Vorticity Magnutide Scene");
        tmpScene = SceneTool.getScene(simu,"Field - Vorticity Magnitude");
        tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",allWallParts(allBodyRegs),simProxy);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        if(needRunRotors){
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Rotors",allWallParts(allRotorRegs),simProxy);
            SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0);
        }
        String scalarColor = ColorMapTool.getHighContrastBands(simu);
        tmpSD = SceneTool.getScalarDisplayer(tmpScene, "Vorticity", omegaVRParts);
        VorticityVectorFunction vortVF = 
                  ((VorticityVectorFunction) simu.getFieldFunctionManager().getFunction("VorticityVector"));
        vMFF= ((VectorMagnitudeFieldFunction) vortVF.getMagnitudeFunction());
        SceneTool.setScalarDisplayerField(tmpSD,vMFF.getFunctionName(),"Off", "Off", 0.0, 100.0, simProxy);
        tmpSD.getInputParts().setObjects(omegaVRParts);
        SceneTool.setScalarColormap(tmpSD,scalarColor,32,6,false,"Left Side");
        VolumeRenderingSettings vrSettings = tmpSD.getVolumeRenderingSettings();
        vrSettings.setLightingMode(VolumeLightingMode.LOCAL_LIGHTING);
        vrSettings.setAccuracy(1.0);
        vrSettings.setRenderingMode(VolumeRenderingMode.DIRECT);
        SceneTool.axisLocation(tmpScene, axisLoc, true);
        tmpScene.getCurrentView().setView(fieldViews.get(0));
        SceneTool.removeDefaultLogo(tmpScene);
        for(Annotation tmpAnn:standardAnnotations){
          SceneTool.addAnnotation(tmpScene,tmpAnn);
        }
        tmpScene.close(true);
        
      return tmpScene;
  }
  private ArrayList<Scene> setupRotorScenes(){
    Scene tmpScene;
    PartDisplayer tmpPD;
    ScalarDisplayer tmpSD;
    Collection<Part> tmpColl = new ArrayList();
    // Auto-range options
    // "Min and Max Values"
    // "Min Value"
    // "Max Value"
    // Clip options
    // "Below Min, Above Max"
    // "Below Min"
    // "Above Max"
    ArrayList<Scene> rotorScenes=new ArrayList();
    //walls for kite
    Collection<NamedObject> kiteBodyWalls=allWallParts(gatherBodyRegions());

    //Take all regions, subtract anything in kite
    simu.println("Gathering Rotating Regions");
    Collection<Region> allRotorRegions=gatherRotatingBodyRegions();
    double[] tmpdbl = getMinMaxRotorSpeed(allRotorRegions);

    //Scenes with All Rotors
    simu.println("Setting up Rotor Spin Direction");
    tmpScene=SceneTool.getScene(simu,"Rotor Spin Direction");
    tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",kiteBodyWalls,simProxy);
    tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Velocity",allWallParts(allRotorRegions));
    SceneTool.setScalarVectorComponent(tmpSD,"Velocity",inletCsys,1,"Auto","Off",-1.0,1.0,simProxy);
    SceneTool.setScalarColormap(tmpSD,"blue-red",11,6,true,"Bottom");
    setSceneView(tmpScene,zeroOrigin,new double[] {25.,0.,0.},new double[] {0.,0.,-1.0},bodyCsys,1.0,0);

    //Scenes for each rotor
    simu.println("Setting up Scenes for Each Rotor");

    for(Region tmpReg:allRotorRegions){
        // regions to group together
        ArrayList<Scene> addToGroup=new ArrayList();
        // walls for rotors
        Collection<NamedObject> rotorBodyWalls=allWallParts(tmpReg);
        //region specific stuff
        String regName=tmpReg.getPresentationName();
        CartesianCoordinateSystem regCoord = SimTool.getNestedCoordinate(bodyCsys, regName);

        double[] coordOrig=regCoord.getOrigin().getVector().toDoubleArray();
        double[] focalPt = new double[3]; 
        focalPt[0]=coordOrig[0];
        focalPt[1]=coordOrig[1];
        focalPt[2]=coordOrig[2];
        focalPt[0]=focalPt[0]+4.6;
        //account for freestream normalization of coefficients
        double tipSpeed=getRotorSpeed(tmpReg.getPresentationName())*1.26;
        double factor=(airSpeed*airSpeed+tipSpeed*tipSpeed)/(airSpeed*airSpeed);

        //Skin Friction
        simu.println(regName+": Skin Friction Coefficient");
        //On kite

            tmpScene=SceneTool.getScene(simu,"Kite - "+regName+" Skin Friction Coefficient");
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cf",allWallParts(tmpReg));
            SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",0.0,0.005*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,coordOrig,focalPt,new double[] {0.,0.,-1.0},bodyCsys,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
        //on rotor
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Skin Friction Coefficient");
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",kiteBodyWalls,simProxy);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cf",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",0.0,0.005*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,6.0},new double[] {1.,0.,0.0},regCoord,-1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Skin Friction Coefficient Back");
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",kiteBodyWalls,simProxy);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cf",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",0.0,0.005*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,-6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
        simu.println(regName+": Pressure Coefficient");
        //Pressure Coefficient
            //On kite
            tmpScene=SceneTool.getScene(simu,"Kite - "+regName+" Pressure Coefficient");
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cp",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient","Off","Off",-6.0*factor,1.0*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,coordOrig,focalPt,new double[] {0.,0.,-1.0},bodyCsys,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            //on rotor
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Pressure Coefficient");
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",kiteBodyWalls,simProxy);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cp",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient","Off","Off",-6.0*factor,1.0*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            //
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Pressure Coefficient Back");
            tmpPD = SceneTool.getPartDisplayer(tmpScene,"Kite",kiteBodyWalls,simProxy);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Cp",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient","Off","Off",-6.0*factor,1.0*factor,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,-6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
        simu.println(regName+": Wall y+");
        //Wall y+
        //On kite
            // Viscous Sublayer
            tmpScene=SceneTool.getScene(simu,"Kite - "+regName+" Wall Y+ Viscous Sublayer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",0.0,5.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"water",5,6,false,"Left Side");
            setSceneView(tmpScene,coordOrig,focalPt,new double[] {0.,0.,-1.0},bodyCsys,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Buffer Layer
            tmpScene=SceneTool.getScene(simu,"Kite - "+regName+" Wall Y+ Buffer Layer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",5.0,100.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,coordOrig,focalPt,new double[] {0.,0.,-1.0},bodyCsys,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Log Layer
            tmpScene=SceneTool.getScene(simu,"Kite - "+regName+" Wall Y+ Log Layer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",100.0,300.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,false,"Left Side");
            setSceneView(tmpScene,coordOrig,focalPt,new double[] {0.,0.,-1.0},bodyCsys,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
        //on rotor
            // Viscous Sublayer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Viscous Sublayer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",0.0,5.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"water",5,6,false,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Buffer Layer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Buffer Layer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",5.0,100.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Log Layer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Log Layer");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",100.0,300.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,false,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Viscous Sublayer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Viscous Sublayer Back");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",0.0,5.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"water",5,6,false,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,-6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Buffer Layer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Buffer Layer Back");
            //wipeDisplayers(tmpScene);
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",5.0,100.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,true,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,-6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);
            // Log Layer
            tmpScene=SceneTool.getScene(simu,"Rotor - "+regName+" Wall Y+ Log Layer Back");
            tmpSD = SceneTool.getScalarDisplayer(tmpScene,"Low y+",rotorBodyWalls);
            SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Below Min, Above Max",100.0,300.0,simProxy);
            SceneTool.setScalarColormap(tmpSD,"spectrum",11,6,false,"Left Side");
            setSceneView(tmpScene,zeroOrigin,new double[] {0.,0.0,-6.0},new double[] {1.,0.,0.0},regCoord,1.0,0);
            addToGroup.add(tmpScene);
            rotorScenes.add(tmpScene);
            SceneTool.removeDefaultLogo(tmpScene);
            tmpScene.close(true);

        //add to Rotor Group
        SceneTool.groupScenes(simu, regName, addToGroup);

      }
      return rotorScenes;
  }

  // post process slices
  private void postYPlaneSlices(ArrayList<Scene> sceneList){
    double yStart = -13.0;
    double yEnd = 13.0;
    int nSlices = 261;
    double yValue;

    // use a Y-plane slice
    PlaneSection yCutter = DerivedPartTool.singlePlane(simu,
            simu.getRegionManager().getObjects(),
            "Y Slicer",bodyCsys,zeroOrigin,yOnlyAxis);

    ArrayList<NamedObject> partsBin=new ArrayList();
    partsBin.add(yCutter);

    // int dispMesh = 0 for no mesh -> goes to USER folder
    //     dispMesh = 1 for all mesh -> goes to CFD folder
    for( int dispMesh = 0; dispMesh<2 ; dispMesh ++){
      for(Scene tmpScene:sceneList){
        Collection<Displayer> dispList = tmpScene.getDisplayerManager().getObjects();
        for(Displayer tmpDisp:dispList){
          // put slice into scalar displayer
          if(tmpDisp instanceof ScalarDisplayer){
              String sdName=tmpDisp.getPresentationName();
              ((ScalarDisplayer) tmpDisp).getInputParts().setObjects();
              ScalarDisplayer tmpSD = SceneTool.getScalarDisplayer(tmpScene,sdName,partsBin);
              tmpSD.setDisplayMesh(dispMesh);
              if(dispMesh == 0){ // Smooth
                tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
              }else{ // Cell-valued
                tmpSD.setFillMode(ScalarFillMode.CELL_FILLED);
              }
          }
          //make geometry parts translucent
          if(tmpDisp instanceof PartDisplayer){
              tmpDisp.setOpacity(1.0);
          }
        }
        String fieldSceneName=tmpScene.getPresentationName().replaceAll("\\s","_");

        //walk through the slices
        for(int i=0;i<=nSlices;i++){
          yValue = yStart+i*(yEnd-yStart)/nSlices;
          double[] newOrigin = {0.,yValue,0.};
          yCutter.getOriginCoordinate().setCoordinate(prefUVec, prefUVec, prefUVec,
                  new DoubleVector(newOrigin));
          setSceneView(tmpScene,new double[] {-1.8,yValue,0.0}, new double[] {-1.8,yValue-4.0,0.0},
              new double[] {0.,0.,-1.0},bodyCsys,4.0,1);

          String myCounter="";
          if(i<10){
              myCounter="0"+i;
          }else{
              myCounter=myCounter+i;
          }
          String fieldFolder = "";
          if ( dispMesh == 0 ){
            fieldFolder = userScenes + File.separator + fieldSceneName;
          }else{
            fieldFolder = cfdScenes + File.separator + fieldSceneName;
          }
          SystemTool.touchDirectory(fieldFolder);
          tmpScene.printAndWait(fieldFolder + File.separator 
            + tmpScene.getPresentationName() + "_Y_" + myCounter + ".png", 1, outputRes[0], outputRes[1]);
        }
      }
    }
  }
  private void postZPlaneSlices(ArrayList<Scene> sceneList){
    double zStart=4.0;
    double zEnd=-3.0;
    int nSlices = 71;
    double zValue;
    PlaneSection zCutter=DerivedPartTool.singlePlane(simu, 
            simu.getRegionManager().getObjects(),
            "Z Slicer",bodyCsys,zeroOrigin,zOnlyAxis);
    ArrayList<NamedObject> partsBin=new ArrayList();
    partsBin.add(zCutter);

    // int dispMesh = 0 for no mesh -> goes to USER folder
    //     dispMesh = 1 for all mesh -> goes to CFD folder
    for( int dispMesh = 0; dispMesh<2 ; dispMesh ++){
      for(Scene tmpScene:sceneList){
        String sceneName=tmpScene.getPresentationName();
        Collection<Displayer> dispList=tmpScene.getDisplayerManager().getObjects();
        for(Displayer tmpDisp:dispList){
          if(tmpDisp instanceof ScalarDisplayer){
            String sdName=tmpDisp.getPresentationName();
            ((ScalarDisplayer) tmpDisp).getInputParts().setObjects();
            ScalarDisplayer tmpSD = SceneTool.getScalarDisplayer(tmpScene,sdName,partsBin);
            if(dispMesh == 0){ // Smooth
              tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
            }else{ // Cell-valued
              tmpSD.setFillMode(ScalarFillMode.CELL_FILLED);
            }
            tmpSD.setDisplayMesh(dispMesh);
          }
          if(tmpDisp instanceof PartDisplayer){
            tmpDisp.setOpacity(1.0);
          }
        }
        String fieldSceneName=
                tmpScene.getPresentationName().replaceAll("\\s","_");
        //walk through the slices
        for(int i=0;i<=nSlices;i++){
          zValue = zStart+i*(zEnd-zStart)/nSlices;
          double[] newOrigin = {0.,0.0,zValue};
          zCutter.getOriginCoordinate().setCoordinate(prefUVec,
                  prefUVec, prefUVec,
                  new DoubleVector(newOrigin));

          setSceneView(tmpScene,new double[] {-1.8,0.0,zValue},
                  new double[] {-1.8,0.0,-30.0},
                  new double[] {1.,0.,0.0},bodyCsys,4.0,0);

          String myCounter="";
          if(i<10){
              myCounter="0"+i;
          }else{
              myCounter=myCounter+i;
          }
          String fieldFolder = "";
          if ( dispMesh == 0 ){
            fieldFolder = userScenes + File.separator + fieldSceneName;
          }else{
            fieldFolder = cfdScenes + File.separator + fieldSceneName;
          }
          SystemTool.touchDirectory(fieldFolder);
          tmpScene.printAndWait(fieldFolder + File.separator + sceneName + "_Z_" + myCounter + ".png", 
            1, outputRes[0], outputRes[1]);
          }
      }
    }
  }
  private void postYWingSlices(ArrayList<Scene> sceneList){
    double yStart=-13.0;
    double yEnd=13.0;
    int nSlices = 14;
    double yValue;

    PlaneSection yCutter = DerivedPartTool.singlePlane(simu,
            simu.getRegionManager().getObjects(),
            "Y Slicer",bodyCsys,zeroOrigin,yOnlyAxis);

    ArrayList<NamedObject> partsBin=new ArrayList();
    partsBin.add(yCutter);

    for( int dispMesh = 0; dispMesh < 2 ; dispMesh ++){
      for(Scene tmpScene:sceneList){
        Collection<Displayer> dispList=tmpScene.getDisplayerManager().getObjects();
        for(Displayer tmpDisp:dispList){
          if(tmpDisp instanceof ScalarDisplayer){
            String sdName=tmpDisp.getPresentationName();
            ((ScalarDisplayer) tmpDisp).getInputParts().setObjects();
            ScalarDisplayer tmpSD = SceneTool.getScalarDisplayer(tmpScene,sdName,partsBin);
            tmpSD.setDisplayMesh(dispMesh);
            if(dispMesh == 0){ // Smooth
              tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
            }else{ // Cell-valued
              tmpSD.setFillMode(ScalarFillMode.CELL_FILLED);
            }            
          }
          if(tmpDisp instanceof PartDisplayer){
            tmpDisp.setVisibilityOverrideMode(DisplayerVisibilityOverride.SHOW_ALL_PARTS);
          }
        }

        String fieldSceneName=tmpScene.getPresentationName().replaceAll("\\s","_");
        //walk through the slices
        for(int i=0;i<=nSlices;i++){
          yValue = yStart+i*(yEnd-yStart)/nSlices;
          double zValue = (-0.2-0.0)/(13.0-6.6)*(Math.abs(yValue)+0.0)-0.0;
          double[] newOrigin = {0.,yValue,0.};
          yCutter.getOriginCoordinate().setCoordinate(prefUVec, prefUVec, prefUVec,
                  new DoubleVector(newOrigin));

          setSceneView(tmpScene,new double[] {-0.55,yValue,zValue}, new double[] {-0.55,yValue-2.0,zValue},
              new double[] {0.,0.,-1.0},bodyCsys,4.0,0);
          String myCounter="";
          if(i<10){
              myCounter="0"+i;
          }else{
              myCounter=myCounter+i;
          }
          String fieldFolder = "";
          if ( dispMesh == 0 ){
            fieldFolder = userScenes + File.separator + fieldSceneName;
          }else{
            fieldFolder = cfdScenes + File.separator + fieldSceneName;
          }
          SystemTool.touchDirectory(fieldFolder);
          tmpScene.printAndWait(fieldFolder + File.separator + "Wing_Slice" + "_Y_" + myCounter + ".png",
            1, outputRes[0], outputRes[1]);
        }   
      }
    }
  }
  private void postYFlapSlices(ArrayList<Scene> sceneList){
    double yStart=-13.0;
    double yEnd=13.0;
    int nSlices = 131;
    double yValue;

    PlaneSection yCutter = DerivedPartTool.singlePlane(simu,
            simu.getRegionManager().getObjects(),
            "Y Slicer",bodyCsys,zeroOrigin,yOnlyAxis);

    ArrayList<NamedObject> partsBin=new ArrayList();
    partsBin.add(yCutter);

    for( int dispMesh = 0; dispMesh < 2 ; dispMesh ++){    
      for(Scene tmpScene:sceneList){
        Collection<Displayer> dispList=tmpScene.getDisplayerManager().getObjects();
        for(Displayer tmpDisp:dispList){
          if(tmpDisp instanceof ScalarDisplayer){
            String sdName=tmpDisp.getPresentationName();
            ((ScalarDisplayer) tmpDisp).getInputParts().setObjects();
            ScalarDisplayer tmpSD = SceneTool.getScalarDisplayer(tmpScene,sdName,partsBin);
            tmpSD.setDisplayMesh(dispMesh);
            if(dispMesh == 0){ // Smooth
              tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
            }else{ // Cell-valued
              tmpSD.setFillMode(ScalarFillMode.CELL_FILLED);
            }                        
          }
          if(tmpDisp instanceof PartDisplayer){
            tmpDisp.setVisibilityOverrideMode(DisplayerVisibilityOverride.SHOW_ALL_PARTS);
          }
        }
        String fieldSceneName=tmpScene.getPresentationName().replaceAll("\\s","_");
        //walk through the slices
        for(int i=0;i<=nSlices;i++){
            yValue = yStart+i*(yEnd-yStart)/nSlices;
            double[] newOrigin = {0.,yValue,0.};
            yCutter.getOriginCoordinate().setCoordinate(prefUVec, prefUVec, prefUVec,
                    new DoubleVector(newOrigin));

            if(Math.abs(yValue)<6.6){
                setSceneView(tmpScene,new double[] {-0.9,yValue,0.0425}, new double[] {-0.9,yValue-0.3,0.0425},
                new double[] {0.,0.,-1.0},bodyCsys,4.0,0);
            }else{
                double xValue = (-0.8+.32)/(6.7-12.5)*(Math.abs(yValue)-12.5)-0.32;
                double zValue = (-0.444-0.028)/(12.5-6.7)*(Math.abs(yValue)-6.7)-0.028;
                simu.println("x,y,z: "+xValue+","+yValue+","+zValue);
                setSceneView(tmpScene,new double[] {xValue,yValue,zValue}, new double[] {xValue,yValue-0.3,zValue},
                new double[] {0.,0.,-1.0},bodyCsys,4.0,0);
            }
            String myCounter="";
            if(i<10){
                myCounter="0"+i;
            }else{
                myCounter=myCounter+i;
            }
            String fieldFolder = "";
            if ( dispMesh == 0 ){
              fieldFolder = userScenes + File.separator + fieldSceneName;
            }else{
              fieldFolder = cfdScenes + File.separator + fieldSceneName;
            }
            SystemTool.touchDirectory(fieldFolder);
            tmpScene.printAndWait(fieldFolder + File.separator + "Flap_Slice" + "_Y_" + myCounter + ".png",
              1, outputRes[0], outputRes[1]);
        }   
      }
    }
  }
  
  // post-processing methods
  private void postProcessBodyScenes(ArrayList<Scene> tmpScenes){
    //Collection<VisView> tmpViewColl = simu.getViewManager().getObjects();

    long initTime = System.nanoTime();
    String view = "near";
    int i=0;
    for(VisView tmpV:fieldViews){
            if(i==0){
                view = "near";
            } else if(i==1){
                view= "far";
            } 
    for(Scene tmpScene:tmpScenes){
      String sceneName = tmpScene.getPresentationName()+"_"+view;
      tmpScene.getCurrentView().setView(tmpV);
      long sceneStartTime = System.nanoTime();
      if(tmpScenes.contains(tmpScene)&&!tmpScene.getPresentationName().contains("VR_")){
       // VisView tmpSceneView = tmpScene.getCurrentView();
       // for(VisView tmpView:tmpViewColl){
        //    tmpSceneView.copyProperties(tmpView);
            try{
                tmpScene.printAndWait(userScenes + sceneName+".png", 1, outputRes[0], outputRes[1]);
            }catch(NeoException e){
                simu.println("Unable to render Scene: " + sceneName);
            }
       // }
      }
      long sceneEndSceneTime = System.nanoTime();
      double sceneElapsedSec = ((double) (sceneEndSceneTime-sceneStartTime))/1.0E9;
      simu.println(String.format("M600 remaining Scene: " 
        + sceneName + " time to process all views %10.3f (s)",sceneElapsedSec));
      if(needStarView){
          //export this scene to STAR-VIEW
          long startExportTime = System.nanoTime();
          SceneTool.exportSceneToStarView(simu, userScenes + File.separator + 
            "STARVIEW",tmpScene, sceneName, sceneName, false);
          long endExportTime = System.nanoTime();
          double sceneElapsedExportTime = ((double) (endExportTime-startExportTime))/1.0E9;
          simu.println(String.format("M600  Scene: " 
            + sceneName + " time to export to STAR-View %10.3f (s)",sceneElapsedExportTime));
      }
    }i++;
    }
    //long initTime = System.nanoTime();
    long endTime = System.nanoTime();
    double elapsedSec = ((double)(endTime-initTime))/1.0E9;
    simu.println(String.format("Total Time Spent in Remaining Scenes: %1$10.3f (s)",(elapsedSec)));
  }
  private void postProcessRotorScenes(ArrayList<Scene> tmpScenes){

    long initTime = System.nanoTime();
    for(Scene tmpScene:tmpScenes){
      String sceneName = tmpScene.getPresentationName();
      if(tmpScenes.contains(tmpScene)){
        try{
          tmpScene.printAndWait(userScenes + "ROTOR" + File.separator 
            + sceneName+".png", 1, outputRes[0], outputRes[1]);
        }catch(NeoException e){
          simu.println("Unable to render Rotor Scene: "+sceneName);
        }
      }
      //export this scene to STAR-VIEW
      if(needStarView){
          long startExportTime = System.nanoTime();
          SceneTool.exportSceneToStarView(simu, userScenes + "ROTOR" + File.separator + "STARVIEW",
            tmpScene, sceneName, sceneName, false);
          long endExportTime = System.nanoTime();
          double sceneElapsedExportTime = ((double) (endExportTime-startExportTime))/1.0E9;
          simu.println(String.format("M600  Scene: " 
            + sceneName + " time to export to STAR-View %10.3f (s)",sceneElapsedExportTime));
      }
    }
    long endTime = System.nanoTime();
    double elapsedSec = ((double)(endTime-initTime))/1.0E9;
    simu.println(String.format("Total Time Spent in Remaining Scenes: %1$10.3f (s)",(elapsedSec)));
  }
  private void postProcessFieldScenes(ArrayList<Scene> tmpScenes){
   //Export quick post scenes to Case study post folder: 
    ArrayList<Scene> quickPostScenes = new ArrayList();
    Scene tmpScene = setupFieldVelocityIn(needRunRotors);
    quickPostScenes.add(tmpScene);
    tmpScene.close(true);
    tmpScene = setupFieldVortMag(needRunRotors);
    quickPostScenes.add(tmpScene);
    tmpScene.close(true);
    
    String uOut1 = m600PostProcFolder+File.separator +postSubFolder;
    String view = "near";
    int j=0;
    for(VisView tmpV:fieldViews){
        if(j==0){
           view = "near";
        } else if(j==1){
           view= "far";
        } 
        for (Scene tmpSc:quickPostScenes){
          String sceneName = tmpSc.getPresentationName()+"_"+view;
          tmpSc.getCurrentView().setView(tmpV); 
          try{
           tmpSc.printAndWait(uOut1+File.separator+sceneName+".png",
              1,outputRes[0], outputRes[1]);
          }catch(NeoException e){
            simu.println("Unable to render Rotor Scene: "+sceneName+".png");
          }
        }j++;
    }
   //Export Scenes to USER/SCENES   
    String userOut = m600PostProcFolder+File.separator +postSubFolder + File.separator + "USER";
    String sceneOut = userOut + File.separator +"SCENES";
    simu.println("M600 POST: Post processing field scenes");
    simu.println("scene out folder: "+sceneOut);
    //String view = "near";
    int i=0;
    for(VisView tmpV:fieldViews){
        if(i==0){
           view = "near";
        } else if(i==1){
           view= "far";
        } 
        for(Scene tmpS: tmpScenes){
          String sceneName = tmpS.getPresentationName()+"_"+view;
          tmpS.getCurrentView().setView(tmpV);
          simu.println("saving scene to file: "+sceneName);
          try{
           tmpS.printAndWait(sceneOut+File.separator+sceneName+".png",
              1,outputRes[0], outputRes[1]);
          }catch(NeoException e){
            simu.println("Unable to render Rotor Scene: "+sceneName+".png");
          }
          if(needStarView){
            //STAR-VIEW export this scene
            long startExportTime = System.nanoTime();
            SceneTool.exportSceneToStarView(simu, userScenes + File.separator + 
              "STARVIEW",tmpS, sceneName, sceneName, false);
            long endExportTime = System.nanoTime();
            double sceneElapsedExportTime = ((double) (endExportTime-startExportTime))/1.0E9;
             simu.println(String.format("M600  Scene: " 
             + sceneName + " time to export to STAR-View %10.3f (s)",sceneElapsedExportTime));
            }
        }
        i++;
     }
      
  }
  private void quickPostFieldScenes(){
    String quickMSG = "QuickPost MSG: ";
    studyPostDir = m600PostProcFolder + File.separator
        + postSubFolder + File.separator;  
    SystemTool.touchDirectory(m600PostProcFolder);
    SystemTool.touchDirectory(studyPostDir); 

    simu.println(quickMSG + "Setting annotations.");
    initAnnotations(simu);
    
    simu.println(quickMSG + "Creating directories.");
    createPostProcDirectories(simu, studyPostDir);
    
    simu.println(quickMSG + "Setting up derived parts.");
    setupDerivedParts(allBodyRegs);

    simu.println(quickMSG + "Setting kite views.");
    setStandardKiteViews();
    
    simu.println(quickMSG + "Setting field scenes.");
    ArrayList<Scene> quickPostScenes = new ArrayList();
    
    // Velocity
    Scene tmpScene = setupFieldVelocityIn(needRunRotors);
    quickPostScenes.add(tmpScene);
    tmpScene.close(true);
    
    // Vorticity
    tmpScene = setupFieldVortMag(needRunRotors);
    quickPostScenes.add(tmpScene);
    tmpScene.close(true);
   
    // Make sure proxy goes to Volume Mesh
    setSimProxyVolumeMesh();
    
    // Begin output of appropriate scenes.
    String uOut1 = m600PostProcFolder+File.separator +postSubFolder;
    String view = "near";
    int j=0;
    for(VisView tmpV:fieldViews){
      if(j==0){
        view = "near";
      } else if(j == 1){
        view= "far";
      } 
      for (Scene tmpSc:quickPostScenes){
        String sceneName = tmpSc.getPresentationName()+"_"+view;
        tmpSc.getCurrentView().setView(tmpV); 
        try{
          tmpSc.printAndWait(uOut1 + File.separator + sceneName + ".png",
              1,outputRes[0], outputRes[1]);
        }catch(NeoException e){
          simu.println("Unable to render Scene: " + sceneName + ".png");
        }
      }j++;
    }
  }
  //scene building methods
  private CurrentView setSceneView(Scene tmpScene,double[] focalPt, double[] camPos,double[] viewUp,CoordinateSystem tmpCsys,double pllMag,int projMode){
      CurrentView tmpView = tmpScene.getCurrentView();
      Units units_0 = 
          ((Units) simu.getUnitsManager().getObject("m"));
      tmpView.setInput(new DoubleVector(new double[] {focalPt[0],focalPt[1],focalPt[2]}),
               new DoubleVector(new double[] {camPos[0],camPos[1],camPos[2]}),
                   new DoubleVector(new double[] {viewUp[0],viewUp[1],viewUp[2]}), pllMag, projMode);
      tmpView.setCoordinateSystem(tmpCsys);
      Coordinate focalPtCoord = 
          tmpView.getFocalPointCoordinate();
      focalPtCoord.setCoordinate(units_0, units_0, units_0, new DoubleVector(focalPt));
      Coordinate camPosCoord = 
          tmpView.getPositionCoordinate();
      camPosCoord.setCoordinate(units_0, units_0, units_0, new DoubleVector(camPos));
      Coordinate viewUpCoord=
          tmpView.getViewUpCoordinate();
      viewUpCoord.setCoordinate(units_0, units_0, units_0, new DoubleVector(viewUp));
      return tmpView;
  }
 
  //=============================================
  // View Methods
  //=============================================
  private void setStandardKiteViews(){
      //setup for a 2000x1000 frame output
      simu.getViewManager().deleteViews(simu.getViewManager().getObjects());
      VisView near;
      VisView far;
      simu.println("***VIEWs*****");
      simu.println("view angle value is: "+viewAngleDistanceOffset);
      double[] viewCameraPos = {-wingSpanNorm/2,-wingSpanNorm,-wingSpanNorm/2.5};
      simu.println("camera position is: "+viewCameraPos[0]+", "+viewCameraPos[1]+", "+viewCameraPos[2]);
      //double[] viewFocalPt = {-viewCameraPos[0]/16,viewCameraPos[0]/16,-viewCameraPos[0]/25};
      double[] viewFocalPt = {4*viewAngleDistanceOffset,-4*viewAngleDistanceOffset, .1};
      double[] viewUp = {viewAngleDistanceOffset, 2*viewAngleDistanceOffset,viewFocalPt[1]};
      
      if(isCrossWindMRFCase){
        near = SceneTool.getView(simu,"nearfield",bodyCsys,viewFocalPt,viewCameraPos,viewUp,false);  
      }else{
        near = SceneTool.getView(simu,"nearfield",labCsys,viewFocalPt,viewCameraPos,viewUp,false);
      }
      fieldViews.add(near);
           
      //farfield
      double[] viewCameraPosFar = {-1.6*wingSpanNorm,-3.*wingSpanNorm,-1.*wingSpanNorm};
      double[] viewFocalPtFar = {-.75*wingSpanNorm,wingSpanNorm/4., 2};
      double[] viewUpFar = {viewAngleDistanceOffset, 2*viewAngleDistanceOffset,-4*viewAngleDistanceOffset};
      if(isCrossWindMRFCase){
        far = SceneTool.getView(simu,"Farfield", bodyCsys,viewFocalPtFar,viewCameraPosFar,viewUpFar,false);
      }else{
        far = SceneTool.getView(simu,"Farfield", labCsys,viewFocalPtFar,viewCameraPosFar,viewUpFar,false);
      }
      fieldViews.add(far);
      
  }

  //=============================================
  // TOOLS Methods
  //============================================= 
  // representations
  private ProxyRepresentation getSimProxy(String proxyName){
      ProxyRepresentation myProxy;
      try{
          myProxy = (ProxyRepresentation) simu.getRepresentationManager().getObject(proxyName);
      }catch(NeoException e){
          myProxy =  simu.getRepresentationManager().createUserRepresentation();
          myProxy.setPresentationName(proxyName);
      }
      return myProxy;
  }
  private void setSimProxyVolumeMesh(){
      PartRepresentation partRep = 
    ((PartRepresentation) simu.getRepresentationManager().getObject("Volume Mesh"));
      simProxy.setRepresentation(partRep);
  }
  private void setFlowFieldCoefficients(){
      PressureCoefficientFunction pCF = 
      ((PressureCoefficientFunction) simu.getFieldFunctionManager().getFunction("PressureCoefficient"));

      pCF.getReferenceDensity().setDefinition("${Case Density}");
      pCF.getReferencePressure().setValue(0.0);
      
      pCF.getReferenceVelocity().setDefinition("${Case Airspeed}");


      SkinFrictionCoefficientFunction sFCF = 
      ((SkinFrictionCoefficientFunction) simu.getFieldFunctionManager().getFunction("SkinFrictionCoefficient"));
      sFCF.getReferenceDensity().setDefinition("${Case Density}");
      sFCF.getReferenceVelocity().setDefinition("${Case Airspeed}");

  }
  private void setProxyToVolumeMesh(){
      long initTime = System.nanoTime();
      ProxyRepresentation proxy = (ProxyRepresentation) simu.getRepresentationManager().getObject("Proxy");
      proxy.setRepresentation(simu.getRepresentationManager().getObject("Volume Mesh"));
      //long initTime = System.nanoTime();
      long endTime = System.nanoTime();
      double elapsedSec = ((double)(endTime-initTime))/1.0E9;
      simu.println(String.format("Total Time to Switch: %1$10.3f (s)",(elapsedSec)));
  }

  // Motions
  private void setRotorSpeeds(double r_1_l,double r_2_l,double r_3_l,double r_4_l,
                              double r_1_u,double r_2_u,double r_3_u,double r_4_u,
                              boolean isRBM){
      String needFF="";
      if(!isRBM){
          needFF="*$rotor_ramp";
          get_RotorRamp_FF(250,500);
      }

      //Rotor 1
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 1 Lower"))
            .getRotationRate().setDefinition(""+r_1_l+needFF);
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 1 Upper"))
            .getRotationRate().setDefinition(""+r_1_u+needFF);
      //Rotor 1
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 2 Lower"))
            .getRotationRate().setDefinition(""+r_2_l+needFF);
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 2 Upper"))
            .getRotationRate().setDefinition(""+r_2_u+needFF);
      //Rotor 1
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 3 Lower"))
            .getRotationRate().setDefinition(""+r_3_l+needFF);
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 3 Upper"))
            .getRotationRate().setDefinition(""+r_3_u+needFF);
      //Rotor 1
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 4 Lower"))
            .getRotationRate().setDefinition(""+r_4_l+needFF);
    ((RotatingMotion) simu.get(MotionManager.class).getObject("Rotor 4 Upper"))
            .getRotationRate().setDefinition(""+r_4_u+needFF);
  }
  private void applyRotorRigidBodyMotions(){
      //Modify rotor_ramp ff
      get_RotorRamp_FF(0,0);
      for(Region tmpReg:allRotorRegs){
          MotionSpecification regMotionSpecification = 
              tmpReg.getValues().get(MotionSpecification.class);
          LabReferenceFrame labReferenceFrame = 
              ((LabReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("Lab Reference Frame"));
          regMotionSpecification.setReferenceFrame(labReferenceFrame);
          RotatingMotion rotatingMotion_0 = 
              ((RotatingMotion) simu.get(MotionManager.class).getObject(tmpReg.getPresentationName()));
          regMotionSpecification.setMotion(rotatingMotion_0);
      }
  }
  private double getRotorSpeed(String tmpName){
      double retStr;
      //Rotor 1
      retStr =((RotatingMotion) simu.get(MotionManager.class).getObject(tmpName)).getRotationRate().getSIValue();
      return retStr;
  }
  private void setRotatingMotionSpeed(RotatingMotion rotMotion,double rotSpeed,int strtIt,int endIt){
      double retStr;
      //Rotor 1
      String needFF="";
      if(strtIt>0&&endIt>0){
          needFF="*$rotor_ramp";
          get_RotorRamp_FF(strtIt,endIt);
      }
      //Set Rotor Speed
      rotMotion.getRotationRate().setDefinition(""+rotSpeed+needFF);
  }
  private RotatingMotion getRotatingMotion(String motionName){
      RotatingMotion rotMotion;
      try{
          rotMotion = 
              (RotatingMotion) simu.get(MotionManager.class).getObject(motionName);
      }catch(NeoException e){
          rotMotion = 
              simu.get(MotionManager.class).createMotion(RotatingMotion.class, "Rotation");
          rotMotion.setPresentationName(motionName);
      }
      return rotMotion;
  }
  private double[] getMinMaxRotorSpeed(Collection<Region> tmpRegs){
      double min=0.;
      double max=0.;
      double dbl;
      for(Region tmp:tmpRegs){
          dbl=getRotorSpeed(tmp.getPresentationName());
          if(dbl<min){
              min=dbl;
          }else if(dbl>max){
              max=dbl;
          }
      }
      double[] retVal={min,max};
      return retVal;
  }
  private void debugAlphaBetaValue(){
      //Creates individual probe points to check alpha and beta values
      simu.println("Debugging Alpha/Beta Values.");
      //Create 5 probe points in BodyCsys
      //PointPart makePointPart(Simulation tmpSim,String tmpName,CoordinateSystem tmpCsys, double[] xyzLoc)
      ArrayList<PointPart> allPoints = new ArrayList();
      PointPart tmpPoint;
      String pointName = "Point_y=-12";
      tmpPoint = DerivedPartTool.makePointPart(simu, pointName, bodyCsys, new double[] {0.,-12.,0.});
      tmpPoint.getInputParts().addPart(simu.getRegionManager().getObject("Kite"));
      allPoints.add(tmpPoint);
      pointName = "Point_y=-6";
      tmpPoint = DerivedPartTool.makePointPart(simu, pointName, bodyCsys, new double[] {0.,-6.,0.});
      tmpPoint.getInputParts().addPart(simu.getRegionManager().getObject("Kite"));
      allPoints.add(tmpPoint);
      pointName = "Point_y=0";
      tmpPoint = DerivedPartTool.makePointPart(simu, pointName, bodyCsys, new double[] {0.,-0.,0.});
      tmpPoint.getInputParts().addPart(simu.getRegionManager().getObject("Kite"));
      allPoints.add(tmpPoint);
      pointName = "Point_y=6";
      tmpPoint = DerivedPartTool.makePointPart(simu, pointName, bodyCsys, new double[] {0.,6.,0.});
      tmpPoint.getInputParts().addPart(simu.getRegionManager().getObject("Kite"));
      allPoints.add(tmpPoint);
      pointName = "Point_y=12";
      tmpPoint = DerivedPartTool.makePointPart(simu, pointName, bodyCsys, new double[] {0.,12.,0.});
      tmpPoint.getInputParts().addPart(simu.getRegionManager().getObject("Kite"));
      allPoints.add(tmpPoint);
      
      getdebugABReports(allPoints);
  }
  private void getdebugABReports(ArrayList<PointPart> allPts){
      ArrayList<Report> allABReports = new ArrayList();
      ArrayList<Report> allEXPREports = new ArrayList();
		simu.println("Setting up reports.");
		simu.println("IS MRF XWIND:" + isCrossWindMRFCase);
      MaxReport tmpReport;
      VectorComponentFieldFunction vCFF;
      PrimitiveFieldFunction pFF = ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
      if(isCrossWindMRFCase){
        RotatingReferenceFrame rotRF = 
            ((RotatingReferenceFrame) simu.get(ReferenceFrameManager.class).getObject("ReferenceFrame for Kite XWind MRF"));
			simu.println("This is a crosswind case!!!!!");
			simu.println("This is a crosswind case!!!!!");
			simu.println("This is a crosswind case!!!!!");
        for(PointPart tmpPoint:allPts){
            String ptName = tmpPoint.getPresentationName();
            vCFF = ((VectorComponentFieldFunction) pFF.getFunctionInReferenceFrame(rotRF
            ).getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(0));
            tmpReport  = ReportTool.maxReport(simu,ptName+" Velocity_i", vCFF, Collections.singleton(tmpPoint), simProxy);
            tmpReport.setSmooth(true);
            allABReports.add(tmpReport);
            vCFF = ((VectorComponentFieldFunction) pFF.getFunctionInReferenceFrame(rotRF
            ).getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(1));
            tmpReport  = ReportTool.maxReport(simu,ptName+" Velocity_j", vCFF, Collections.singleton(tmpPoint), simProxy);
            tmpReport.setSmooth(true);
            allABReports.add(tmpReport);
            vCFF = ((VectorComponentFieldFunction) pFF.getFunctionInReferenceFrame(rotRF
            ).getFunctionInCoordinateSystem(bodyCsys).getComponentFunction(2));
            tmpReport  = ReportTool.maxReport(simu,ptName+" Velocity_k", vCFF, Collections.singleton(tmpPoint), simProxy);
            tmpReport.setSmooth(true);
            allABReports.add(tmpReport);
           } 
		}else{
          for(PointPart tmpPoint:allPts){
             String ptName = tmpPoint.getPresentationName();
             vCFF=((VectorComponentFieldFunction)
                    pFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(0));
             tmpReport  = ReportTool.maxReport(simu, ptName+" Velocity_i", vCFF, Collections.singleton(tmpPoint), simProxy);
             tmpReport.setSmooth(true);
             allABReports.add(tmpReport);
             vCFF=((VectorComponentFieldFunction)
                    pFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(1));
             tmpReport  = ReportTool.maxReport(simu, ptName+" Velocity_j", vCFF, Collections.singleton(tmpPoint), simProxy);
             tmpReport.setSmooth(true);
             allABReports.add(tmpReport);
             vCFF=((VectorComponentFieldFunction)
                    pFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(2));
             tmpReport  = ReportTool.maxReport(simu, ptName+" Velocity_k", vCFF, Collections.singleton(tmpPoint), simProxy);
             tmpReport.setSmooth(true);
             allABReports.add(tmpReport);
           }
      }
      //Get Expression report for alpha and beta for each probe
      ExpressionReport tmpEPa;
      ExpressionReport tmpEPb;
      for(PointPart tmpPoint:allPts){
        String ptName = tmpPoint.getPresentationName();
        ptName =ptName.replace("\\s", "");
        //Alpha
        String probeAName = ptName +"Alpha";
        String alphaEqn = "atan2(-${"+ptName+"Velocity_kReport},-${"+ptName+"Velocity_iReport}+1.e-20)*180/${pi}";
        tmpEPa = ReportTool.getExpressionReport(simu,probeAName);
        tmpEPa.setDefinition(alphaEqn);
        allEXPREports.add(tmpEPa);
        //Beta
        String probeBName = ptName +"Beta";
        String betaEqn = "atan2(-${"+ptName+"Velocity_jReport},mag(["
            + "${"+ptName+"Velocity_iReport},${"+ptName+"Velocity_jReport},${"
            +ptName+"Velocity_kReport}])+1.e-20)*180/${pi}";
        tmpEPb = ReportTool.getExpressionReport(simu, probeBName);
        tmpEPb.setDefinition(betaEqn);
        allEXPREports.add(tmpEPb);
        setupMonitorPlot(ptName,tmpEPa,tmpEPb);
      }
  }
  private void setupMonitorPlot(String ptName,ExpressionReport tmpA, ExpressionReport tmpB){
      //get one plot per point of Alpha and Beta
       Monitor rptM1 = MonitorTool.reportIterationMonitor(simu, tmpA, maxSteadySteps, 1, 1);
       StarUpdate starUp1 = rptM1.getStarUpdate();
       IterationUpdateFrequency itUpF1 = starUp1.getIterationUpdateFrequency();
        itUpF1.setIterations(1);
       Monitor rptM2 = MonitorTool.reportIterationMonitor(simu, tmpB, maxSteadySteps, 1, 1);
      StarUpdate starUp2 = rptM2.getStarUpdate();
       IterationUpdateFrequency itUpF2 = starUp2.getIterationUpdateFrequency();
        itUpF2.setIterations(1);
        
      MonitorPlot tmpPlot;
      String tmpName;
      tmpPlot = PlotTool.getMonitorPlot(simu, ptName, ptName);
      tmpName = ptName + " Plot";
      PlotTool.addDataSet(simu, tmpPlot, rptM1.getPresentationName(), ptName+ " Alpha");
      PlotTool.addDataSet(simu, tmpPlot, rptM2.getPresentationName(), ptName+ " Beta");
    
  }
  //
  //============================================
  //  COORDINATE SYSTEMS
  //============================================
  //
  private void setupMainCoordinateSystems(){
    // This method sets up all required coordinate systems in a simulation.
    // That is the Body, Inlet, and CAD Zero coordinate system.
    
    
    // Check for older versions of the M600 code where body coordinate system
    // used to have an underscore
    try{
      CartesianCoordinateSystem oldBody = (CartesianCoordinateSystem) labCsys.
        getCoordinateSystemManager().getCoordinateSystem("Body_Csys");
      oldBody.setPresentationName(bodyCsysName);
      CartesianCoordinateSystem oldInlet = (CartesianCoordinateSystem) oldBody.
        getLocalCoordinateSystemManager().getObject("Velocity_Inlet");
      oldInlet.setPresentationName(inletCsysName);
    }catch(NeoException e){
    }

    //Body Coordinate System
    simu.println("Setting up Main Coordinate Systems");
    bodyCsys = SimTool.getLabBasedCoordinate(simu, bodyCsysName);

    // Automatically adjust Body Coordinate System to be STAR-CCM+ Lab 
    double[] xBodyBasis=bodyCsys.getBasis0().toDoubleArray();
    double[] yBodyBasis=bodyCsys.getBasis1().toDoubleArray();
    double[] origBody = bodyCsys.getOrigin().getVector().toDoubleArray();

    // TODO: check if we can't always just reset the Body to CCM Lab basis
    if( (Math.abs(1.0-xBodyBasis[0]) > SMALL_EPS) ||
        (Math.abs(xBodyBasis[1]) > SMALL_EPS) ||
        (Math.abs(xBodyBasis[2]) > SMALL_EPS) ){
      simu.println("M600 MAIN CSYS: Aligning Body Coordinate to CCM Lab X.");
      bodyCsys.setBasis0(new DoubleVector(xOnlyAxis));
    }
    if( (Math.abs(yBodyBasis[0]) > SMALL_EPS) ||
        (Math.abs(1.0-yBodyBasis[1]) > SMALL_EPS) ||
        (Math.abs(yBodyBasis[2]) > SMALL_EPS) ){

      simu.println("M600 MAIN CSYS: Aligning Body Coordinate to CCM Lab Y.");
      bodyCsys.setBasis1(new DoubleVector(yOnlyAxis));
    }

    if( (Math.abs(origBody[0]) > SMALL_EPS) ||
        (Math.abs(origBody[1]) > SMALL_EPS) ||
        (Math.abs(origBody[2]) > SMALL_EPS) ){
        // set back to [0,0,0] wrt LAB
      bodyCsys.getOrigin().setCoordinate(prefUVec, prefUVec, prefUVec,
                                         new DoubleVector(zeroOrigin));
      simu.println("M600 MAIN CSYS: Setting Body origin to CCM Lab origin.");
    }

    // Need a zero angle rotation CAD system
    simu.println("M600 MAIN CSYS: Making zero angle rotation CAD system.");
    makeCADZeroBodyCSys();
    simu.println("M600 MAIN CSYS: Making a beta0 rotation CAD system.");
    SimTool.getLabBasedCoordinate(simu, "Beta_o");
    modifyCoordSys("Beta_o", xOnlyAxis, yOnlyAxis, zeroOrigin);
    
    // We do not modify the mesh angle Csys here 
    simu.println("M600 MAIN CSYS: Making mesh angle rotation system.");
    makeMeshAngleCSys();


    // Velocity Inlet Coordinate System
    inletCsys = SimTool.getNestedCoordinate(bodyCsys, inletCsysName);
    double[] xAeroBasis=inletCsys.getBasis0().toDoubleArray();
    double[] yAeroBasis=inletCsys.getBasis1().toDoubleArray();
    double[] origAero = bodyCsys.getOrigin().getVector().toDoubleArray();
    if( (Math.abs(1.0-xAeroBasis[0])>SMALL_EPS) ||
        (Math.abs(xAeroBasis[1])>SMALL_EPS) ||
        (Math.abs(xAeroBasis[2])>SMALL_EPS)){
      inletCsys.setBasis0(new DoubleVector(xOnlyAxis));
    }
    if( (Math.abs(yAeroBasis[0])>SMALL_EPS) ||
        (Math.abs(1.0-yAeroBasis[1])>SMALL_EPS) ||
        (Math.abs(yAeroBasis[2])>SMALL_EPS)){
      inletCsys.setBasis1(new DoubleVector(yOnlyAxis));
    }
    if( (Math.abs(origAero[0])>SMALL_EPS) ||
        (Math.abs(origAero[1])>SMALL_EPS) ||
        (Math.abs(origAero[2])>SMALL_EPS) ){
      inletCsys.getOrigin().setCoordinate(prefUVec, prefUVec, prefUVec, new DoubleVector(zeroOrigin));
    }
    simu.println("Inlet Csys = "+inletCsys.getPresentationName());
    
    // Under crosswind, we operate on the assumption that the STAR-CCM+
    // laboratory coordinate system and the motion
    // coordinate system have been rotated kitePhi degrees. 
    //
    // This means that the mesh remains aligned with the body under alpha
    // and beta variations. 
    //
    // In crosswind simulations, we introduce a new coordinate basis
    // called "Ground Lab" to help keep track of the real ground 
    // inertial coordinate system.
    //
    // Initially, "Ground Lab" is aligned with Lab and is rotated by
    // the thether roll to create the new "true" inertial coordinate
    // system. Everything in the MRF and boundary conditions will be
    // defined relative to this coordinate system. This is possible
    // because both "Ground Lab" and the STAR-CCM+ lab frame are inertial
    // frames of reference in a crosswind simulation.
    //
    this.groundCsys = SimTool.getLabBasedCoordinate(simu, "Ground Lab");
    ((CartesianCoordinateSystem) groundCsys).setBasis0(new DoubleVector(xOnlyAxis));
    ((CartesianCoordinateSystem) groundCsys).setBasis1(new DoubleVector(yOnlyAxis));
  }
  private void adjustGroundLabCoordinateSystem(){
    // Transform the XWind origin_r vector into the CCM Lab coordinates to get
    // the origin of the MRF Motion relative to an unperturbed Body coordinate
    // system
    SimTool.rotateCoordinateSystem(simu, kitePhi, xOnlyAxis, groundCsys, groundCsys);
    
    double[][] lab2ground = getR_X(-1.*kitePhi*Math.PI/180.);
    double[] r_BwrtGround = {om_origin_rx, om_origin_ry, om_origin_rz};
    double[] ccm_origin_r = SimTool.transformVector(lab2ground,r_BwrtGround);
    simu.println(""+ccm_origin_r[0]+","+ccm_origin_r[1]+","+ccm_origin_r[2]);

    // Put the groundCsys location on the origin of the MRF Motion.
    // Note that the CCM Lab and Body are collocated - we want to move the
    // origin away from the body, so the 'r' vector is negative.
    Units units_m = ((Units) simu.getUnitsManager().getObject("m"));
    groundCsys.getOrigin().setCoordinate(units_m, units_m, units_m,
        new DoubleVector(
            new double[] {-ccm_origin_r[0],
                          -ccm_origin_r[1],
                          -ccm_origin_r[2]}));
    
  }
  private void forceDefaultBodyCoordinateSystems(){
    String tmpStr;
    CartesianCoordinateSystem tmpCsys;  
    //=============================================================
    // COORDINATES THAT ARE INDEPENDENT TO CAD BODY RE-ORIENTATION
    //=============================================================
    // 
    tmpStr="Lab CAD Slat Axis";
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-3.7470027081099043E-16, 1.0, 8.326672684688675E-17},
                    new double[]{3.120006509907323E-32, -8.326672684688675E-17, 1.0},
                    new double[]{0.583948572992419, 0.0, -0.061646867115182746});

    tmpStr="Lab CAD Flap A1A2 Axis";
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.07949698177127244, 0.9933872435954825, 0.08283727513038741},
                    new double[]{ 0.006608024558261529, -0.08257328964298874, 0.9965630867381013},
                    new double[]{-0.9169, -5.9212, 0.1842});

    tmpStr= "Lab CAD Flap A3A6 Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{-0.8754, 0.0, 0.1415});

    tmpStr= "Lab CAD Flap A7A8 Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.07949698177127244, 0.9933872435954825, -0.08283727513038741},
                    new double[]{0.0071528492768224965, 0.08252967699171493, 0.9965629378834366},
                    new double[]{-1.102, 3.607, 0.377});

    tmpStr = "Lab CAD H Tail Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.051402545, 0.998678015, 0.0},
                    new double[]{0.998678015, -0.051402545, 0.0},
                    new double[]{-6.709, -0.151, 0.817});

    tmpStr = "Lab CAD Rudder Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{-7.432, -0.009, -0.0});
    
    
    // Meshing pipeline is set up so that surface meshes are performed before
    // any surface rotation for mesh subtraction. 
    // These coordinate systems are created in the CCM Lab basis because there
    // is no required rotation based on mesh Alpha and mesh Beta.
    tmpStr="Surf CAD Flap A1A2 Axis";
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
        new double[]{-0.07949698177127244, 0.9933872435954825, 0.08283727513038741},
        new double[]{ 0.006608024558261529, -0.08257328964298874, 0.9965630867381013},
        new double[]{-0.9169, -5.9212, 0.1842});

    tmpStr= "Surf CAD Flap A3A6 Axis";
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
        new double[]{0.0, 1.0, 0.0},
        new double[]{0.0, 0.0, 1.0},
        new double[]{-0.8754, 0.0, 0.1415});

    tmpStr= "Surf CAD Flap A7A8 Axis";
    tmpCsys = SimTool.getLabBasedCoordinate(simu, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
        new double[]{0.07949698177127244, 0.9933872435954825, -0.08283727513038741},
        new double[]{0.0071528492768224965, 0.08252967699171493, 0.9965629378834366},
        new double[]{-1.102, 3.607, 0.377});

    //============================================================
    // COORDINATES THAT NEED TO ROTATE WITH BODY CSYS ORIENTATION
    //============================================================
    // All of the Axes below need to be oriented with the Body Coordinate system
    // because they are used for post-processing Reports
    tmpStr="CAD Slat Axis";
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-3.7470027081099043E-16, 1.0, 8.326672684688675E-17},
                    new double[]{3.120006509907323E-32, -8.326672684688675E-17, 1.0},
                    new double[]{0.583948572992419, 0.0, -0.061646867115182746});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-3.7470027081099043E-16, 1.0, 8.326672684688675E-17},
                    new double[]{3.120006509907323E-32, -8.326672684688675E-17, 1.0},
                    new double[]{0.583948572992419, 0.0, -0.061646867115182746});

    tmpStr="CAD Flap A1A2 Axis";
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.07949698177127244, 0.9933872435954825, 0.08283727513038741},
                    new double[]{ 0.006608024558261529, -0.08257328964298874, 0.9965630867381013},
                    new double[]{-0.9169, -5.9212, 0.1842});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.07949698177127244, 0.9933872435954825, 0.08283727513038741},
                    new double[]{ 0.006608024558261529, -0.08257328964298874, 0.9965630867381013},
                    new double[]{-0.9169, -5.9212, 0.1842});

    tmpStr= "CAD Flap A3A6 Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{-0.8754, 0.0, 0.1415});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{-0.8754, 0.0, 0.1415});

    tmpStr= "CAD Flap A7A8 Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.07949698177127244, 0.9933872435954825, -0.08283727513038741},
                    new double[]{0.0071528492768224965, 0.08252967699171493, 0.9965629378834366},
                    new double[]{-1.102, 3.607, 0.377});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.07949698177127244, 0.9933872435954825, -0.08283727513038741},
                    new double[]{0.0071528492768224965, 0.08252967699171493, 0.9965629378834366},
                    new double[]{-1.102, 3.607, 0.377});


    tmpStr = "CAD H Tail Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.051402545, 0.998678015, 0.0},
                    new double[]{0.998678015, -0.051402545, 0.0},
                    new double[]{-6.709, -0.151, 0.817});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.051402545, 0.998678015, 0.0},
                    new double[]{0.998678015, -0.051402545, 0.0},
                    new double[]{-6.709, -0.151, 0.817});

    tmpStr = "CAD Rudder Axis";
    controlsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{-7.432, -0.009, -0.0});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.0, 0.0, 1.0},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{-7.432, -0.009, -0.0});

    //===================================
    // SETUP OF ROTOR COORD SYS
    //===================================
    //
    tmpStr = "Rotor 1 Lower";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, -3.639282991481, 1.597750999197});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, -3.639282991481, 1.597750999197});
    
    tmpStr = "Rotor 1 Upper";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, -3.639282991481, -1.216276674717});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, -3.639282991481, -1.216276674717});
    //2
    tmpStr = "Rotor 2 Lower";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, -1.213094330494, 1.597750999197});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, -1.213094330494, 1.597750999197});
    
    tmpStr = "Rotor 2 Upper";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, -1.213094330494, -1.216276674717});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, -1.213094330494, -1.216276674717});
    //3
    tmpStr = "Rotor 3 Lower";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, 1.213094330494, 1.597750999197});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, 1.213094330494, 1.597750999197});
    
    tmpStr = "Rotor 3 Upper";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, 1.213094330494, -1.216276674717});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, 1.213094330494, -1.216276674717});
    
    //4
    tmpStr = "Rotor 4 Lower";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, 3.639282991481, 1.597750999197});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.560650441211, 3.639282991481, 1.597750999197});
    
    tmpStr = "Rotor 4 Upper";
    rotorsCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, 3.639282991481, -1.216276674717});
    tmpCsys = SimTool.getNestedCoordinate(meshAngleCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{0.05233595624294346, 0.0, -0.9986295347545739},
                    new double[]{0.0, 1.0, 0.0},
                    new double[]{1.908401851641, 3.639282991481, -1.216276674717});

    //===================================
    // SETUP OF RADIATOR COORD SYS
    //===================================
    // lower row
    tmpStr = "Lower Radiators";
    radiatorCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.6385334249069631, 0.6715351524276049, -0.3759196780305833},
                    new double[]{0.5786956011093864, 0.7409726979140551, 0.3406917407314305},
                    new double[]{0.0,0.0,0.0});

    tmpStr = "Upper Radiators";
    radiatorCsysList.add(tmpStr);
    tmpCsys = SimTool.getNestedCoordinate(bodyCsys, tmpStr);
    SimTool.modifyLocalCoordinate(simu, tmpCsys,
                    new double[]{-0.660014223027085, 0.6575660990544325, 0.363301597541615},
                    new double[]{0.5760615716020966, 0.7533968578208542, -0.31709027160886916},
                    new double[]{0.0,0.0,0.0});

  }
  
  
  private void modifyCoordSys(String csysName, double[] xAxis , double[] yAxis, double[] axisOrigin){
      CartesianCoordinateSystem tmpCsys;

      double eps=1.e-6;

      if(csysName.startsWith("Surf")||csysName.startsWith("Lab")){
          tmpCsys = (CartesianCoordinateSystem) labCsys.getCoordinateSystemManager().getCoordinateSystem(csysName);
      }else if(!csysName.equals("Beta_o")){
          tmpCsys = (CartesianCoordinateSystem) bodyCsys.getLocalCoordinateSystemManager().getObject(csysName);
      }else{
          tmpCsys = (CartesianCoordinateSystem) labCsys.getCoordinateSystemManager().getCoordinateSystem(csysName);
      }

      double[] tmpXBasis=tmpCsys.getBasis0().toDoubleArray();
      double[] tmpYBasis=tmpCsys.getBasis1().toDoubleArray();
      double[] tmpOrigin=tmpCsys.getOrigin().getVector().toDoubleArray();

      if((Math.abs(xAxis[0]-tmpXBasis[0])>eps)||
         (Math.abs(xAxis[1]-tmpXBasis[1])>eps)||
         (Math.abs(xAxis[2]-tmpXBasis[2])>eps)){
          tmpCsys.setBasis0(new DoubleVector(xAxis));
      }
      if((Math.abs(yAxis[0]-tmpYBasis[0])>eps)||
         (Math.abs(yAxis[1]-tmpYBasis[1])>eps)||
         (Math.abs(yAxis[2]-tmpYBasis[2])>eps)){
          tmpCsys.setBasis1(new DoubleVector(yAxis));
      }
      if((Math.abs(tmpOrigin[0]-axisOrigin[0])>eps)||
         (Math.abs(tmpOrigin[1]-axisOrigin[1])>eps)||
         (Math.abs(tmpOrigin[2]-axisOrigin[2])>eps)){
          tmpCsys.getOrigin().setCoordinate(prefUVec, prefUVec, prefUVec, new DoubleVector(axisOrigin));
      }
  }
  private ArrayList<CartesianCoordinateSystem> getAllRotorCoords(){
      ArrayList<CartesianCoordinateSystem> retArr=new ArrayList();
      Collection<CoordinateSystem> allBodyCSYS = bodyCsys.getLocalCoordinateSystemManager().getObjects();
      for(CoordinateSystem tmp:allBodyCSYS){
          if(tmp instanceof CartesianCoordinateSystem){
              String coordName=tmp.getPresentationName();
              if(coordName.startsWith("Rotor")){
                  retArr.add((CartesianCoordinateSystem) tmp);
              }
          }
      }
      return retArr;
  }
  private CartesianCoordinateSystem makeCADZeroBodyCSys(){
      // Need a CAD Zero Body Coordinate System if we are running alpha/beta
      // cases, otherwise the mesh operations will all go out of date if the
      // body coordinate basis is rotated in alpha/beta
      zeroCADAngleBodyCsys =
              SimTool.getLabBasedCoordinate(simu, zeroCADAngleBodyCsysName);

      // need to check whether the CAD Zero System is out of alignment or not
      //  if it's in alignment, leave it alone
      double[] xBodyBasis=zeroCADAngleBodyCsys.getBasis0().toDoubleArray();
      double[] yBodyBasis=zeroCADAngleBodyCsys.getBasis1().toDoubleArray();
      if( (Math.abs(1.0-xBodyBasis[0])>SMALL_EPS) ||
          (Math.abs(xBodyBasis[1])>SMALL_EPS) ||
          (Math.abs(xBodyBasis[2])>SMALL_EPS) ){
        simu.println("M600 RUN: Aligning Zero CAD Csys to CCM Lab X.");
        zeroCADAngleBodyCsys.setBasis0(new DoubleVector(xOnlyAxis));

      }
      if( (Math.abs(yBodyBasis[0])>SMALL_EPS) ||
          (Math.abs(1.0-yBodyBasis[1])>SMALL_EPS) ||
          (Math.abs(yBodyBasis[2])>SMALL_EPS) ){
        simu.println("M600 RUN: Aligning Zero CAD Csys to CCM Lab Y.");
        zeroCADAngleBodyCsys.setBasis1(new DoubleVector(yOnlyAxis));
      }
      return zeroCADAngleBodyCsys;
  } 
  private CartesianCoordinateSystem makeMeshAngleCSys(){
      // This coordinate system only needs to exist.
      // Do not reset this coordinate system to lab.
      //
      // Conceptually, this coordinate system:
      //   Should be set to zero during a pre-processing setup.
      //   Should be modified during a mesh operation, where either:
      //     It is reset if alpha_0=beta_0=0 (or not specified)
      //     It is modified to the correct rotation angle (when specified)
      //   When solved for - not touched, but alpha_0 and beta_0 must be
      //   accounted for properly:
      //     kiteAlpha_0 and kiteBeta_0 should be tracked in the sim
      //   In post-processing, alpha_0 and beta_0 accounted for properly in
      //   calculating alpha.
      //   
      meshAngleCsys =
              SimTool.getLabBasedCoordinate(simu, meshAngleCsysName);

      // This coordinate system 
      return zeroCADAngleBodyCsys;
  }
  //
  //==================================================
  //  UTILITIES
  //==================================================
  private void setInletVectorAlphaBeta(){
    // Align velocity vector with body csys if not already aligned
    inletCsys.setBasis0(new DoubleVector(xOnlyAxis));
    inletCsys.setBasis1(new DoubleVector(yOnlyAxis));

    // Rotate velocity vector alpha
    // Recacll for inlet, you must counter rotate the inlet
    SimTool.rotateCoordinateSystem(simu, -kiteAlpha, yOnlyAxis, inletCsys, inletCsys);
    SimTool.rotateCoordinateSystem(simu,  kiteBeta, zOnlyAxis, inletCsys, inletCsys);

    // Rotate velocity vector 180 deg around z-axis
    SimTool.rotateCoordinateSystem(simu,  180.0, zOnlyAxis, inletCsys, inletCsys);
    SimTool.rotateCoordinateSystem(simu,  180.0, xOnlyAxis, inletCsys, inletCsys);
  }
  private void setBodyBasedCoordinateAlphaBeta(CartesianCoordinateSystem thisCsys){
      thisCsys.setBasis0(new DoubleVector(xOnlyAxis));
      thisCsys.setBasis1(new DoubleVector(yOnlyAxis));
      // Rotate velocity vector alpha
      if(Math.abs(kiteAlpha) > SMALL_EPS){
          bodyCsys.getLocalCoordinateSystemManager().rotateLocalCoordinateSystems(new NeoObjectVector(new Object[] {thisCsys}),
              new DoubleVector(yOnlyAxis),
              new NeoObjectVector(new Object[] {prefUVec, prefUVec, prefUVec}), -1.*kiteAlpha*Math.PI / 180., thisCsys);
      }
      // Rotate velocity vector beta
      if(Math.abs(kiteBeta) > SMALL_EPS){
          bodyCsys.getLocalCoordinateSystemManager().rotateLocalCoordinateSystems(new NeoObjectVector(new Object[] {thisCsys}),
              new DoubleVector(zOnlyAxis),
              new NeoObjectVector(new Object[] {prefUVec, prefUVec, prefUVec}), kiteBeta*Math.PI / 180., thisCsys);
      }
  }
  private void setWindAxisTransRotMotion(double om_x,double om_y,double om_z){
      RotatingAndTranslatingMotion rotAndTransMotion;
      try{
          rotAndTransMotion=
                  (RotatingAndTranslatingMotion) simu.get(MotionManager.class).getObject("Kite Wind Axis XWind");
      }catch(NeoException e){
          rotAndTransMotion=simu.get(MotionManager.class).createMotion(RotatingAndTranslatingMotion.class, "Kite Wind Axis XWind");
      }
      CartesianCoordinateSystem windAxis = 
        ((CartesianCoordinateSystem) bodyCsys.getLocalCoordinateSystemManager().getObject("Wind Axis"));
      rotAndTransMotion.setCoordinateSystem(windAxis);
      // Set Wind Axis Velocity Vector
      get_windVel_FF();
      rotAndTransMotion.getTranslationVelocity().setDefinition("$$wind_axis_velocity");
      rotAndTransMotion.getRotationRate().setDefinition("$omega_ramp");
      //rotAndTransMotion.getRotationRate().setValue(Math.sqrt(om_x*om_x+om_y*om_y+om_z*om_z));
      // Calculate proper omega vector

      simu.println("omega vector");
      simu.println(""+om_x+","+om_y+","+om_z);

      // need Body axis rotation matrix inverse
      double[][] bodyRot = SimTool.get3x3Inverse(SimTool.getBasisSet(bodyCsys));
      // need Wind axis rotation matrix inverse

      double[][] windR = SimTool.getBasisSet(windAxis);
      simu.println("Wind R");
      for (int i=0;i<=2;i++){
          simu.println(""+windR[i][0]+","+windR[i][1]+","+windR[i][2]);
      }



      double[][] windRot = SimTool.get3x3Inverse(SimTool.getBasisSet(windAxis));

      simu.println("Wind Inv");
      for (int i=0;i<=2;i++){
          simu.println(""+windRot[i][0]+","+windRot[i][1]+","+windRot[i][2]);
      }

      simu.println("Body");
      for (int i=0;i<=2;i++){
          simu.println(""+bodyRot[i][0]+","+bodyRot[i][1]+","+bodyRot[i][2]);
      }     
      // effect of 2 rotation matrices for new omega vector
      //multiply3x3Matrices(bodyRot,windRot)
      double[][] prod = SimTool.multiply3x3Matrices(bodyRot,windRot);
      simu.println("Product");
      for (int i=0;i<=2;i++){
          for(int j =0;j<=2;j++){
          simu.println(""+i+","+j+","+prod[i][j]);
          }
      }


      //double[] wind_om = transformVector(multiply3x3Matrices(bodyRot,windRot),
      double[] wind_om = SimTool.transformVector(windRot,
              new double[] {om_x,om_y,om_z});
      simu.println("wind omega vector");
      simu.println(""+wind_om[0]+","+wind_om[1]+","+wind_om[2]);
      rotAndTransMotion.getAxisDirection()
              .setComponents(wind_om[0],wind_om[1],wind_om[2]);
  }
  private boolean needRunXWind(double om_x,double om_y,double om_z){
      boolean retBool=false;

      if(Math.abs(om_x)>1.e-6){
          retBool=true;
      } else if(Math.abs(om_y)>1.e-6){
          retBool=true;
      } else if(Math.abs(om_z)>1.e-6){
          retBool=true;
      }
      return retBool;
  }
  //
  private String dblToFileStr(double angle){
      /* Method to tak a decimal and convert to legible file string, e.g.
           angle of +3.5 to p03p5
         Only intends 1 sig digit
      */
      int intPart=(int) angle;
      double fltPart=angle-intPart;
      String posNeg;
      String smAng="";
      // positive of negative angle
      if(angle<0){
          posNeg = "-";
      }else{
          posNeg = "";
      }
      //between (-10,10)?
      if(Math.floor(Math.abs(angle))<10){
          smAng = "0";
      }
      return (posNeg+smAng+Math.abs(intPart)+"p"+Math.abs(Math.round(fltPart*10)));
  }
  private String getRotorRPMString(String rotName){
    String retStr;
    retStr="_"+dblToFileStr(((RotatingMotion) simu.get(MotionManager.class).getObject(rotName))
      .getRotationRate().getRawValue());
    return retStr;
  }
  private Collection<Region> gatherBodyRegions(){
    Collection<Region> kiteBodyRegions = new ArrayList();
    //
    for(Region tmp:simu.getRegionManager().getRegions()){
      if(tmp.getPhysicsContinuum()!=null){
        boolean hasWalls = false; //only Regions with walls should contain a kite
        // if Region Reference Frame is Stationary and Motion not of type rotation
        boolean noMotion = tmp.getValues().get(MotionSpecification.class).getMotion() instanceof StationaryMotion;
        boolean noMRF = !(tmp.getValues().get(MotionSpecification.class).getReferenceFrame() instanceof RotatingReferenceFrame);
        //boolean xWindMRF = (tmp.getValues().get(MotionSpecification.class).getReferenceFrame().getPresentationName().equals("XwindMRF"));
        boolean xWindMRF = (tmp.getValues().get(MotionSpecification.class).getReferenceFrame().getPresentationName().contains("XWind MRF"));
        if(noMotion && (noMRF || xWindMRF)){
          //Test for wall boundaries
            for(Boundary tmpWall:tmp.getBoundaryManager().getBoundaries()){
            if(tmpWall.getBoundaryType() instanceof WallBoundary){
              hasWalls=true;
              break;
            }
          }
          if(hasWalls){
            kiteBodyRegions.add(tmp);
          }
        }
      }
    }
    simu.println("size of kiteBodyRegions ="+kiteBodyRegions.size());
    return kiteBodyRegions;
  }
  private Collection<Region> gatherRotatingBodyRegions(){
    Collection<Region> rotRegions = new ArrayList();
    for(Region tmp:simu.getRegionManager().getRegions()){
      if(!(tmp.getPhysicsContinuum()==null)){
        // if Region is of type Rotation, or if Regions is Stationary and Reference Frame is of type rotation
        if((tmp.getValues().get(MotionSpecification.class).getMotion() instanceof RotatingMotion)
            ||((tmp.getValues().get(MotionSpecification.class).getReferenceFrame() instanceof RotatingReferenceFrame)
               &&(tmp.getValues().get(MotionSpecification.class).getMotion() instanceof StationaryMotion))){
          rotRegions.add(tmp);
        }
      }
    }
    return rotRegions;
  }
  private double getRecommendedTimeStep(){
      double minPeriod=1e37;
      double rotRAD;
      double rotFREQ;
      double rotPER;
      double minTimeStep=unSteadyTimeStep;
      if(!allRotorRegs.isEmpty()){
          for(Region tmp:allRotorRegs){
              if(tmp.getValues().get(MotionSpecification.class).getMotion() instanceof RotatingMotion){
                  rotRAD=((RotatingMotion) tmp.getValues().get(MotionSpecification.class).getMotion()).getRotationRate().getSIValue();
                  rotFREQ=Math.abs(rotRAD/2/Math.PI); // frequency of rotation
                  rotPER = 1./rotFREQ;      // rotation period
                  if(rotPER<minPeriod){
                      minPeriod=rotPER;
                  }
              }else{
                  //simu.println("Region: " + tmp.getPresentationName() + " does not have a RBM.");
              }
          }
      }
      double minRotorTimeStep=minPeriod/propellerDiscretizationsPerTimeStep;
      double airTimeStep = charBaseSize/airSpeed;
      if(minTimeStep>minRotorTimeStep) minTimeStep=minRotorTimeStep;
      if(minTimeStep>airTimeStep) minTimeStep=airTimeStep;
      minTimeStep = Double.parseDouble(String.format("%."+2+"G", minTimeStep));
      return minTimeStep;
  }
  //
  private void makeAllReportProxiesVolumeMesh(){
    for(Report tmp:simu.getReportManager().getObjects()){
      if(tmp instanceof ForceCoefficientReport){
        ((ForceCoefficientReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
      if(tmp instanceof ForceReport){
        ((ForceReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
      if(tmp instanceof SumReport){
        ((SumReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
      if(tmp instanceof MomentCoefficientReport){
        ((MomentCoefficientReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
      if(tmp instanceof MomentReport){
        ((MomentReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
      if(tmp instanceof SurfaceIntegralReport){
        ((SurfaceIntegralReport) tmp).setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
      }
    }
  }
  private void makeAllSceneProxiesVolumeMesh(){
    for(Scene tmp:simu.getSceneManager().getObjects()){
      for(Displayer tmpDisp:tmp.getDisplayerManager().getObjects()){
        if(tmpDisp.getRepresentation()==simProxy){
          tmpDisp.setRepresentation(simu.getRepresentationManager().getFvRepresentation("Volume Mesh"));
        }
      }
    }
  }
  //
  // ==================================================
  // POST - PROCESSING EXPORT METHODS into CSV FILES
  // ==================================================
  private void postProcessMeanPitotTubeValuesToCSVFile(String csvFilePath,String csvFileName,int numSamp){
    SystemTool.touchDirectory(csvFilePath);
    // Method to output data to the CFD results file
    Writer writer = null;

    ArrayList<PitotProbe> allPitot = getAllPitotTubeObjects();

    if(allPitot.size()>0){
      try {
        writer = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(csvFilePath+csvFileName,true)
                  , "utf-8"));
        String lineStr="";
        for(int i=0;i<allPitot.size();i++){
          PitotProbe tmpPitot = allPitot.get(i);
          int lengthPitotName = tmpPitot.getName().length();
          String pitotName = tmpPitot.getName();
          if(i == 0){
            lineStr=lineStr+"Name, NumberOfSamples";
            for(Monitor tmpMon:tmpPitot.getMonitors()){
              String shortenedHeader = tmpMon.getPresentationName().substring(lengthPitotName+1);
              lineStr=lineStr+","+shortenedHeader+"_avg"+","+shortenedHeader+"_stdDev";
            }
            lineStr=lineStr+"\n";
          }
          lineStr=lineStr+tmpPitot.getName()+","+numSamp+","+tmpPitot.dataToCSV(simu,numSamp)+"\n";
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
  // methods for putting post-processed data into spreadsheet format
  private void writeOutToDataFile(String simOutputData,String nameDataFile,String directoryLocation){
    Writer writer = null;
    try {
      writer = new BufferedWriter(new OutputStreamWriter(
              new FileOutputStream(directoryLocation+File.separator+nameDataFile,true)
              , "utf-8"));
      writer.write(simOutputData);
      ((BufferedWriter) writer).newLine();
    } catch (IOException ex) {
      // report
    } finally {
       try {writer.close();} catch (Exception ex) {/*ignore*/}
    }
  }
  private String writeSimDataToSpreadsheetFormat(boolean isUnsteady,boolean needGRT,boolean needRotors,boolean needBEM){
      int numSamp=nDataSamples;
      String appEnd;
      String retStr;
      if(isUnsteady){
          appEnd=" - UNS";
      }else{
          appEnd=" - It";
      }

      // Case Description
      String caseDescription = caseName;
      caseDescription.replaceAll("_"," ");

      // CFD Information
      String cfdInformation = cfdInformationToCSVString(numSamp,appEnd);

      // Reference Values
      String refValues = referenceValuesToCSVString();

      // Flight Conditions
      String flightCondition = flightConditionCSVString();

      // Run summary
      String geometryConditions = geometryCSVString();

      //omega file string
      NumberFormat simpleform = new DecimalFormat("0.#E0");
      NumberFormat formatter = new DecimalFormat("0.##E0");
      double[] om_tmp = getXWindOmegaReports();
      String omegaString = "";
      omegaString = omegaString+formatter.format(om_tmp[0])
                           +","+formatter.format(om_tmp[1])
                           +","+formatter.format(om_tmp[2])
                           +","+formatter.format(om_tmp[3]);
      
      //radius file string
      double[] om_origin = getXWindOmegaOriginReports();
      
      String omegaOriginString = "";
      omegaOriginString = omegaOriginString+formatter.format(om_origin[0])
                                       +","+formatter.format(om_origin[1])
                                       +","+formatter.format(om_origin[2]);

      // Run performance metrics
      String performanceMetrics = performanceMetricsCSVString(numSamp,appEnd);

      // Kite total data
      String kiteTotalData=kiteTotalDataToSpreadsheetCSVString(numSamp,appEnd);
      String kiteOnlyData=kiteOnlyDataToSpreadsheetCSVString(numSamp,appEnd);
      String radiatorData=kiteRadiatorDataToSpreadsheetCSVString(numSamp,appEnd);

      // Slat data
      String slatData;
      slatData=generalObjDataToSpreadsheetCSVString(numSamp,"Slat",appEnd);

      // Main wing - note that this is the main elementm, not the region's
      // main wing boundary
      simu.println("main wing");
      String mainWingData;
      mainWingData=generalObjDataToSpreadsheetCSVString(numSamp,"MainWing",appEnd);

      // Flaps
      simu.println("flap data");
      String part41=flapDataToSpreadsheetCSVString(numSamp,appEnd,1);
      String part42=flapDataToSpreadsheetCSVString(numSamp,appEnd,2);
      String part43=flapDataToSpreadsheetCSVString(numSamp,appEnd,3);
      String part44=flapDataToSpreadsheetCSVString(numSamp,appEnd,4);
      String part45=flapDataToSpreadsheetCSVString(numSamp,appEnd,5);
      String part46=flapDataToSpreadsheetCSVString(numSamp,appEnd,6);
      String part47=flapDataToSpreadsheetCSVString(numSamp,appEnd,7);
      String part48=flapDataToSpreadsheetCSVString(numSamp,appEnd,8);
      String flapData = part41+part42+part43+part44
                       +part45+part46+part47+part48;

      // Main wing
      simu.println("fuselage");
      String fuselageData;
      fuselageData=generalObjDataToSpreadsheetCSVString(numSamp,"Fuselage",appEnd);

      //
      simu.println("vtail data");
      String vTailData=vTailDataToSpreadsheetCSVString(numSamp,appEnd);
      //
      simu.println("htail data");
      String hTailData=vTailDataToSpreadsheetCSVString(numSamp,appEnd);
      //
      simu.println("rudder data");
      String rudderData=rudderDataToSpreadsheetCSVString(numSamp,appEnd);
      //
      String part71=pylonDataToSpreadsheetCSVString(numSamp,appEnd,1);
      String part72=pylonDataToSpreadsheetCSVString(numSamp,appEnd,2);
      String part73=pylonDataToSpreadsheetCSVString(numSamp,appEnd,3);
      String part74=pylonDataToSpreadsheetCSVString(numSamp,appEnd,4);
      String pylonData= part71+part72+part73+part74;
      //
      simu.println("rotor data");
      // speeds - there are 8 rotors on the M600
      String rotorSpeeds="n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a";
      // data output - based on number of columns that are spit out
      String rotorData  ="n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a,n/a";
      rotorData=","+rotorData+","+rotorData+","+rotorData+","+rotorData+",";
      if(needRotors){
          String rtr1low=getRotorRPMString("Rotor 1 Lower");
          String rtr2low=getRotorRPMString("Rotor 2 Lower");
          String rtr3low=getRotorRPMString("Rotor 3 Lower");
          String rtr4low=getRotorRPMString("Rotor 4 Lower");
          String rtr1upp=getRotorRPMString("Rotor 1 Upper");
          String rtr2upp=getRotorRPMString("Rotor 2 Upper");
          String rtr3upp=getRotorRPMString("Rotor 3 Upper");
          String rtr4upp=getRotorRPMString("Rotor 4 Upper");
          rotorSpeeds=rtr1low.replaceAll("p", ".")+","+rtr2low.replaceAll("p", ".")
                  +","+rtr3low.replaceAll("p", ".")+","+rtr4low.replaceAll("p", ".")
                  +","+rtr1upp.replaceAll("p", ".")+","+rtr2upp.replaceAll("p", ".")
                  +","+rtr3upp.replaceAll("p", ".")+","+rtr4upp.replaceAll("p", ".");

          String part81=rotorDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,1);
          String part82=rotorDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,2);
          String part83=rotorDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,3);
          String part84=rotorDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,4);
          String part85=rotorDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,1);
          String part86=rotorDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,2);
          String part87=rotorDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,3);
          String part88=rotorDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,4);
          rotorData= part81+part82+part83+part84
                    +part85+part86+part87+part88;
      }else if(needBEM){
          String rtr1low=getRotorRPMString("Rotor 1 Lower");
          String rtr2low=getRotorRPMString("Rotor 2 Lower");
          String rtr3low=getRotorRPMString("Rotor 3 Lower");
          String rtr4low=getRotorRPMString("Rotor 4 Lower");
          String rtr1upp=getRotorRPMString("Rotor 1 Upper");
          String rtr2upp=getRotorRPMString("Rotor 2 Upper");
          String rtr3upp=getRotorRPMString("Rotor 3 Upper");
          String rtr4upp=getRotorRPMString("Rotor 4 Upper");
          rotorSpeeds=rtr1low.replaceAll("p", ".")+","+rtr2low.replaceAll("p", ".")
                  +","+rtr3low.replaceAll("p", ".")+","+rtr4low.replaceAll("p", ".")
                  +","+rtr1upp.replaceAll("p", ".")+","+rtr2upp.replaceAll("p", ".")
                  +","+rtr3upp.replaceAll("p", ".")+","+rtr4upp.replaceAll("p", ".");
          String part81=bemDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,1);
          String part82=bemDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,2);
          String part83=bemDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,3);
          String part84=bemDataToSpreadsheetCSVString(numSamp,"Lower",appEnd,4);
          String part85=bemDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,1);
          String part86=bemDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,2);
          String part87=bemDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,3);
          String part88=bemDataToSpreadsheetCSVString(numSamp,"Upper",appEnd,4);
          rotorData= part81+part82+part83+part84
                    +part85+part86+part87+part88;
      }

      String tmpGRT = ""; if(needGRT) tmpGRT=" GRT"; String justGRT=tmpGRT;

      //rotor file string
      String needRtr= ""; if(needRotors) needRtr="Rotors "; if(!needRotors) needRtr="No Rotors ";
      
      // unsteady simulation tracker
      String tmpUNS = ""; if(isUnsteady) tmpUNS="Unsteady ";
      
      // FINAL STRING compile
      String oldCaseDescription = "M600_"+tmpUNS+needRtr+"kwSST"+tmpGRT;

      // Compile Spreadheet CSV
      retStr= caseDescription + "," + studyName + "," 
              + cfdInformation + "," + getOldCaseName()+ "," + oldCaseDescription + "," + refValues + "," 
              + flightCondition + ","
              + geometryConditions + "," + omegaString + "," + omegaOriginString + "," + performanceMetrics + ","
              + rotorSpeeds + "," 
              +kiteTotalData+kiteOnlyData+radiatorData+
              slatData+mainWingData+flapData+fuselageData+
              hTailData+vTailData+rudderData+pylonData+rotorData;

      simu.println("CSV Data Line write to: " + m600PostProcFolder+File.separator+"M600_RUNNER_RESULTS.csv");
      simu.println(retStr);
      simu.println(" ");
      SystemTool.touchDirectory(m600PostProcFolder);
      writeOutToDataFile(retStr,"M600_RUNNER_RESULTS.csv",m600PostProcFolder);
      return retStr;
  }
  private String cfdInformationToCSVString(int numSamples, String appEnd){
    String retStr = "";

    // Runner / CCM Version
    String ccmHeader = javaScriptVersion + "," + ccmVersion;

    // Steady or Unsteady
    String steadyStatus = "Steady";
    if(CFD_Physics.isUnsteady(allUsedFluidPhysics)){
      steadyStatus = "Unsteady";
    }

    // Case Total Iterations
    int startIt = ((int) getCaseStartIterationParameter().getQuantity().getInternalValue());
    int endIt =  simu.getSimulationIterator().getCurrentIteration();
    int totalIt = endIt-startIt;

    //Seconds per Iteration
    String monName="sec/it"+appEnd;
    double secIt_ave = MonitorTool.getLastMonitorSamplesMean(simu,monName,numSamples);
    double secIt_stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,monName,numSamples,secIt_ave);

    //Total Cell Count
    long bodyCellCount = VolumeMeshTool.getVolumeMeshCount(simu,allBodyRegs);
    long rotorCellCount = VolumeMeshTool.getVolumeMeshCount(simu,allRotorRegs);
    long cellCount = bodyCellCount+rotorCellCount;

    //Return String
    retStr = retStr + ccmHeader + "," 
      + steadyStatus + ","
      + totalIt + "," + secIt_ave + "," +secIt_stdDev + ","
      + cellCount;

    return retStr;

  }
  private String referenceValuesToCSVString(){
    String retStr="";
    retStr=retStr+nominalChord+","+wingSpanNorm+","+referenceArea + "," + nDataSamples;
    return retStr;
  }
  private String flightConditionCSVString(){
    String retStr="";
    double repDensity = SimTool.getScalarSimulationParameter(simu, "Case Density").getQuantity().getInternalValue();
    double repAirspeed = SimTool.getScalarSimulationParameter(simu, "Case Airspeed").getQuantity().getInternalValue();
    double reNumber = repDensity*repAirspeed*nominalChord/airViscosity;
    simu.println("Flight Conditions");
    simu.println("Re #    ");
    simu.println("Density "+repDensity);
    simu.println("Speed   "+repAirspeed);
    simu.println("Mu      "+airViscosity);
    retStr=retStr+reNumber+","+repDensity+","+repAirspeed+","+airViscosity;
    return retStr;
  }
  private String geometryCSVString(){
      simu.println("Geometry CSV data");
      // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      double alpha0=getRotationAngle("Kite Rotate Alpha0");
      double beta0=getRotationAngle("Kite Rotate Alpha0");
      double flap1=getRotationAngle("Kite Rotate Flap A1");
      double flap2=getRotationAngle("Kite Rotate Flap A2");
      double flap3=getRotationAngle("Kite Rotate Flap A3");
      double flap4=getRotationAngle("Kite Rotate Flap A4");
      double flap5=getRotationAngle("Kite Rotate Flap A5");
      double flap6=getRotationAngle("Kite Rotate Flap A6");
      double flap7=getRotationAngle("Kite Rotate Flap A7");
      double flap8=getRotationAngle("Kite Rotate Flap A8");
      double hTailSet=getRotationAngle("Kite Rotate H Tail");
      double rudderSet=getRotationAngle("Kite Rotate Rudder");

      double repAlpha = getKiteAlpha();
      double repBeta = getKiteBeta();

      retStr=retStr+alpha0+","+beta0+","+repAlpha+","+repBeta+
              ","+flap1+","+flap2+","+flap3+","+flap4+","+flap5+","+flap6+","+
              flap7+","+flap8+","+hTailSet+","+rudderSet;
      return retStr;
  }
  private String performanceMetricsCSVString(int numSamp,String appEnd){
    simu.println("Performance Metrics Start");
    String tmpName;
    String retStr = "";
    // CL
    tmpName="Aero Kite Total Cl"+appEnd;
    double CL_ave = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    // CD
    tmpName="Aero Kite Total Cd"+appEnd;
    double CD_ave = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);

    double CL_over_CD = CL_ave/CD_ave;
    double zetaTerm = Math.pow(CL_ave,3.0)/(Math.pow(CD_ave+systemCD0,2.0));

    retStr=retStr + CL_over_CD + "," + systemCD0 + "," + zetaTerm;

    return retStr;

  }
  private String kiteTotalDataToSpreadsheetCSVString(int numSamp,String appEnd){
      simu.println("Write Kite Total Data to Spreadsheet ");
      // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";
      /*        
      tmpName="";
      aveVal = getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      */        
      //
      tmpName="Body Kite Total CX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+aveVal+","+stdDev;
      //
      tmpName="Body Kite Total CY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite Total CZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite Total CmX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite Total CmY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;        
      //
      tmpName="Body Kite Total CmZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;      
      //
      tmpName="Aero Kite Total Cl"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Total Cd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Total Cy"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Total CmX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Total CmY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Total CmZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;


      return retStr;
  }
  private String kiteOnlyDataToSpreadsheetCSVString(int numSamp, String appEnd){
    simu.println("Write Kite ONLY Data to Spreadsheet ");  
    // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      */     

      tmpName="Body Kite CX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite CY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite CZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite CmX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Body Kite CmY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;        
      //
      tmpName="Body Kite CmZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;      
      //
      tmpName="Aero Kite Cl"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Cd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite Cy"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite CmX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite CmY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //
      tmpName="Aero Kite CmZ"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      return retStr;
  }
  private String kiteRadiatorDataToSpreadsheetCSVString(int numSamp,String appEnd){
      simu.println("Write Kite Radiator Data to Spreadsheet ");
      // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      */
      tmpName="Aero Radiator 1 Lower Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 1 Lower Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 2 Lower Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 2 Lower Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 3 Lower Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 3 Lower Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 4 Lower Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 4 Lower Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      tmpName="Aero Radiator 1 Upper Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 1 Upper Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 2 Upper Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 2 Upper Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 3 Upper Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 3 Upper Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Aero Radiator 4 Upper Fd"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      tmpName="Radiator 4 Upper Performance"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp)/airSpeed;
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      return retStr;
  }
  private String flapDataToSpreadsheetCSVString(int numSamp, String appEnd, int flapNum){
    simu.println("Write Flap Data to Spreadsheet ");  
    // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";
      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      */
      // Flap numStr
      try{
        tmpName="Body Flap A"+flapNum+" CX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body Flap A"+flapNum+" CY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body Flap A"+flapNum+" CZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body Flap A"+flapNum+" CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body Flap A"+flapNum+" CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body Flap A"+flapNum+" CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" Cl"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" Cd"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" Cy"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero Flap A"+flapNum+" CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      
      try{
        tmpName="Local Flap A"+flapNum+" Moment"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      return retStr;
  }
  private String vTailDataToSpreadsheetCSVString(int numSamp, String appEnd){
    simu.println("Write vTail Data to Spreadsheet ");  
    // Method to output data to the CFD results file
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      String surfaceName = "V Tail";
      try{
        tmpName="Body " + surfaceName + " CX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cl"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cd"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cy"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      return retStr;
  }
  private String hTailDataToSpreadsheetCSVString(int numSamp, String appEnd){
    simu.println("Write hTail Data to Spreadsheet ");  
    // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";
      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;  
      */

      String surfaceName = "H Tail";
      try{
        tmpName="Body " + surfaceName + " CX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cl"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cd"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cy"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      // local moment
      try{
        tmpName="Local " + surfaceName +" Moment"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev; 
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      return retStr;
  }
  private String rudderDataToSpreadsheetCSVString(int numSamp, String appEnd){
      simu.println("Write Rudder Data to Spreadsheet ");
      // Method to output data to the CFD Master Results File
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      String surfaceName = "Rudder";
      try{
        tmpName="Body " + surfaceName + " CX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cl"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cd"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cy"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      // local moment
      try{
        tmpName="Local " + surfaceName +" Moment"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev; 
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      return retStr;
  }
  private String pylonDataToSpreadsheetCSVString(int numSamp, String appEnd,int pylonNum){
      simu.println("Write Pylon Data to Spreadsheet ");
      // Method to output data to the CFD results file
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";

      String surfaceName = "Pylon "+pylonNum;
      try{
        tmpName="Body " + surfaceName + " CX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Body " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cl"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cd"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " Cy"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmX"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmY"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }
      try{
        tmpName="Aero " + surfaceName + " CmZ"+appEnd;
        aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
        stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
        retStr = retStr+","+aveVal+","+stdDev;
      }catch(NeoException e){
        retStr = retStr+","+"na"+","+"na";
      }

      
      return retStr;
  }
  private String rotorDataToSpreadsheetCSVString(int numSamp, String rowName,String appEnd,int rotorNum){
      simu.println("Write rotor Data to Spreadsheet ");
      // Method to output data to the CFD master file
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";
      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;  
      */

      //FX
      tmpName="Rotor "+rotorNum+" "+rowName+" FX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //FY
      tmpName="Rotor "+rotorNum+" "+rowName+" FY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;        
      //FZ
      tmpName="Rotor "+rotorNum+" "+rowName+" Thrust"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //MX
      tmpName="Rotor "+rotorNum+" "+rowName+" MX"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //MY
      tmpName="Rotor "+rotorNum+" "+rowName+" MY"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //MZ
      tmpName="Rotor "+rotorNum+" "+rowName+" Torque"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      return retStr;
  }
  private String bemDataToSpreadsheetCSVString(int numSamp, String rowName,String appEnd,int rotorNum){
    simu.println("Write bem Data to Spreadsheet ");  
    // Method to output data to the CFD results file
      double aveVal;
      double stdDev;
      String tmpName;
      String retStr="";
      /*        
      tmpName="";
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;  
      */

      //FX
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" X-Axis Force"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //FY
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" Y-Axis Force"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;        
      //FZ
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" Z-Axis Force"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //MX
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" X-Axis Moment"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      //MY
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" Y-Axis Moment"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;
      //MZ
      tmpName="BEM Rotor "+rotorNum+" "+rowName+" Z-Axis Moment"+appEnd;
      aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
      stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
      retStr = retStr+","+aveVal+","+stdDev;

      return retStr;
  }
  private String generalObjDataToSpreadsheetCSVString(int numSamp, String objName, String appEnd){
      simu.println("Write general object Data to Spreadsheet ");
// Method to output data to the CFD results file
  double aveVal;
  double stdDev;
  String tmpName;
  String retStr="";

  // body
  try{
    tmpName="Body "+objName+" CX"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  try{
    tmpName="Body "+objName+" CY"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  try{
    tmpName="Body "+objName+" CZ"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  // moment coefficient reports, if available (added 1v04d feature)
  try{
    tmpName="Body "+objName+" CmX"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  // moment coefficient reports, if available (added 1v04d feature)
  try{
    tmpName="Body "+objName+" CmY"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  // moment coefficient reports, if available (added 1v04d feature)
  try{
    tmpName="Body "+objName+" CmZ"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  
try{
    tmpName="Aero "+objName+" Cl"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  try{
    tmpName="Aero "+objName+" Cd"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
  try{
    tmpName="Aero "+objName+" Cy"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+"n/a"+","+"n/a";
  }
 
  try{
    tmpName="Aero "+objName+" CmX"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+" "+","+" ";
  }
  try{
    tmpName="Aero "+objName+" CmY"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+" "+","+" ";
  }
  try{
    tmpName="Aero "+objName+" CmZ"+appEnd;
    aveVal = MonitorTool.getLastMonitorSamplesMean(simu,tmpName,numSamp);
    stdDev = MonitorTool.getLastMonitorSamplesStDev(simu,tmpName,numSamp,aveVal);
    retStr = retStr+","+aveVal+","+stdDev;
  }catch(NeoException e){
    retStr = retStr+","+" "+","+" ";
  }

  return retStr;
}

  //============================================
  // M600 RUNNER INPUT CONTROL METHODS
  //============================================
  //read file & assign variables
  private void readInputFile(){
    ArrayList<String> data = new ArrayList();
    ArrayList<String> vars = new ArrayList();
    String mdRelativePath = "";
    mdRelativePath = ".."+File.separator+".."+File.separator+".."+File.separator+".."+File.separator;
    try{
      String inputFileStr=simPathName+File.separator+"m600_input.cfd";
      String multiDesignFileStr=simPathName+File.separator+mdRelativePath+"m600_input.cfd";
      File localMXInput = new File(inputFileStr);
      File mdInput = new File(multiDesignFileStr);
      Scanner scan;
      if(localMXInput.exists()){
        scan = new Scanner(localMXInput); 
      }else{
        scan = new Scanner(mdInput); 
          } 

      while (scan.hasNext()){
        String line = scan.nextLine();
        if(readerIsCommentLine(line)){
        } else{
          line=line.replaceAll("\\s","");
          String[] parts = line.split("=");

          String tmp = null;
          String p2  = null;
          boolean b  = true;
          int i = parts[1].indexOf("/"); // note: that this line requires/expects a comment section after the variable
          if(i!=-1){
            tmp = parts[1];
            p2= tmp.substring(0, i);
            if(p2.equals("true")||p2=="false"){
               data.add(p2);
            } else if(p2.startsWith("{")){
              p2=p2.replaceAll("\\{", "");
              p2=p2.replaceAll("\\}", "");

              // this line makes it expect a comma
              String[] p3 = new String[1];
              if(p2.contains(",")){
                p3= p2.split(",");
              }else{
                p3[0]=p2;
              }
              if(p3[0].matches("false")||p3[0].contains("true")){
                //This part converts it to boolean value when read in
                //move this to the case booleans
                boolean[] b2= new boolean[p3.length];
                for(int a=0;a<p3.length;a++){
                  b2[a] = Boolean.parseBoolean(p3[a]);
                }
                String f1 = Arrays.toString(b2).substring(1, Arrays.toString(b2).length()-1);
                f1 = "{"+f1+"}";
                data.add(f1);

              }else if(p2.startsWith("\"")){
                if(parts[0].matches("newKitePartFileName")){
                  String s1= p2;
                  data.add(s1);
                }else{
                data.add(p2);
                }
              }else if(parts[0].equals("maxSteadySteps")||parts[0].equals("numHighViscItSteps")){
                int[] i2= new int[p3.length];
                data.add(i2.toString());

              }
          else {
                double[] p3a= new double[p3.length];
                for(int c=0; c<p3.length; c++){
                    p3a[c]=Double.parseDouble(p3[c]);
                }
                String f2 = Arrays.toString(p3a).substring(1,Arrays.toString(p3a).length()-1);
                f2 = "{"+f2+"}";
                data.add(f2);
              }
            }else{
              data.add(p2);
            }
          }
          vars.add(parts[0]);
        }
      }

    } catch (FileNotFoundException e) {  
      simu.println("ERROR: There is no m600_input.cfd file located in the study directory!");
    }
    setVariables(vars,data);
  }
  private boolean readerIsCommentLine(String currentLine){
    return !(currentLine.contains("="))||currentLine.isEmpty()||currentLine.startsWith("\\//")
          ||(currentLine.contains("=")&&currentLine.startsWith("\\//"))
          || currentLine.indexOf("//")==0;
  }
  private void setVariables(ArrayList<String> vars, ArrayList<String> data){
      boolean fileReaderVerbose = false;
      String a1=null;

      for(int j =0;j<vars.size();j++){
        String variable=vars.get(j);
        switch(variable)
        {

          case "newKitePartFileName":
              this.newKitePartFileName = (String) data.get(j).toString();
              break;
          case "autoSimStep":
              this.autoSimStep = Boolean.parseBoolean(data.get(j));
              break;    
          case "needRunPreProc":
              this.needRunPreProc = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunMesh":
              this.needRunMesh = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunSolver":
              this.needRunSolver = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunPost":
              this.needRunPost = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunQuickPost":
              this.needQuickPost = Boolean.parseBoolean(data.get(j));
              break;    
          case "needStarView":
              this.needStarView = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunRotors" :
              this.needRunRotors= Boolean.parseBoolean(data.get(j));
              break;     
          case "needRunBEM" :
              this.needRunBEM= Boolean.parseBoolean(data.get(j));
              break;     
          case "needRunUnsteady" :
              this.needRunUnsteady= Boolean.parseBoolean(data.get(j));
              break;         
          case "needClearSolution" :
              this.needClearSolution= Boolean.parseBoolean(data.get(j));
              break;     
          case "needHighViscosityInit" :
              this.needHighViscosityInit= Boolean.parseBoolean(data.get(j));
              break;     
          case "needChangeMaxSteadySteps" :
              this.needChangeMaxSteadySteps= Boolean.parseBoolean(data.get(j));
              break;        
          case "maxSteadySteps":
              this.maxSteadySteps = Integer.valueOf((String) data.get(j));
              break;
          case "numHighViscItSteps":
              this.numHighViscItSteps = Integer.valueOf((String) data.get(j));
              break;    
          case "kiteAlpha_o":
              this.kiteAlpha_o = Double.valueOf((String) data.get(j));
              break;    
          case "kiteBeta_o":
              this.kiteBeta_o = Double.valueOf((String) data.get(j));
              break;  
          case "baseSize":
              this.charBaseSize = Double.valueOf((String) data.get(j));
              break;
          case "kiteAlpha":
              this.kiteAlpha = Double.valueOf((String) data.get(j));
              break;      
          case "kiteBeta":
              this.kiteBeta = Double.valueOf((String) data.get(j));
              break;
          case "airSpeed":
              this.airSpeed = Double.valueOf((String) data.get(j));
              break;    
          case "airDensity":
              this.airDensity = Double.valueOf((String) data.get(j));
              break;  
          case "flapA1_angle":
              this.flapA1_angle = Double.valueOf((String) data.get(j));
              break;      
          case "flapA2_angle":
              this.flapA2_angle = Double.valueOf((String) data.get(j));
              break;   
          case "flapA3_angle":
              this.flapA3_angle = Double.valueOf((String) data.get(j));
              break;    
          case "flapA4_angle":
              this.flapA4_angle = Double.valueOf((String) data.get(j));
              break;  
          case "flapA5_angle":
              this.flapA5_angle = Double.valueOf((String) data.get(j));
              break;      
          case "flapA6_angle":
              this.flapA6_angle = Double.valueOf((String) data.get(j));
              break;   
          case "flapA7_angle":
              this.flapA7_angle = Double.valueOf((String) data.get(j));
              break;    
          case "flapA8_angle":
              this.flapA8_angle = Double.valueOf((String) data.get(j));
              break;  
          case "htail_angle":
              this.htail_angle = Double.valueOf((String) data.get(j));
              break;      
          case "rudder_angle":
              this.rudder_angle = Double.valueOf((String) data.get(j));
              break;   
          case "omega_hat_x":
              this.omega_hat_x = Double.valueOf((String) data.get(j));
              break;    
          case "omega_hat_y":
              this.omega_hat_y = Double.valueOf((String) data.get(j));
              break;  
          case "omega_hat_z":
              this.omega_hat_z = Double.valueOf((String) data.get(j));
              break;
          case "om_origin_rx":
              this.om_origin_rx = Double.valueOf((String) data.get(j));
              break;
          case "om_origin_ry":
              this.om_origin_ry = Double.valueOf((String) data.get(j));
              break;
          case "om_origin_rz":
              this.om_origin_rz = Double.valueOf((String) data.get(j));
              break;
          case "rtr_1_low":
              this.rtr_1_low = Double.valueOf((String) data.get(j));
              break;   
          case "rtr_2_low":
              this.rtr_2_low = Double.valueOf((String) data.get(j));
              break;    
          case "rtr_3_low":
              this.rtr_3_low = Double.valueOf((String) data.get(j));
              break;  
          case "rtr_4_low":
              this.rtr_4_low = Double.valueOf((String) data.get(j));
              break;      
          case "rtr_1_upp":
              this.rtr_1_upp = Double.valueOf((String) data.get(j));
              break;       
          case "rtr_2_upp":
              this.rtr_2_upp = Double.valueOf((String) data.get(j));
              break;    
          case "rtr_3_upp":
              this.rtr_3_upp = Double.valueOf((String) data.get(j));
              break;    
          case "rtr_4_upp":
              this.rtr_4_upp = Double.valueOf((String) data.get(j));
              break;        
        default:
              simu.println("M600 File Reader. Variable for string "
                      + variable + " not found");
        }
      }
      if(fileReaderVerbose){
        simu.println("***************************");
        for(int d=0; d<vars.size();d++){
          simu.println(vars.get(d)+" == "+data.get(d));
        }
        simu.println("***************************");
      }
    }
  
// END OF M600 RUNNER
} // END OF M600 RUNNER
