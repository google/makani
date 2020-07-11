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
  This code automates aerodynamic analyses in STAR-CCM+ release: 14.06.013

Framework Design Philosophy:
  1) Object-oriented code
  2) Respect a CFD engineer's ability to add non-standard settings
  2) Adhere to existing STAR-CCM+ functionality wherever possible

Framework Benefits:
  1) Save engineering time: automate common, repeated, tedious tasks
    *Automatically adhere to Best-Known-Practices
  2) Future-Proof Simulation Results:
    *Remove user setup errors or "oh I forgots"
    *Adapt mindset of only running a case once

*/

import AeroCFDRunner.WindTunnelAnalysis;
import AeroCFDRunner.RotorAnalysis;
import java.util.*;
import java.io.*;

//
import star.common.*;
import star.base.neo.*;
//
import Naming.*;
import Tools.SimTool;
import Tools.SystemTool;

public class Aero_Runner_v2p01 extends StarMacro {
//====================================
// SCROLL DOWN TO execute()
//====================================
private final double SMALL_EPS = 1.0E-6;
private final NamingConvention globalNames = new NamingConvention();
private final boolean _input_debug = false;

// Simulation based information
private Simulation simu;
private String simPathName;

// STAR-CCM+ case based information
private String caseName;
private String simOutputFolder;
private String studyName = "Local"; // Local is the default

private String mutliDesignInputFilePath = ""; // File that DM reads to drive a sim
private String mutliDesignOutputFolder = ""; // Where DM puts output
private String mutliDesignProjectName =""; // What DM .dmprj file is called

// Aero Runner Output post processing names
private String aeroFolderName = "";
private String aeroPostProcFolder = "AERO_POST";
private final String aeroWTPostProcCSVFile = "AERO_WT_Sweep_Results.csv";

//Version Control
private String ccmVersion;
private String javaScriptVersion;

// Solver & Stopping Criteria
private boolean autoSimStep=true;
private int maxSteadySteps=1000;
private double unSteadyTimeStep = 2e-3;
private int unSteadyInnerIterations=20;
private int unSteadyMaxSteps = 10;
private double timeDiscretizations=40;

// Postprocessing defaults
private int[] outputRes={2000,1000};

// Overall simulation selection tools
private boolean isMultiDesignCase  =false;
private boolean needPreProc        =true;
private boolean needRunMesh        =false;
private boolean needRunSolver      =false;
private boolean needClearSolution  =false;
private boolean needUnsteady       =false;
private boolean needRunPostProc    =false;
private boolean needRunQuickPost   =false;
private boolean turnOnAutoTimeStep =false;

// Windtunnel Analysis
private boolean needRun2D          =false;
private boolean needRunWindTunnel  =true;

private boolean needRotorAnalysis  =false;
private boolean needAlphaSweep     =false;

// Coordinate system stuff
private LabCoordinateSystem labCsys;
private CartesianCoordinateSystem bodyCsys;

private double chordLength=0.0; // [m] chord length
private double defaultLengthScale=1.0;

private double refRho=1.176;
private double refVel=10.0;
private double refTemp=300.0;
private double refMu=1e-5;
private double refMa=0.3;
private double refRe=1.E6;
private double refArea=0.0;
private double defaultReferenceArea=1.0;

private double fsPressure = 0.0; //[Pa] gauge pressure
private double[] refMomentRadius;
private double defaultReferenceMomentRadius=1.0;

//Airfoil Meshing
//Element Arrays of Size N
private ArrayList<Boolean> af_CustomMesh   = new ArrayList();

// General Mesh Settings
private ArrayList<Boolean> af_LowYPlus     = new ArrayList(); // currently not used
private ArrayList<Boolean> af_HighYPlus    = new ArrayList(); // currently not used
private ArrayList<Double>  cust_TargetSize   = new ArrayList();
private ArrayList<Double>  cust_MinSize      = new ArrayList();
private ArrayList<Double>  cust_NPtsOnCircle = new ArrayList();
private ArrayList<Double>  cust_PrismAbsThick= new ArrayList();
private ArrayList<Double>  cust_1stCellThick = new ArrayList();
private ArrayList<Integer> cust_NPrisms      = new ArrayList();

// Physics
double highMachNumber=0.4;
private String physicsName="Air";
private String physicsModel="kwSST";

// Meshing & Position Variables
private double    newAngleOfAttack;
private double    newSideSlipAngle;
private double[]  alphaSweepAngles;
private double[]  betaSweepAngles;
// Wind tunnel custom values.
private String[]  wtCustomReferenceNames;
private double[]  wtCustChordLengths;
private double[]  wtCustReferenceAreas;
private double[]  wtCustReferenceMomR;
private ArrayList<String> wtCustomReferenceNamesArray = new ArrayList();
private ArrayList<Double> wtCustChordLengthArray = new ArrayList();
private ArrayList<Double> wtCustReferenceAreaArray = new ArrayList();
private ArrayList<double[]> wtCustReferenceMomRDoubleArray = new ArrayList();

// Airfoil custom values.
private double[]  newAF_CADAngles;
private boolean[] newAF_CustomMesh;
private String[]  newRTR_SpeedName;
private double[]  newRTR_Speed;
private boolean   forceRTR_MRF;
private String[]  custRTR_MeshName;
private double[]  newCust_TargetSize;
private double[]  newCust_MinSize;
private double[]  newCust_NPtsOnCircle;
private double[]  newCust_PrismAbsThick;
private double[]  newCust_1stCellThick;
private int[]     newCust_NPrismLayers;  

int maxNumSteps=5000;

@Override
public void execute(){
  
// Familiarize w/ Simulation environment
this.simu = getActiveSimulation();
this.ccmVersion = this.simu.getStarVersion().getString("ReleaseNumber");

// Get file system based names
this.simPathName = this.simu.getSessionDir();
this.caseName = this.simu.getPresentationName();
this.simOutputFolder = "POST_" + this.caseName;

// Version control is now logged in the git log.
this.javaScriptVersion = Aero_Runner_v2p01.class.getSimpleName() + "f";

simu.println("AERO RUNNER: Starting.");
//==================================
// SIMULATION SETTINGS TO RUN CODE
//==================================
simu.println("Aero Runner: Reading input File.");
readInputFile();

//==============================
// SET UP FOLDER AND SIM NAMES
//==============================
// Acount for Design Manager cases
simu.println("Aero Runner: Single sim file is being run.");
this.aeroPostProcFolder = simPathName + File.separator + ".." + File.separator + this.aeroPostProcFolder;
this.aeroFolderName  = getSubFolderName();
this.studyName = getSubFolderName();

simu.println("Aero Runner: Creating directory structure for MX Post Proc Folder:");
simu.println("  " + this.aeroPostProcFolder);
SystemTool.touchDirectoryChain(simu, this.aeroPostProcFolder);
simu.println("Aero Runner: Creating directory structure for MX Post Proc Case Name Folder.");
simu.println("  " + this.aeroPostProcFolder + File.separator + this.aeroFolderName);
SystemTool.touchDirectoryChain(simu, this.aeroPostProcFolder + File.separator + this.aeroFolderName);
simu.println("Aero Runner: System is prepared to run.");

// Custom Reference Values
if(needRunWindTunnel){
  simu.println("CFD WT: Found " + wtCustomReferenceNames.length
      + " custom WindTunnelDomain reference value entries.");
  simu.println("CFD WT: Setting custom reference value arrays.");  
  for(int i = 0; i < wtCustomReferenceNames.length; i++){
    this.wtCustomReferenceNamesArray.add(wtCustomReferenceNames[i]);
    this.wtCustChordLengthArray.add(wtCustChordLengths[i]);
    this.wtCustReferenceAreaArray.add(wtCustReferenceAreas[i]);
    this.wtCustReferenceMomRDoubleArray.add(
        new double[] {wtCustReferenceMomR[i],
        wtCustReferenceMomR[i], wtCustReferenceMomR[i]} );
  simu.println("  Registering: " + wtCustomReferenceNames[i]);
  simu.println("     Chord: " + wtCustChordLengths[i]);  
  simu.println("      Area: " + wtCustReferenceAreas[i]);   
  simu.println("      MomR: " + wtCustReferenceMomR[i]);
  }
}

// Airfoil custom mesh stuff
if(needRunWindTunnel){
  for(int i=0;i<newAF_CustomMesh.length;i++){
    this.af_CustomMesh.add(newAF_CustomMesh[i]);
    this.cust_TargetSize.add(newCust_TargetSize[i]);
    this.cust_MinSize.add(newCust_MinSize[i]);
    this.cust_NPtsOnCircle.add(newCust_NPtsOnCircle[i]);
    this.cust_PrismAbsThick.add(newCust_PrismAbsThick[i]);
    this.cust_1stCellThick.add(newCust_1stCellThick[i]);
    this.cust_NPrisms.add(newCust_NPrismLayers[i]);
  }
}

//Rotor-specific custom mesh stuff
if(needRotorAnalysis){
  simu.println("CFD RTR: Found "+custRTR_MeshName.length+ " rotor mesh entries.");
  simu.println("CFD RTR: Setting custom mesh array.");  
  for(int i=0;i<custRTR_MeshName.length;i++){
    this.cust_TargetSize.add(newCust_TargetSize[i]);
    this.cust_MinSize.add(newCust_MinSize[i]);
    this.cust_NPtsOnCircle.add(newCust_NPtsOnCircle[i]);
    this.cust_PrismAbsThick.add(newCust_PrismAbsThick[i]);
    this.cust_1stCellThick.add(newCust_1stCellThick[i]);
    this.cust_NPrisms.add(newCust_NPrismLayers[i]);
  }
}

//Coordinate system setup
this.labCsys = simu
  .getCoordinateSystemManager().getLabCoordinateSystem();

// File I/O
String saveString = "";      // for Analysis objects to return file name convention
String subFolderString = ""; // for Aero Runner to name file correctly

//=====================================
// Windtunnel Analysis
//=====================================
if(needRunWindTunnel){
  simu.println("AERO RUNNER: Creating windtunnel analysis.");
  getAnyExistingReferenceLengthAndArea();
  calculateAllRemainingReferenceValues();

  refMomentRadius = new double[] {chordLength, chordLength, chordLength};

  // Instantiate Wind Tunnel Analysis Object.
  // The Wind Tunnel Analysis understands a generic chord length. 
  // Custom chord lengths are manipulated later on.
  WindTunnelAnalysis cfdSolver = new WindTunnelAnalysis(simu, chordLength);
  cfdSolver.setIs2D(needRun2D);
  cfdSolver.setHighMachNumber(highMachNumber);
  cfdSolver.setGlobalReferenceLengthVariable(chordLength);
  cfdSolver.setGlobalReferenceAreaVariable(refArea);

  if(needPreProc){
    simu.println("WT RUNNER: Preprocessing windtunnel.");
    cfdSolver.runPreProc(simu, false);

    // Not all WindTunnel Domains need to have specific custom settings.
    // For any wind tunnel that is not in the customization list, pad the
    // ArrayLists with the generic settings.
    int numWTDomains = cfdSolver.getNumberOfWTDomains();
    int numOfCustomAdds = numWTDomains - wtCustomReferenceNames.length;
    for(int i=0; i < numOfCustomAdds; i++){
      wtCustomReferenceNamesArray.add("Default");
      wtCustChordLengthArray.add(chordLength);
      wtCustReferenceAreaArray.add(refArea);
      wtCustReferenceMomRDoubleArray.add(
          new double[] {chordLength, chordLength, chordLength});
    }
    cfdSolver.updateWTCustomChordLengths(
            simu, wtCustomReferenceNamesArray, wtCustChordLengthArray);
    cfdSolver.updateWTCustomMeshBaseSizes(
            simu, wtCustomReferenceNamesArray, wtCustChordLengthArray);

    cfdSolver.initPhysics(simu, physicsName, physicsModel);
    cfdSolver.createOversetInterfaces(simu);
    cfdSolver.updateAllReferenceValues(simu, refRho, refVel, refTemp, refMu,
                                       refMa, refRe,
                                       wtCustomReferenceNamesArray,
                                       wtCustReferenceAreaArray,
                                       wtCustReferenceMomRDoubleArray);

    cfdSolver.setInitialConditions(simu);
    if(refMa > highMachNumber){
      cfdSolver.highMachAdjustment();
    }
    cfdSolver.setPostProc(simu);

    // Enables mutli design configuration setup.
    cfdSolver.setUpAlphaSweepMetrics(simu, alphaSweepAngles);

    // Save base case state.
    if(needPreProc){
      simu.saveState(simPathName + File.separator + caseName + "_pre.sim");
    }

  }
  
  if(needRunMesh){
    simu.println("WT MESH: Starting to mesh.");

    // Check for existing objects and physics continua.
    cfdSolver.instantiateExistingObjects(simu);
    cfdSolver.instantiateExistingPhysics(simu, physicsName);

    // Create OversetDomain interfaces.
    cfdSolver.createOversetInterfaces(simu);
    
    // Set the correct geometric configuration of any airfoil before the
    // alpha sweep proceeds.
    simu.println("WT SWEEP: Setting airfoil constant angles.");
    cfdSolver.modifyAirfoilSetAngles(simu, newAF_CADAngles);
    
    //Set constant mesh settings throughout the sweep
    // using any external flags
    simu.println("WT SWEEP: Setting any custom airfoil surface mesh controls.");
    cfdSolver.setAirfoilSurfaceMeshSettings(af_CustomMesh,
        cust_TargetSize, cust_MinSize, cust_NPtsOnCircle);
    
    simu.println("WT SWEEP: Setting any custom airfoil volume mesh controls.");
    cfdSolver.setAirfoilVolumeMeshSettings(simu,
        af_CustomMesh, af_LowYPlus, af_HighYPlus,
        cust_PrismAbsThick, cust_1stCellThick, cust_NPrisms);
    simu.println("WT SWEEP: Executing Mesh Models");
    cfdSolver.executeAllMeshModels(simu);
    
    // Save base case state.
    caseName = simu.getPresentationName();
    if(needRunMesh && !needRunSolver){
      simu.saveState(simPathName + File.separator + caseName + "_meshed.sim");
    }
  }

  if(needRunSolver){
    simu.println("WT SWEEP: Starting alpha sweep.");

    // Check for existing objects and physics continua.
    cfdSolver.instantiateExistingObjects(simu);
    cfdSolver.instantiateExistingPhysics(simu, physicsName);

    // Create OversetDomain interfaces.
    cfdSolver.createOversetInterfaces(simu);
    
    // Set the correct geometric configuration of any airfoil before the
    // alpha sweep proceeds.
    simu.println("WT SWEEP: Setting airfoil constant angles.");
    cfdSolver.modifyAirfoilSetAngles(simu, newAF_CADAngles);
    
    //Set constant mesh settings throughout the sweep
    // using any external flags
    simu.println("WT SWEEP: Setting any custom airfoil surface mesh controls.");
    cfdSolver.setAirfoilSurfaceMeshSettings(af_CustomMesh,
        cust_TargetSize, cust_MinSize, cust_NPtsOnCircle);
    
    simu.println("WT SWEEP: Setting any custom airfoil volume mesh controls.");
    cfdSolver.setAirfoilVolumeMeshSettings(simu,
        af_CustomMesh, af_LowYPlus, af_HighYPlus,
        cust_PrismAbsThick, cust_1stCellThick, cust_NPrisms);
    
    // Apply correct physics settings.
    // Not all WindTunnel Domains need to have specific custom settings.
    // For any wind tunnel that is not in the customization list, pad the
    // ArrayLists with the generic settings.
    int numWTDomains = cfdSolver.getNumberOfWTDomains();
    int numOfCustomAdds = numWTDomains - wtCustomReferenceNames.length;

    for(int i=0; i < numOfCustomAdds; i++){
      wtCustomReferenceNamesArray.add("Default");
      wtCustChordLengthArray.add(chordLength);
      wtCustReferenceAreaArray.add(refArea);
      wtCustReferenceMomRDoubleArray.add(
          new double[] {chordLength, chordLength, chordLength});
    }

    // Set additional new chord lengths for updated chord lengths without redo
    // of the pre-processor.
    cfdSolver.updateWTCustomChordLengths(
        simu, wtCustomReferenceNamesArray, wtCustChordLengthArray);

    simu.println("Setting Reference Values:");
    simu.println("Names: "+wtCustomReferenceNamesArray);
    simu.println("Areas: "+wtCustReferenceAreaArray);
    simu.println("MomentR:");
    for(int i=0; i < wtCustomReferenceNamesArray.size();i++){
      SimTool.simPrint1x3Array(simu, wtCustReferenceMomRDoubleArray.get(i));
    }

    cfdSolver.updateAllReferenceValues(simu, refRho, refVel, refTemp, refMu,
                                       refMa, refRe,
                                       wtCustomReferenceNamesArray,
                                       wtCustReferenceAreaArray,
                                       wtCustReferenceMomRDoubleArray);
    cfdSolver.setInitialConditions(simu);
    cfdSolver.setTmpSim(simu);
    cfdSolver.applyBoundaryConditions(refVel, refTemp, refMa, fsPressure);
    cfdSolver.updateBLProbes(simu,refRho,refMu);
    //Solver settings
    if(refMa > highMachNumber){
      cfdSolver.highMachAdjustment();
    }
    if(turnOnAutoTimeStep){
      simu.println("WT SWEEP: Turning on Auto Time Step");
      cfdSolver.turnOnAutoTimeStep();
    }
    if(needPreProc || needClearSolution){
      simu.println("WT SWEEP: Clearing Solution");
      cfdSolver.clearSolution(simu);
    }
    simu.println("WT SWEEP: Executing Mesh Models");
    cfdSolver.executeAllMeshModels(simu);
    
    simu.println("WT SWEEP: Setting up airfoil relative stopping criterion.");
    double pctCLChange = 1.0;
    double pctCDChange = 1.0;
    double pctCMChange = 10.0;
    cfdSolver.setAirfoilRelativeChangeStoppingCriterion(
        simu, pctCLChange, pctCDChange, pctCMChange);

    simu.println("WT SWEEP: Setting necessary post processing.");
    cfdSolver.setPostProc(simu);

    //==================================
    // BEGIN THE WIND TUNNEL SWEEP
    //==================================
    //Step through angles during sweep
    boolean use3EqSGamma = false;
    double oldSweepAngle = alphaSweepAngles[0];
    simu.println("WT SWEEP Starting Angle: " + oldSweepAngle);
    int timesThrough = 0;
    for(double nextAngleOfAttack : alphaSweepAngles){
      // Assembly rotation/meshing
      cfdSolver.modifyMeshAngleOfAttack(simu, nextAngleOfAttack);
      cfdSolver.generateSurfaceMesh(simu);
      cfdSolver.generateVolumeMesh(simu);

      // 2D Simulations need to re-inject their domains and regions into the
      // simulation after badging.
      if(needRun2D && timesThrough == 0){
        cfdSolver.implement2DObjectsIntoScenes(simu);
        timesThrough = 1;
      }

      // Overset needs to modify its angle separately
      // and is not a mesh driven operation.
      cfdSolver.createOversetInterfaces(simu);
      cfdSolver.modifyOversetAngleOfAttack(simu, nextAngleOfAttack);

      // BLP CAD parts need to be manually rotated to next angle
      cfdSolver.rotateBoundaryLayerProbeCADParts(simu, nextAngleOfAttack);

      // Set up appropriate physics and solver.
      simu.println("WT SWEEP: Starting solver.");
      //kw-SST options
      if(physicsModel.equals("kwSST-3EqSGamma")){
          use3EqSGamma = true;
          simu.println("Using 3Eq Gamma Model");
          physicsModel="kwSST";
      }
      if((nextAngleOfAttack > 18.0 || (nextAngleOfAttack<oldSweepAngle))
         &&!needRun2D && !(refMa > highMachNumber)){
          if(use3EqSGamma){
             physicsModel = "kwSST-3EqSGamma-DES";
             simu.println("Using 3Eq Gamma DES Model");
          }else{
              physicsModel = "kwSSTGRTDES";
          }
      }else if(needRun2D || 
          (nextAngleOfAttack > 12.0&& !(refMa > highMachNumber))){
          if(use3EqSGamma){
              physicsModel = "kwSST-3EqSGamma";
              simu.println("Using 3Eq Gamma Model");
          }else{
              physicsModel = "kwSSTGRT";
          }
      }
      // note: This exists because it updates the internal angle
      cfdSolver.updateAllReferenceValues(simu, refRho, refVel, refTemp, refMu,
                                         refMa, refRe,
                                         wtCustomReferenceNamesArray,
                                         wtCustReferenceAreaArray,
                                         wtCustReferenceMomRDoubleArray);
      cfdSolver.initPhysics(simu,physicsName,physicsModel);
      //relax BL Probes at this point in the code to avoid detecting background region

      simu.println("WT SWEEP: Simulation is unsteady.");
      if(cfdSolver.isUnsteady()){
        //additional post
        cfdSolver.createUnsteadyObjectReporting(simu);
        //time step controls
        if(turnOnAutoTimeStep){
            unSteadyTimeStep = WindTunnelAnalysis.setTimeStep(simu,
                cfdSolver.getRecommendedTimeStep(
                    unSteadyTimeStep, timeDiscretizations));
        }else{
            WindTunnelAnalysis.setTimeStep(simu, unSteadyTimeStep);
        }
        WindTunnelAnalysis.setInnerIterations(simu, unSteadyInnerIterations);

        // If needed, set up unsteady stopping criteria.
        if(turnOnAutoTimeStep){
            //let cfdSolver calculate based on physics
            double autoFinalTime = cfdSolver.getRecommendedTotalRunTime();
            WindTunnelAnalysis.deactivateMaxStepsCriteria(simu);
            maxNumSteps=(int) (autoFinalTime/unSteadyTimeStep);
        }else{
            WindTunnelAnalysis.setFinalSolutionTime(simu,unSteadyMaxSteps*unSteadyTimeStep);
            WindTunnelAnalysis.deactivateMaxStepsCriteria(simu);
        }
      }

      // Update to new case name before solving based on new physics
      caseName = cfdSolver.getFileString(simu);
      simu.println("  = "+caseName);

      cfdSolver.setCaseAnnotationName(studyName, caseName);
      cfdSolver.initAnnotations(simu);

      //Solve
      cfdSolver.runSolver(simu,maxNumSteps);

      //Post Processing
      simu.println("WT RUNNER: Postprocessing this angle."); 
      cfdSolver.completeSweepMetricValues(simu, nextAngleOfAttack);
      cfdSolver.updateBLProbes(simu, refRho, refMu/refRho);

      String fileString = cfdSolver.makeSheetsCSVData(simu,
          studyName, ccmVersion, javaScriptVersion);
      simu.println("CSV Sheets Output Line:");
      simu.println(fileString);
      simu.println(" ");

      // Writes all csv values into a unified AERO_POST study directory.
      String dataFileStr = this.aeroPostProcFolder + File.separator
              + this.aeroWTPostProcCSVFile;
      simu.println(
              "Writing Aero Runner Post Folder based CSV data to: " + dataFileStr);
      writeOutToDataFile(fileString, dataFileStr);

      // Writes all csv values into a localized study directory.
      String localFileStr = this.aeroPostProcFolder + File.separator + ".."
          + File.separator + getSubFolderName() + File.separator
          + this.aeroWTPostProcCSVFile;
      simu.println("Writing Local CSV data to this location: " + localFileStr);
      writeOutToDataFile(fileString, localFileStr);

      // File I/O
      //   This file is always saved to the local simPath directory
      //   Either this is the study folder or the optimate design folder
      simu.saveState(this.simPathName + File.separator
              + getSubFolderName() + "_" + caseName + ".sim");        

      // Output final step images by default
      cfdSolver
          .postProcessAeroTools(simu, this.aeroPostProcFolder + File.separator
              + this.aeroFolderName + File.separator);
      cfdSolver
          .outputAllScenes(simu, this.aeroPostProcFolder + File.separator
              + this.aeroFolderName + File.separator);

      // track previous sweep angle to see if we are going back down the stall curve
      oldSweepAngle = nextAngleOfAttack;
    }
  }
}

//=====================================
// Rotor Analysis
//=====================================
if(needRotorAnalysis){ //RTR ANALYSIS
    this.bodyCsys = (CartesianCoordinateSystem) labCsys
      .getLocalCoordinateSystemManager()
      .getObject(globalNames.getBodyCsysName());
    // Begin Rotor Analysis
    simu.println("AERO RUNNER: Creating rotor analysis.");
    getAnyExistingReferenceLengthAndArea();
    calculateAllRemainingReferenceValues();
    refMomentRadius=new double[] {chordLength,chordLength,chordLength};

    //Instantiate Full Configuration Study
    RotorAnalysis cfdSolver = new RotorAnalysis(simu,bodyCsys);
    cfdSolver.setLengthScale(chordLength);
    cfdSolver.setMeshingBaseSize(chordLength*0.1);

    //Preprocessing
    if(needPreProc){ // ROTOR ANALYSIS
        simu.println("RTR RUNNER: Preprocessing rotor analysis.");
        cfdSolver.runPreProc(simu,false);
        cfdSolver.initPhysics(physicsName,physicsModel);

        cfdSolver.updateAllReferenceValues(simu,refRho,refVel,1.0,refTemp
                ,refMu,refMa,refRe,refArea,refMomentRadius);
        //rotor needs to init motions
        cfdSolver.initRotorMotions();
        cfdSolver.setInitialConditions();

        //Initialize Post Processing
        cfdSolver.instantiateExistingObjects(); // Make sure everything updated
        cfdSolver.initPostProc(simPathName+File.separator
                +"AERO_POST"+File.separator,false,1);

        //File Save
        if(!needRunMesh){
            simu.saveState(caseName+"_preproc.sim");
        }
    }

    //Meshing
    if(needRunMesh){ // ROTOR ANALYSIS
      simu.println("RTR RUNNER: Starting mesher.");
      cfdSolver.instantiateExistingObjects();

      // Rotors can update their assigned speeds if needed
      simu.println("RTR RUNNER: Checking for Rotor Speed updates.");
      cfdSolver.updateRotorSpeeds(newRTR_SpeedName,newRTR_Speed);

      // Manipulate Geometric Configuration
      simu.println("RTR RUNNER: Setting assembly mesh angle of attack.");
      cfdSolver.modifyAlphaAndBeta(new double[] {newAngleOfAttack,
              newSideSlipAngle});

      // Modify airfoil elements based on any external flags
      simu.println("RTR RUNNER: Setting custom rotor surface mesh Settings.");
      cfdSolver.setRotorSurfaceMeshSettings(custRTR_MeshName,
              cust_TargetSize,cust_MinSize,cust_NPtsOnCircle);

      simu.println("RTR RUNNER: Setting custom rotor Volume Mesh settings.");
      cfdSolver.setRotorVolumeMeshSettings(
              custRTR_MeshName,af_LowYPlus,af_HighYPlus,
              cust_PrismAbsThick,cust_1stCellThick,cust_NPrisms);

      simu.println("RTR RUNNER: Generating Surface Mesh Settings.");
      cfdSolver.generateSurfaceMesh();
      cfdSolver.generateVolumeMesh();
      //save out in case you just posted it up
      if(!needRunSolver&&needPreProc){
        simu.saveState(caseName+"_preproc_mesh.sim");
      }else if (!needRunSolver && !needPreProc){
        simu.saveState(caseName+"_mesh.sim");
      }
    }

    if(needRunSolver){ // ROTOR ANALYSIS
        simu.println("RTR RUNNER: Starting solver.");
        simu.println("RTR RUNNER: Is Mesh Up To Date: "
                + cfdSolver.checkMeshOperationsUpToDate());

        //Object feelers
        simu.println("RTR RUNNER: Instantiating Objects.");
        cfdSolver.instantiateExistingObjects();
        cfdSolver.instantiateExistingPhysics(physicsName);

        cfdSolver.createOversetInterfaces();
        cfdSolver.createSlidingInterfaces();

        //Physics Info
        cfdSolver.getAngleOfAttack();
        cfdSolver.getSideSlipAngle();

        // Rotors load in their own Physics
        cfdSolver.initAllRBMRotorSpeeds();
        cfdSolver.initRotorMotions();

        // Rotors can update their assigned speeds if needed from command line
        simu.println("RTR RUNNER: Updating Rotor specific data.");
        cfdSolver.updateRotorSpeeds(newRTR_SpeedName,newRTR_Speed);
        // for rotors, we use average tip speed for cp
        double tipVel = cfdSolver.getAverageRotorTipSpeed(); 
        cfdSolver.updateAllReferenceValues(simu, refRho, refVel, tipVel,
                refTemp, refMu, refMa,refRe,refArea,refMomentRadius);

        simu.println("RTR RUNNER: Is Mesh Up To Date: "
                +cfdSolver.checkMeshOperationsUpToDate());

        simu.println("RTR RUNNER: Setting continuum initial conditions.");
        cfdSolver.setInitialConditions();

        simu.println("RTR RUNNER: Applying boundary conditions.");
        cfdSolver.applyBoundaryConditions(refVel,refTemp,refMa,fsPressure);

        // Apply Correct Physics Models
        if(needUnsteady) physicsModel="kwSSTDES";
        simu.println("RTR RUNNER: Init physics.");
        cfdSolver.initPhysics(physicsName,physicsModel);

        // Object initial condition setup
        cfdSolver.initializeObjectICs();
        cfdSolver.activateSolverLinearRamp(10);

        if(forceRTR_MRF){
            simu.println("RTR RUNNER: Forcing all rotors to MRF.");
            cfdSolver.forceRotorsToMRF();
        }

        // Solver drivers
        if(needClearSolution){
            simu.println("RTR RUNNER: Clear solution enabled.");
            cfdSolver.clearSolution();
        }
        if(turnOnAutoTimeStep){
            simu.println("RTR RUNNER: Calculating auto time step.");
            cfdSolver.turnOnAutoTimeStep();
        }
        if(cfdSolver.isUnsteady()){
            if(!forceRTR_MRF){
                cfdSolver.initPhysics(physicsName,"kwSSTDES");
                cfdSolver.forceRotorsToRBM();
            }
            // Additional post when performing RBM/Unsteady
            cfdSolver.createUnsteadyObjectReporting();

            // Time step controls
            if(turnOnAutoTimeStep){
                unSteadyTimeStep=cfdSolver.setTimeStep(cfdSolver
                        .getRecommendedTimeStep(unSteadyTimeStep,timeDiscretizations));
            }else{
                cfdSolver.setTimeStep(unSteadyTimeStep);
            }
            cfdSolver.setInnerIterations(unSteadyInnerIterations);
            // Unsteady stopping criteria
            if(turnOnAutoTimeStep){
                // Let cfdSolver calculate based on physics
                double autoFinalTime = cfdSolver.getRecommendedTotalRunTime();
                cfdSolver.deactivateMaxStepsCriteria();
                maxNumSteps=(int) (autoFinalTime/unSteadyTimeStep);
            }else{
                cfdSolver.setFinalSolutionTime(maxNumSteps*unSteadyTimeStep);
                cfdSolver.deactivateMaxStepsCriteria();
            }

        }

        if(cfdSolver.isUnsteady()){
            cfdSolver.cleanUpMonitors(unSteadyMaxSteps);
        }else{
            cfdSolver.cleanUpMonitors(50);
        }

        simu.println("RTR RUNNER: "
                +"Check mesh before post-processing is up-to-date: "
                +cfdSolver.checkMeshOperationsUpToDate());

        // POST PROCESSING SETUP
        // Annotations for active Plots
        simu.println("RTR RUNNER: Updating case annotiations");
        saveString=cfdSolver.getFileString();
        cfdSolver.setCaseAnnotationName(saveString);

        //Ensure Proper Post Processing
        // TODO: Can do better than a constant 5.
        simu.println("RTR RUNNER: Updating post-processing settings");
        cfdSolver.initPostProc(simPathName+File.separator
                +"AERO_POST"+File.separator
                ,cfdSolver.isUnsteady(), 5);

        //****** SOLVER ****//
        // Check & Run SS/MRF Initial Condition
        if(cfdSolver.isUnsteady()){ //wall distance freeze trick
            cfdSolver.wallDistanceSolverTrickStep();
        }

        simu.println("RTR RUNNER: Max Steps: "+maxNumSteps);
        cfdSolver.runSolver(maxNumSteps);

        // File I/O
        subFolderString=getSubFolderName();
        saveString = cfdSolver.getFileString();
        simu.saveState(simPathName+File.separator
                +subFolderString+"_"+saveString+".sim");
    }

    if(needRunPostProc||needRunQuickPost){ // ROTOR ANALYSIS
        simu.println("RTR RUNNER: Postprocessing windtunnel.");

        // Instantiate Objects
        cfdSolver.instantiateExistingObjects();
        cfdSolver.instantiateExistingPhysics(physicsName);

        //Update Information on Angle of Attack and Sideslip
        //  in case there is a stand alone request for post processing.
        refVel = cfdSolver.getDomainInletSpeed();
        cfdSolver.getAngleOfAttack();
        cfdSolver.getSideSlipAngle();
        // for rotors, we use average tip speed for cp
        double tipVel=cfdSolver.getAverageRotorTipSpeed(); 
        cfdSolver.updateAllReferenceValues(simu,refRho,refVel,
                tipVel,refTemp,refMu, refMa,refRe,refArea,refMomentRadius);

        // just in case you are loading a post-process from archive file
        cfdSolver.initAllRBMRotorSpeeds();

        String fileString=cfdSolver.makeSheetsCSVData(
                caseName,ccmVersion,javaScriptVersion);
        simu.println("CSV Sheets Output Line:");
        simu.println(fileString);
        simu.println(" ");
        String mdRelativePath="";
        String optBasePath="";
        if(isMultiDesignCase){
          mdRelativePath = ".."+File.separator+".."+File.separator;
          optBasePath=".."+File.separator;
        }
        String dataFileStr=simPathName+File.separator
                +mdRelativePath+optBasePath+"AERO_RTR_Sweep_Results.csv";
        simu.println("Saving CSV to: "+dataFileStr);
        writeOutToDataFile(fileString, dataFileStr);

        if(!needRunQuickPost){ // Go through entire post processing behavior
          // Creating the Aero Runner Post file system structure
          String mxPostFolder = simPathName + File.separator + "AERO_POST";
          SystemTool.touchDirectoryChain(simu, mxPostFolder);
          String postFileDir = mxPostFolder + File.separator + caseName;
          SystemTool.touchDirectoryChain(simu, postFileDir);

          // File I/O
          simu.println("RTR RUNNER: Updating post-processing settings.");
          subFolderString = getSubFolderName();
          cfdSolver.initPostProc(postFileDir, cfdSolver.isUnsteady(), 5);
          cfdSolver.runAllPostProc(postFileDir);
          saveString = cfdSolver.getFileString();
          simu.saveState(simPathName + File.separator + subFolderString 
                  + "_" + saveString + ".sim");
        }
    }
}

    //=====================================
    // TODO: Full Configuration Analysis
    //=====================================

}

    //============================================
    // SIM FRAMEWORK REFRENCE VALUE METHODS
    //============================================
    public void getAnyExistingReferenceLengthAndArea(){
        simu.println("Aero Runner: Testing for pre-existing "
                + "reference values in sim.");

        // Get any pre-existing Aero Runner Length Scale
        String lengthParamName = "Aero Runner Reference Length Scale";
        ScalarGlobalParameter referenceLengthParam = 
          SimTool.getScalarSimulationParameter(simu, lengthParamName);

        //check if new value is any different than existing value
        double simLengthValue = referenceLengthParam.getQuantity().getSIValue();
        if((Math.abs(simLengthValue - chordLength)>1e-5)){
          simu.println("Aero RUNNER: New Chord Length Detected! Delta: "
              + (Math.abs(simLengthValue-chordLength)>1e-5));          
          simu.println("Aero RUNNER: Value of input ref    chord: "
              + chordLength);
          simu.println("Aero RUNNER: Existing value of sim chord: "
              + simLengthValue);
        }

        if((Math.abs(simLengthValue-chordLength)>1e-5
            && ((Math.abs(chordLength)<1e-5) 
                && Math.abs(simLengthValue)>1e-5))){
            // User has prescribed a different value 
            // and did not choose to overwrite
            referenceLengthParam.getQuantity().
                    setDefinition(String.valueOf(simLengthValue));
            chordLength=simLengthValue;
        }else if (Math.abs(chordLength)>1e-5){
            referenceLengthParam.getQuantity().
                    setDefinition(String.valueOf(chordLength));
        }else{
            // User has not previously entered a value, 
            // use default value for chordLength
            chordLength=defaultLengthScale;
            referenceLengthParam.getQuantity().
                    setDefinition(String.valueOf(chordLength));
        }

        // Get any pre-existing Aero Runner Reference Areas
        String areaParamName = "Aero Runner Reference Area";
        ScalarGlobalParameter referenceAreaParam = 
          SimTool.getScalarSimulationParameter(simu, areaParamName);

        // Check if new value is any different than existing value
        double simAreaValue = 
            referenceAreaParam.getQuantity().getSIValue();
        if((Math.abs(simAreaValue-refArea)>1e-5
            && ((Math.abs(refArea)<1e-5) && Math.abs(simAreaValue)>1e-5))){
            referenceAreaParam.getQuantity()
                .setDefinition(String.valueOf(simAreaValue));
            refArea=simAreaValue;
        }else if (Math.abs(refArea)>1e-5){
            referenceAreaParam.getQuantity()
                .setDefinition(String.valueOf(refArea));
        }else{ // Default value for area
            refArea=defaultReferenceArea;
            referenceAreaParam.getQuantity()
                .setDefinition(String.valueOf(refArea));
        }
    }
    private void calculateAllRemainingReferenceValues(){
        //Simulation reference values
        double idealR   = 8.3144598;  // [kJ/kmol.K]  ideal gas constant
        double molAir   = 0.0289645;  // [kg/mol] molar mass of air
        double airGamma = 1.4;        // [#] ratio of specific heats
        double refPress = 101325.0;   // [Pa] reference pressure for air
        double airR = (idealR/molAir); // [kJ/kg.K] specific gas constant

        // Choices:
        //   User chooses chord or accept chord in sim file or default=1 [m]
        //   User chooses the reference pressure or accept 101 325 Pa default
        //   - Speed & Rho   -> Set Re, Temp, Mach
        //   - Re    & Rho   -> Set Speed, Temperature, Mach
        //   - Ma    & Temp. -> Set Speed, Rho, Re
        //   - Re    & Mach  -> Set Temp, Rho, Speed
        //   Note: This currently does not assume isentropic relationships
        if(Math.abs(refRe)<1e-6){
            refRe=chordLength*refVel*refRho/refMu;
            refTemp=(fsPressure+refPress)/refRho/airR;
            refMa = refVel/Math.sqrt(airGamma*airR*refTemp);
            simu.println("Aero RUNNER:"
                    +" User has selected Speed / Rho combination :");
            simu.println(" Enforced ---> ");
            simu.println("  Speed: "+refVel);
            simu.println("  Rho  : "+refRho);
            simu.println("  Press: "+(fsPressure+refPress));
            simu.println(" Derived ---> ");
            simu.println("    Re: "+refRe);
            simu.println("  Temp: "+refTemp);
            simu.println("  Mach: "+refMa);

        // User has chosen a defacto freestream condition
        }else if(refMa>1e-5&&refTemp>1e-5){
            refVel=refMa*Math.sqrt(airGamma*airR*refTemp);
            refRho=(fsPressure+refPress)/refTemp/airR;
            refRe=chordLength*refVel*refRho/refMu;
            simu.println("Aero RUNNER:"
                    +" User has selected Ma / Temp. combination:");
            simu.println(" Enforced ---> ");
            simu.println("  Mach: "+refMa);
            simu.println("  Temp: "+refTemp);
            simu.println("  Press: "+(fsPressure+refPress));
            simu.println(" Derived ---> ");
            simu.println("  Speed: "+refVel);
            simu.println("    Rho: "+refRho);
            simu.println("     Re: "+refRe);

        // User needs Mach number and Reynolds number option
        }else if(refMa>1e-5&&refRe>1e-6){
          double absPress = (fsPressure+refPress);
          refTemp=airGamma/airR*Math.pow(absPress*chordLength*refMa/refRe/refMu,2);
          refRho =absPress/(airR*refTemp);
          refVel=refRe*refMu/(refRho*chordLength);
          
            simu.println("Aero RUNNER:"
                    +" User has selected Ma / Re combination:");
          simu.println(" Enforced ---> ");
          simu.println("   Mach: "+refMa);
          simu.println("     Re: "+refRe);
          simu.println("  Press: "+absPress);
          simu.println(" Derived ---> ");
          simu.println("   Temp: "+refTemp);
          simu.println("    Rho: "+refRho);
          simu.println("  Speed: "+refVel);

        // User has selected the Reynolds number option
        }else if(Math.abs(refRe)>1e-6){
            refVel=refRe*refMu/refRho/chordLength;
            refTemp=(fsPressure+refPress)/refRho/airR;
            refMa= Math.sqrt(airGamma*airR*refTemp);
            simu.println("Aero RUNNER:"
                    +" Re / Rho / Press combination selected:");
            simu.println(" Enforced ---> ");
            simu.println("  Rho: "+refRho);
            simu.println("  Re : "+refRe);
            simu.println("  Press: "+(fsPressure+refPress));
            simu.println(" Derived ---> ");
            simu.println("  Speed: "+refVel);
            simu.println("  Temp: "+refTemp);
            simu.println("  Mach: "+refMa);
        }
    }

  //============================================
  // DATA I/O METHODS
  //============================================
  private String getSubFolderName(){
    /* getSubFolderName returns the name of the subFolder
        from which the simulation was launched.
        This is referred to as the study directory.
    */
    int lastIndx = simPathName.lastIndexOf(File.separator);
    return simPathName.substring(lastIndx + 1);
}
  private void writeOutToDataFile(String simOutputData, String nameDataFile){
    Writer writer = null;
    try {
      writer = new BufferedWriter(new OutputStreamWriter(
          new FileOutputStream(resolvePath(nameDataFile), true), "utf-8"));
      ((BufferedWriter) writer).newLine();
      writer.write(simOutputData);
    } catch (IOException ex) {
      // report
    } finally {
       try {writer.close();} catch (Exception ex) {/*ignore*/}
    }
  }
  boolean checkIfMultiDesign(){
    ScalarGlobalParameter starGlobalParam;
    double optValue = 0.0;
    try{
      starGlobalParam = ((ScalarGlobalParameter) simu
          .get(GlobalParameterManager.class).getObject("isMultiDesignCase"));
    }catch(NeoException e){
      starGlobalParam = (ScalarGlobalParameter) simu
          .get(GlobalParameterManager.class)
          .createGlobalParameter(
              ScalarGlobalParameter.class, "Scalar");
      starGlobalParam.setPresentationName("isMultiDesignCase");
      starGlobalParam.getQuantity().setValue(0.0);
    }
    optValue = starGlobalParam.getQuantity().getInternalValue();
    if(Math.abs(optValue - 1.0) < 1.0E-4){
      return true;
    }else{
      return false;
    }
  }

  //============================================
  // Aero RUNNER INPUT CONTROL METHODS
  //============================================
  //read file & assign variables
  private void readInputFile(){
    //simu.println("Starting Read Input File.");
    ArrayList<String> data = new ArrayList();
    ArrayList<String> vars = new ArrayList();
    String mdRelativePath = "";
    mdRelativePath = ".."+File.separator+".."+File.separator+".."
            +File.separator+".."+File.separator;
    try{
      String inputFileStr = simPathName + File.separator + "aero_input.cfd";
      String multiDesignFileStr = simPathName + File.separator 
              + mdRelativePath + "aero_input.cfd";
      File localAEROInput = new File(inputFileStr);
      File optAEROInput = new File(multiDesignFileStr);
      Scanner scan;
      if(localAEROInput.exists()){
        scan = new Scanner(localAEROInput); 
      }else{
        scan = new Scanner(optAEROInput); 
      }

      while (scan.hasNext()){
        String line = scan.nextLine();
        if(_input_debug){
          simu.println("line= " + line);
        }
        if(readerIsCommentLine(line)){
        } else{
          line=line.replaceAll("\\s","");
          if(_input_debug){
            simu.println("line: "+line);
          }
          String[] parts = line.split("=");
          String tmp = null;
          String p2  = null;
          boolean b  = true;
          int i = parts[1].indexOf("//"); // note: that this line 
                                          // requires/expects a comment section
                                          // after the variable
          if(i != -1){
            tmp = parts[1];
            p2= tmp.substring(0, i);
            if(_input_debug){
              simu.println("First Part: "+parts[0]+", 2nd Part: "+p2);
            }
            if(p2.equals("true")||p2=="false"){
               data.add(p2);
            } else if(p2.startsWith("{")){
              if(_input_debug){
                simu.println("this is array");
              }
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
                if(_input_debug){
                  simu.println("Check if its a name");
                }
                if ( parts[0].matches("wtCustomReferenceNames") ||
                     parts[0].matches("newRTR_SpeedName") ||
                     parts[0].matches("custRTR_MeshName") ) {
                  String s1= "{"+p2+"}";
                  data.add(s1);
                  if(_input_debug){
                    simu.println(s1+ " string");
                  }
                }else{
                data.add(p2);
                  if(_input_debug){
                    simu.println(p2+ " string");
                  }
                }

              }else if(parts[0].equals("maxNumSteps")
                      || parts[0].equals("unSteadyInnerIterations")
                      || parts[0].equals("unSteadyMaxSteps")){
                if(_input_debug){
                  simu.println("Special variables");
                }
                int[] i2= new int[p3.length];
                if(_input_debug){
                  System.out.println(Arrays.toString(i2)+ " int");
                }
                data.add(i2.toString());
              }else if(parts[0].matches("newCust_NPrismLayers")){
                if(_input_debug){
                  simu.println("Integer prism layers");
                }
                int[] i3 =new int[p3.length];
                for(int c=0; c<p3.length; c++){
                  i3[c]=Integer.parseInt(p3[c]);
                }
                String f3 = Arrays.toString(i3).substring(
                        1,Arrays.toString(i3).length()-1);
                f3 = "{"+f3+"}";
                data.add(f3);
                if(_input_debug){
                  System.out.println(f3+" int");
                }

              }else {
                if(_input_debug){
                  simu.println("Double data");
                }
                double[] p3a= new double[p3.length];
                for(int c=0; c<p3.length; c++){
                    p3a[c]=Double.parseDouble(p3[c]);
                }
                String f2 = Arrays.toString(p3a).substring(
                        1,Arrays.toString(p3a).length()-1);
                f2 = "{"+f2+"}";
                data.add(f2);
                if(_input_debug){
                  System.out.println(f2+" Double");
                }
              }
            }else{
              data.add(p2);
            }
          }
          vars.add(parts[0]);
        }
      }
      if(_input_debug){
        simu.println("Files have been read.");
      }

    } catch (FileNotFoundException e) {  
      simu.println("ERROR: There is no aero_input.cfd" 
              + " file located in the study directory!");
    }
    setVariables(vars,data);
  }
  private boolean readerIsCommentLine(String currentLine){
    boolean isLineEmpty = currentLine.isEmpty();
    boolean ifNoEqualsSign = !(currentLine.contains("="));
    
    return ifNoEqualsSign ||  isLineEmpty
        || currentLine.startsWith("\\//") 
        || ( currentLine.contains("=") && currentLine.startsWith("\\//") )
        || currentLine.indexOf("//") == 0;
  }
  private void setVariables(ArrayList<String> vars, ArrayList<String> data){
      String a1=null;

      for(int j =0;j<vars.size();j++){
        String variable=vars.get(j);
        if (_input_debug){
          simu.println("var = "+variable +" data = " + data.get(j));
        }
        
        switch(variable)
        {
          case "isMultiDesignCase":
            this.isMultiDesignCase= Boolean.parseBoolean(data.get(j));
            break;
          case "needPreProc":
              this.needPreProc = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunMesh":
              this.needRunMesh = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunSolver":
              this.needRunSolver = Boolean.parseBoolean(data.get(j));
              break;
          case "needUnsteady":
              this.needUnsteady = Boolean.parseBoolean(data.get(j));
              break;
          case "needClearSolution":
              this.needClearSolution = Boolean.parseBoolean(data.get(j));
              break;
          case "needRunPostProc":
              this.needRunPostProc = Boolean.parseBoolean(data.get(j));
              break;    
          case "needRunQuickPost" :
              this.needRunQuickPost= Boolean.parseBoolean(data.get(j));
              break;
          case "maxNumSteps":
              this.maxNumSteps = Integer.valueOf((String) data.get(j));
              break;
          case "turnOnAutoTimeStep" :
              this.turnOnAutoTimeStep= Boolean.parseBoolean(data.get(j));
              break;
          case "needRunWindTunnel":
              this.needRunWindTunnel = Boolean.parseBoolean(data.get(j));
              break;
          case "needRun2D":
              this.needRun2D = Boolean.parseBoolean(data.get(j));
              break;
          case "needRotorAnalysis":
              this.needRotorAnalysis = Boolean.parseBoolean(data.get(j));
              break;
          case "needAlphaSweep":
              this.needAlphaSweep = Boolean.parseBoolean(data.get(j));
              break;  
          case "chordLength":
              this.chordLength = Double.valueOf((String) data.get(j));
              break;
          case "refArea":
              this.refArea = Double.valueOf((String) data.get(j));
              break;
          case "refRe":
              this.refRe = Double.valueOf((String) data.get(j));
              break;
          case "refVel":
              this.refVel = Double.valueOf((String) data.get(j));
              break;
          case "refRho":
              this.refRho = Double.valueOf((String) data.get(j));
              break;   
          case "refMa":
              this.refMa = Double.valueOf((String) data.get(j));
              break;
          case "refTemp":
              this.refTemp = Double.valueOf((String) data.get(j));
              break;   
          case "unSteadyTimeStep":
              this.unSteadyTimeStep = Double.valueOf((String) data.get(j));
              break;
          case "unSteadyInnerIterations":
              this.unSteadyInnerIterations = Integer.valueOf(
                      (String) data.get(j));;
              break;
          case "unSteadyMaxSteps":
              this.unSteadyMaxSteps = Integer.valueOf((String) data.get(j));;
              break;   
          case "physicsModel":
              this.physicsModel = (String) data.get(j).toString();
              break;
          case "refMu":
              this.refMu = Double.valueOf((String) data.get(j));
              break;  
          case "newAngleOfAttack":
              this.newAngleOfAttack = Double.valueOf((String) data.get(j));
              simu.println("newAngleOfAttack = " + newAngleOfAttack);
              break;
          case "newSideSlipAngle":
              this.newSideSlipAngle = Double.valueOf((String) data.get(j));
              break;
          case "alphaSweepAngles":
            if(_input_debug){
              simu.println("alphaSweepAngles");
            }
            a1=""; 
            a1 = (String) data.get(j);
            a1=a1.replaceAll("\\{", "");
            a1=a1.replaceAll("\\}", "");
            String[] a2 = a1.split(",");
            this.alphaSweepAngles = new double[a2.length];
            for(int i=0;i<a2.length;i++){
              alphaSweepAngles[i]= Double.valueOf(a2[i]);
              if(_input_debug){
                simu.println(alphaSweepAngles[i]);
              }
            }

            break; 
          case "betaSweepAngles":
              a1=""; a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.betaSweepAngles = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  betaSweepAngles[i]= Double.valueOf(a2[i]);
              }
              break;
          case "wtCustomReferenceNames":
            a1= ""; 
            a1 = (String) data.get(j);
            a1=a1.replaceAll("\\{", "");
            a1=a1.replaceAll("\\}", "");
            this.wtCustomReferenceNames = a1.split(",");
            for(int i=0;i<wtCustomReferenceNames.length;i++){
              wtCustomReferenceNames[i]=
                      wtCustomReferenceNames[i].replaceAll("\"","");
              simu.println(wtCustomReferenceNames[i]);
            }
            break;
          case "wtCustChordLengths":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.wtCustChordLengths = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  wtCustChordLengths[i]= Double.valueOf(a2[i]);
              }
            break; 
          case "wtCustReferenceAreas":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.wtCustReferenceAreas = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  wtCustReferenceAreas[i]= Double.valueOf(a2[i]);
              }
            break;
          case "wtCustReferenceMomR":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.wtCustReferenceMomR = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  wtCustReferenceMomR[i]= Double.valueOf(a2[i]);
              }
            break;
          case "newAF_CADAngles":
            a1="";a2 = new String[0];
            a1 = (String) data.get(j);
            a1=a1.replaceAll("\\{", "");
            a1=a1.replaceAll("\\}", "");
            a2 = a1.split(",");
            this.newAF_CADAngles = new double[a2.length];
            for(int i=0;i<a2.length;i++){
                newAF_CADAngles[i]= Double.valueOf(a2[i]);
            }
            break;  
          case "newAF_CustomMesh":
              a1= ""; a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a1=a1.replaceAll("\\s","");
              a2 = a1.split(",");
              this.newAF_CustomMesh = new boolean[a2.length];
              for(int i=0;i<a2.length;i++){
                  newAF_CustomMesh[i] =
                          Boolean.valueOf(a2[i].replaceAll("\\s",""));
              }
              break;
          case "newRTR_SpeedName":
              a1= ""; 
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              this.newRTR_SpeedName = a1.split(",");
              for(int i=0;i<newRTR_SpeedName.length;i++){
                newRTR_SpeedName[i]=newRTR_SpeedName[i].replaceAll("\"","");
              }
              break;
          case "newRTR_Speed":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newRTR_Speed = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newRTR_Speed[i]= Double.valueOf(a2[i]);
              }
              break;    
           case "forceRTR_MRF":
              this.forceRTR_MRF = Boolean.parseBoolean(data.get(j));
              break;   
          case "custRTR_MeshName":
              a1= ""; 
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              this.custRTR_MeshName = a1.split(",");
              for(int i=0;i<custRTR_MeshName.length;i++){
                custRTR_MeshName[i]=custRTR_MeshName[i].replaceAll("\"","");
              }
              break;
          case "newCust_TargetSize":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newCust_TargetSize = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newCust_TargetSize[i]= Double.valueOf(a2[i]);
              }
              break;
          case "newCust_MinSize":
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newCust_MinSize = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newCust_MinSize[i]= Double.valueOf(a2[i]);
              }
              break;
          case "newCust_NPtsOnCircle":
              if(_input_debug){
                simu.println("newCust_NPtsOnCircle");
              }
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newCust_NPtsOnCircle = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newCust_NPtsOnCircle[i]= Double.valueOf(a2[i]);
              }
             break;
          case "newCust_PrismAbsThick":
              if(_input_debug){
                simu.println("newCust_PrismAbsThick");
              }
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newCust_PrismAbsThick = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newCust_PrismAbsThick[i]= Double.valueOf(a2[i]);
              }
              break;
          case "newCust_1stCellThick":
              if(_input_debug){
                simu.println("newCust_1stCellThick");
              }
              a1="";a2 = new String[0];
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              a2 = a1.split(",");
              this.newCust_1stCellThick = new double[a2.length];
              for(int i=0;i<a2.length;i++){
                  newCust_1stCellThick[i]= Double.valueOf(a2[i]);
              }
              break;
          case "newCust_NPrismLayers":
              a1="";
              if(_input_debug){
                simu.println("newCust_NPrismLayers");
              }
              a1 = (String) data.get(j);
              a1=a1.replaceAll("\\s", "");
              a1=a1.replaceAll("\\{", "");
              a1=a1.replaceAll("\\}", "");
              if(_input_debug){
                simu.println("a1 = " + a1);
              }
              String[] a3 = a1.split(",");
              if(_input_debug){
                simu.println("a3 = " + a3);
              }
              this.newCust_NPrismLayers = new int[a3.length];
              for(int i=0;i<a3.length;i++){
                  newCust_NPrismLayers[i]= Integer.valueOf(a3[i]);
                  newCust_NPrismLayers[i] = Integer.valueOf(a3[i]);
              }
              if(_input_debug){
                simu.println("done");
              }
              break;
          default:
              System.out.println("variable not found");
        }
      }
    }
}
