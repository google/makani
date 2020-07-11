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
package AeroCFDRunner;

import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.meshing.*;
import star.flow.*;

import GeometricObject.*;
import MeshTools.*;
import PhysicsTools.*;
import Domain.*;
import Naming.*;
import Tools.*;
import Solvers.*;
import java.io.File;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import star.base.report.*;
import star.resurfacer.*;
import star.meshing.*;
import star.prismmesher.*;
import star.trimmer.*;
import star.energy.StaticTemperatureProfile;
import star.motion.*;
import star.turbulence.TurbulenceIntensityProfile;
import star.turbulence.TurbulentViscosityRatioProfile;
import star.vis.*;
import star.walldistance.*;

public class RotorAnalysis{

    //Simulation
    Simulation simu;
    Representation proxyRep;
    NamingConvention globalNames;
    
    // Domain string identifier List
    String inStr;
    String outStr;
    String fsStr;
    String symmStr;
    String wallStr;
    String[] allDomStr = {inStr,outStr,fsStr,symmStr,wallStr};
    
    // Type of simulation
    boolean isFS  = false; // Freestream flag
    boolean isUns = false; // Unsteady flag

    // Desired alpha/betas
    double angleOfAttack=0.0;
    double sideSlipAngle=0.0;
    
    //Geometry
    Collection<GeometryPart> allGeomParts;
    ArrayList<String> airfoilStrList;

    // geometric configuration information
    double lengthScale;
    
    //Domain Objects
    String mainDomainName;
    Domain mainDomain;
    ArrayList<Domain> allDomainObjects=new ArrayList();
    ArrayList<OversetDomain> allOversetDomains=new ArrayList();
    ArrayList<RotatingDomain> allRotatingDomains=new ArrayList();
    
    //Geometry Objects
    ArrayList<AerodynamicSurface> allAirfoilObjects=new ArrayList();
    ArrayList<Rotor> allRBMRotorObjects=new ArrayList();
    ArrayList<Rotor> allRotorObjects= new ArrayList();
    
    //Meshing
    double baseSize = 1.0;
    double alpha_0 = 0.0;
    double beta_0 = 0.0;

    // Physics
    CFD_Physics wtPhysics;
    double refRho;
    double refVel;
    double refTemp;
    double refMu;
    double refMa;
    double refArea;
    double refRe;
    
    //Default Freestream values
    double fsTVR=10.0;
    double fsTi=0.01;

    // Regions
    ArrayList<Region> allWTRegs = new ArrayList();

    
    // Coordinate Systems
    LabCoordinateSystem labCsys;
    CartesianCoordinateSystem bodyCsys;
    CartesianCoordinateSystem inletCsys;

    ArrayList<CartesianCoordinateSystem> oversetCoordSystems;
    
    //Basic Stuff
    double[] xVector = {1.,0.,0.};
    double[] yVector = {0.,1.,0.};
    double[] zVector = {0.,0.,1.};
    double[] zeroVector = {0.,0.,0.};

    // Screen Res
    int[] outputRes= {2000,1000};
    double  viewAngleDistanceOffset = Math.atan(15.0 * Math.PI / 180.0);
    double[] wt2DCameraPos = {0., 0., 0.};
    
    //Annotations
    String caseAnnotationName="";
    ArrayList<Annotation> standardAnnotations=new ArrayList();
    
    //Statistic Sampling
    int statSamples=500;
    
    SolverDriver simDriver;
    
    public RotorAnalysis(Simulation simu,CoordinateSystem bodyCsys){
        //Simulation controls
        this.simu=simu;
        this.simDriver = new SolverDriver();
        this.proxyRep=SimTool.getSimProxy(simu,"Proxy");
        
        //Naming conventions
        this.globalNames = new NamingConvention();
        this.inStr   = globalNames.getInletStr();
        this.outStr  = globalNames.getOutletStr();
        this.fsStr   = globalNames.getFreeStr();
        this.symmStr = globalNames.getSymmStr();
        this.wallStr = globalNames.getWallStr();

        // Get all geometry parts in Tree
        this.allGeomParts = simu.getGeometryPartManager().getObjects();
        
        // Get coordinate systems
        this.labCsys = simu.getCoordinateSystemManager()
                .getLabCoordinateSystem();
        this.bodyCsys=(CartesianCoordinateSystem) bodyCsys;
        this.inletCsys=SimTool.getNestedCoordinate(bodyCsys, globalNames.getInletCsysName());

    }
    
    //=======================
    // PUBLIC METHODS
    //=======================
    //REFERENCE VALUES    
    public void setLengthScale(double newVal){
        lengthScale=newVal;
    }
    public void setMeshingBaseSize(double newValue){
        // Meshing values
        baseSize = newValue;
    }
    public void updateAllReferenceValues(Simulation simu, 
                                         double refRho,double refVel,double tipSpeed,
                                         double refTemp,double refMu,
                                         double  refMa,double  refRe,
                                         double refArea,double[] refMomR){
        // Update reference values.
        setRefVel(  refVel);
        setRefTemp(refTemp);
        setRefRho(  refRho);
        setRefMu(    refMu);
        setRefMach(  refMa);
        setRefRe(    refRe);
        setRefArea(refArea);

        //Set pressure coefficient
        SimTool.setPressureCoefficientFF(simu,refRho,0.0,tipSpeed);

        // skin friction coefficient
        SimTool.setSkinFrictionCoefficientFF(simu,refRho,tipSpeed);

        //Angle of attack
        // controlled first by rotate assembly
        //  then overwritten by main overset domain if it exists
        String assemblyOpName="Rotate Assembly";
        alpha_0=MeshOpTool.getRotationAngle(simu,assemblyOpName);
        
        // Rotors
        double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
        double molAir=.0289645; // [kg/mol] molar gas constant of air
        double refPress=101325.0; // [Pa] reference pressure for air
        double soundSpeed=Math.sqrt(1.4*(airR/molAir)*refTemp);
        for(Rotor tmpRotor:allRotorObjects){
            tmpRotor.setSpeedofSound(soundSpeed);
            tmpRotor.setReferenceValues(refRho, tipSpeed);
        }

    }

    //MAIN CODE SECTIONS
    public void runPreProc(Simulation tmpSim,boolean justGetNames){
        simu.println("RTR PRE: Running preprocessing.");
        simu.println("  ");
        //Create physics so that Objects can be made easier
        //======================================================
        // Instantiate Objects 
        //
        // Note: Mesh Operations are Set Up prior to object
        //       creation so that the objects can associate the
        //       operation on their creation
        //
        // Meshing Note: We create the domain object mesh pipleine
        //       first and then backfill the remaining objects
        //       this allows a more streamlined way to instantiate.
        //
        //======================================================
        // User created Parts using mesh operations are not allowed
        //  to be used as base-level geometry
        allGeomParts=removeMeshOpParts(allGeomParts);
        //
        // VOLUME CONTROLS
        //
        ArrayList<GeometryPart> volumeControlParts=getVolumeControlParts();
        CartesianCoordinateSystem assmbCoord=makeCADZeroBodyCSys();
        
        //
        // FIXED ANGLE OVERSET DOMAINS
        //
        //   Overset domains are a bit tricky because they use Body Csys to
        //     understand their angle orientation since an overset domain that
        //     is translated in Regions does not have an exposed angle
        //     orientation when reloaded.
        //
        //   The usual use case for these are for airfoils.
        //
        simu.println("RTR PRE: Creating overset domains.");
        instantiateOversetDomains();
        ArrayList<GeometryPart> oversetParts=new ArrayList();
        for(OversetDomain tmpDom:allOversetDomains){
            oversetParts.add(tmpDom.getDomainPart());
        }
        
        if(allOversetDomains.size()>0){
          simu.println("RTR PRE: OversetDomain(s) detected...");
          simu.println("         ...Setting all SimpleBlockParts");
          simu.println("            to CAD Zero Body coordinates.");
          //If pre-processing, the Overset CAD coordinate angle
          //  must be returned to 0 degree/deflection orientation
          for(OversetDomain tmpDom:allOversetDomains){
              tmpDom.zeroOutDomainCsys();
              tmpDom.determineDomainAngle();
          }
          // Force all SimplePartVolume Controls to be CAD Zero based.
          for(GeometryPart tmpPart:volumeControlParts){
              if(tmpPart instanceof SimpleBlockPart){
                  CartesianCoordinateSystem cadZero = 
                          SimTool.getLabBasedCoordinate(simu,"CAD Zero "
                                  +bodyCsys.getPresentationName());
                  ((SimpleBlockPart) tmpPart).setCoordinateSystem(cadZero);
              }
          }
          // Update base size for mesh if necessary.
          for(OversetDomain tmpDom:allOversetDomains){
              tmpDom.updateMeshBaseSize(baseSize);
          }

          // Cycle through all overset domains to generate volume controls.
          for(GeometryPart tmpPart:volumeControlParts){
              String groupID=getVCGroupID(tmpPart);
              String vcPctStr=getVCPctStr(tmpPart,groupID);
              double vcPct=Double.valueOf(vcPctStr);
              for(OversetDomain tmpDom:allOversetDomains){
                  AutoMeshOperation pBMO = tmpDom.getDomainVolumeMeshOp();
                  VolumeCustomMeshControl groupVC=
                          MeshOpTool.getVolumeControl(pBMO,groupID+" "
                                  +vcPctStr);
                  groupVC.getGeometryObjects().add(tmpPart);
                  MeshOpTool.custVCIsoSize(groupVC, "Relative", vcPct);
              }
          }
        }

        // END FIXED-ANGLE OVERSET DOMAINS
        simu.println("RTR PRE: End Fixed-Angle Overset Domains");

        //
        // ROTATING DOMAINS
        //
        //  There are two types of rotating domains: sliding & overset (TBR)
        //  The overset rotating domain is a different animal than the fixed
        //    rotation angle overset domain in that its motion and angles are
        //    controlled by the Motion or Reference frame. Therefore, it is 
        //    treated differently than its airfoil overset domain brethren.
        //
        //  Identify any sliding interface Geometry Parts.
        //
        //    Note: a sliding interface part is comprised of only Surfaces using
        //          the global name for rotating boundaries.
        String slidingIntID = globalNames.getSlidingStr();
        ArrayList<String> tmpIntStrList = new ArrayList();
        tmpIntStrList.add(slidingIntID);
        ArrayList<GeometryPart> slidingIntParts=
                MeshOpTool.findGeometryParts(simu,allGeomParts,tmpIntStrList);

        //  Identify any GeometryPart that contains a Rotor ID
        String rotorID = globalNames.getRotorID();
        ArrayList<String> tmpRotorIDList = new ArrayList();
        tmpRotorIDList.add(rotorID);
        ArrayList<GeometryPart> rotorParts=
                MeshOpTool.findGeometryParts(simu,allGeomParts,tmpRotorIDList);

         // Remove parts that have a rotor component in them
         //  but are not pure rotors.
        rotorParts=MeshOpTool.filterOutGeometryParts(simu,rotorParts,rotorID);

        //  Create new CAD mesh operation Part from the interface and rotor 
        //    and any other non-special geometry parts: VC_, OS_
        ArrayList<GeometryPart> staticParts = new ArrayList();
        staticParts.addAll(allGeomParts);

        // remove specific special part categories.
        staticParts.remove(getRawDomainGeometryPart());
        ArrayList<GeometryPart> anySpecialPart = new ArrayList();
        ArrayList<String> specialNames = new ArrayList();
        anySpecialPart.addAll(MeshOpTool.findPartsByFirstName(allGeomParts, "VC_"));
        anySpecialPart.addAll(MeshOpTool.findPartsByFirstName(allGeomParts, "OS"));
        staticParts.removeAll(anySpecialPart);
        ArrayList<GeometryPart> rotatingParts=
                createRotatingDomainParts(
                        slidingIntParts,rotorParts,staticParts);
        simu.println("RTR PRE: Instantiating Rotating Domains.");
        instantiateRotatingDomains();

        //
        // WT Domain Based Object Mesh Based Turn Table
        //
        //  This is to allow any CAD objects that are subtracted from the
        //    main windtunnel domain to be rotated with change in angle of
        //    of attack.
        //
        simu.println("RTR PRE: Building turn table.");
        //  Gather all remaining windtunnel domain-based parts, e.g.:
        //    1) Airfoils using the remesher workflow.
        //    2) Rotating Domain Interfaces (Sliding rotating domains only)
        //  and put into the turn-table operation.
        makeCADZeroBodyCSys();
        ArrayList<GeometryPart> allRotatedParts=new ArrayList();
        allRotatedParts.addAll(slidingIntParts);
        allRotatedParts.addAll(staticParts);

        // Don't put the rotor parts in the op below.
        // Doing so creates circular logic. 
        allRotatedParts.removeAll(rotorParts);
        
        simu.println("RTR PRE: Making rotating domain surface ops.");
        // Surface meshes of rotating parts first for efficiency in alpha/beta.
        instantiateRotatingDomainSurfaceMeshOps();
        
        simu.println("RTR PRE: Filtering parts on the turn table.");
        if(allRotatedParts.size()>0){
            // VCs
            allRotatedParts.addAll(volumeControlParts);
            allRotatedParts.removeAll(getStationaryVolumeControlParts());

            // Rotors can add an Alpha or Beta component.
            //  Beta_0 is always rotated in Lab coord first.
            //  Alpha_0 comes off the rotated Body Csys.
            if(rotatingParts.size()>0){
                CartesianCoordinateSystem assmbCoord_beta=makeCADZeroBetaAngleBodyCSys();
                MeshOpTool.rotationTransform(simu,"Rotate Assembly - Beta",allRotatedParts,labCsys,new double[] {0.,0.0,-1.0});
                MeshOpTool.rotationTransform(simu,"Rotate Assembly",allRotatedParts,assmbCoord_beta,yVector);
            }else{
                MeshOpTool.rotationTransform(simu,"Rotate Assembly",allRotatedParts,assmbCoord,yVector);
            }
        }
        simu.println("RTR PRE: Making rotating domain volume ops.");
        
        // Now make the RotatingDomain Mesh Operations.
        instantiateRotatingDomainVolumeMeshOps();

        //
        // Main Windtunnel Part
        //
        // Note: The windtunnel domain must subtract all parts left over.
        simu.println("RTR PRE: Looking for WT geometry part.");
        ArrayList<GeometryPart> allWTParts = new ArrayList();
        GeometryPart domGeomPart=getRawDomainGeometryPart();
        String domGeomPartName=domGeomPart.getPresentationName(); //need to name based on Part
        allWTParts.add(domGeomPart);
        simu.println("RTR PRE: Parent domain part is "
                +domGeomPart.getPresentationName());
        allWTParts.addAll(slidingIntParts);
        allWTParts.addAll(staticParts);
        allWTParts.removeAll(rotorParts);
        // create finalized wind tunnel part
        //   (or not if there is nothing to subtract)
        GeometryPart wtPart;
        if(allWTParts.size()>1){
            SubtractPartsOperation wtSubOp = MeshOpTool
                .subtractOp(simu,"Subtract "+domGeomPartName,domGeomPartName,
                        domGeomPart,allWTParts);
            wtPart=MeshOpTool.getSubtractOpPart(simu,wtSubOp);
        }else{
            wtPart=domGeomPart;
        }
        mainDomainName=wtPart.getPresentationName(); //need to name based on wtPart

        // MAIN WIND TUNNEL DOMAIN
        //
        // The Final Volume Mesh Operation created is *Always* the Wind Tunnel
        //
        simu.println("RTR PRE: Setting up wind tunnel Domain.");
        createMainTunnelDomain(wtPart);
        mainDomain.getExistingDomainBoundaries();
        // Make sure allDomainObjects knows this domain exists!
        allDomainObjects.add(mainDomain);
        // Custom values for object default mesh settings.
        double domainTargetPct = 800.0;
        
        // Wind Tunnel Surface Mesh
        String meshOpName="SM "+mainDomainName;
        AutoMeshOperation wtSurfaceOp=MeshOpTool
                .surfMeshOp(simu,meshOpName,baseSize*20.0, 100.0, 12.5, 32.,1.3,false);

        mainDomain.setDomainSurfMeshOp(wtSurfaceOp);
        wtSurfaceOp.getInputGeometryObjects().setQuery(null);
        wtSurfaceOp.getInputGeometryObjects().setObjects(mainDomain.getDomainPart());
        SurfaceCustomMeshControl domainControl =
                MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(),"00X Domain");
        MeshOpTool.surfFilter(domainControl,2);
        MeshOpTool.surfCustTargSize(domainControl,"Relative",domainTargetPct);

        // Wind Tunnel Volume Mesh
        simu.println("RTR PRE: Making volume mesh operations.");
        CFD_TrimmerModel wtVolumeMeshOp = new CFD_TrimmerModel(simu,
                "VM "+mainDomainName,labCsys, baseSize);
        mainDomain.setDomainVolumeMeshOp(wtVolumeMeshOp.getVMOp());
        wtVolumeMeshOp.addPart(wtPart);
        
        //override default settings
        wtVolumeMeshOp.getVMOp().getDefaultValues()
                .get(PrismThickness.class).setAbsoluteSizeValue(refRe, MeshOpTool.getPrefUVec(simu));
        wtVolumeMeshOp.getVMOp().getDefaultValues()
                .get(MaximumCellSize.class).setRelativeSizeValue(25600.0);
        wtVolumeMeshOp.getVMOp().getDefaultValues()
                .get(PrismThickness.class).getAbsoluteSizeValue()
                .setValue(baseSize*10.0);

          simu.println("RTR PRE: Applying Volume Controls to WT domain");
          //Apply user based volume controls to windtunnel volume mesh operations
          //change any body csys based volume control SimplePart to Body csys at CAD zero
          for(GeometryPart tmpPart:volumeControlParts){
              String groupID=getVCGroupID(tmpPart);
              String vcPctStr=getVCPctStr(tmpPart,groupID);
              double vcPct=Double.valueOf(vcPctStr);
              VolumeCustomMeshControl groupVC=wtVolumeMeshOp.getVolumeControl(groupID+" "+vcPctStr);
              groupVC.getGeometryObjects().add(tmpPart);
              wtVolumeMeshOp.custVCIsoSize(groupVC, "Relative", vcPct);
          }
          simu.println("RTR PRE: Overset Volume Conrols to WT domain");
          //Apply overset based volume controls to windtunnel volume mesh operations
          for(OversetDomain tmpDomain:allOversetDomains){
              GeometryPart osPart = tmpDomain.getDomainPart();
              String osName = osPart.getPresentationName();
              VolumeCustomMeshControl groupVC=wtVolumeMeshOp.getVolumeControl(osName);
              groupVC.getGeometryObjects().add(osPart);
              wtVolumeMeshOp.custVCIsoSize(groupVC, "Relative", 100.0);
          }
          
          simu.println("RTR PRE: Cleaning Up Default Boundary");
          //TODO: Fix this.
          //cleanupDefaultBoundary(allWTRegs);

          // ORGANIZE REMAINING NON-OBJECT PART SURFACES INTO BOUNDARIES
          // Put remaining surfaces into appropriate boundaries.
          simu.println("RTR PRE: Sorting remainder parts into boundaries.");
          for(Domain tmpDomain:allDomainObjects){
            simu.println("RTR PRE: For domain: " + tmpDomain.getName());
            Region tmpRegion = tmpDomain.getRegion();
            simu.println("RTR PRE: has region: " + tmpRegion.getPresentationName());
            ArrayList<PartSurface> allRemainingSurfaces  = new ArrayList();
            Collection<GeometryPart> parentPartObjs = tmpRegion.getPartGroup().getObjects();
            simu.println("RTR PRE: has " + parentPartObjs.size() + "parents");
            for(GeometryPart tmpPart:parentPartObjs){
                allRemainingSurfaces.addAll(tmpPart.getPartSurfaces());
                simu.println("RTR PRE: added all parent part surfaces");
            }
            ArrayList<PartSurface> allAssignedSurfaces = getAllInUsePartSurfaces(tmpRegion);
            allRemainingSurfaces.removeAll(allAssignedSurfaces);
            simu.println("RTR PRE: Unassigned surface count:"+allRemainingSurfaces.size());
            
            // all remaining surfaces not assigned to boundaries
            ArrayList<PartSurface> preFixRemainingSurfaces = new ArrayList();
            for(String tmpPre : globalNames.getAllFullConfigurationPreFixes()){
              simu.println("RTR PRE: Assigning prefix: "+tmpPre);
              for(PartSurface tmpSurf:allRemainingSurfaces){
                String tmpSurfName=tmpSurf.getPresentationName();
                simu.println("RTR PRE: Looking at surface: "+tmpSurfName);
                if(tmpSurfName.startsWith(tmpPre)||tmpSurfName.contains("."+tmpPre)){
                    preFixRemainingSurfaces.add(tmpSurf);
                }
              }
              addSurfaceToRegionGroupBoundary(tmpRegion,tmpPre,preFixRemainingSurfaces);
              preFixRemainingSurfaces.removeAll(preFixRemainingSurfaces);
            }
          }

          // Note: Now that volume mesh operations are completed, we can
          //       propogate object settings into the pipeline

          // ROTORS
          simu.println("RTR PRE: Instantiating Rotor Objects");
          instantiateRotorObjects();
          instantiateRotorSurfaceMeshOps();

        // Will remove all the things that have nothing to do w/ getting Rotor Names.
        if(justGetNames){
          tmpSim.get(MeshOperationManager.class).removeObjects((tmpSim.get(MeshOperationManager.class).getObjects()));
          tmpSim.getRegionManager().deleteChildren(tmpSim.getRegionManager().getObjects());
          
          tmpSim.println("RTR PRE: Just setting up for getting names. Deleting trees.");

        }else{  
          // Now we add the rotor geometry Parts into the rotation operations.
          for(Rotor tmpRotor:allRBMRotorObjects){
              GeometryPart geometryPart = tmpRotor.getGeometryPart();
              MeshOpTool.getRotationOperation(simu,"Rotate Assembly - Beta").getInputGeometryObjects().add(geometryPart);
              MeshOpTool.getRotationOperation(simu,"Rotate Assembly").getInputGeometryObjects().add(geometryPart);

              // Special rotor wake volume controls.
              GeometryPart rotorWakePart=tmpRotor.createRBMRotorWakePart();
              volumeControlParts.add(rotorWakePart);

              // Put into the mesh operation.
              MeshOpTool.getRotationOperation(simu,"Rotate Assembly - Beta").getInputGeometryObjects().add(rotorWakePart);
              MeshOpTool.getRotationOperation(simu,"Rotate Assembly").getInputGeometryObjects().add(rotorWakePart);

              // Get Windtunnel on-board with it.
              String groupID=getVCGroupID(rotorWakePart);
              String vcPctStr=getVCPctStr(rotorWakePart,groupID);
              double vcPct=Double.valueOf(vcPctStr);
              VolumeCustomMeshControl groupVC=wtVolumeMeshOp.getVolumeControl(groupID+" "+vcPctStr);
              groupVC.getGeometryObjects().add(rotorWakePart);
              CFD_TrimmerModel.custVCIsoSize(groupVC, "Relative", vcPct);
          }

          instantiateRotorVolumeMeshOps();
          applyRotorStandardMeshSettings(allRBMRotorObjects);
          for(Rotor tmpRotor:allRBMRotorObjects){
              AutoMeshOperation meshOp
                      =tmpRotor.getRotatingDomain().getDomainVolumeMeshOp();
              AutoMeshOperation surfOp
                      =tmpRotor.getRotatingDomain().getDomainSurfMeshOp();
          }

          //ADDITIONAL WIND TUNNEL MESH STUFF
          // FARFIELD TARGET SIZE > 5*ROTOR DIAMETERS
          //  otherwise tiny cells in far field
          double newPct=domainTargetPct;
          double farFieldToRotorRatio=10.;
          // Based on surface size.
          for(Rotor tmpRotor:allRotorObjects){
              double rotorD=2.*tmpRotor.getBladeRadius();
              double farFieldLength=baseSize*newPct/100.0;
              while(farFieldLength<rotorD*farFieldToRotorRatio){
                  newPct=newPct*2.0;
                  farFieldLength=baseSize*newPct/100.0;
              }
          }
          // Make sure that primary prism layer total cell thickness is not
          //  zero. Also, need to check the max cell size and reduce pct to
          //  meet max cell size, otherwise it makes elongated prisms.
          double domainPct=mainDomain.getDomainVolumeMeshOp().getDefaultValues()
                  .get(MaximumCellSize.class).getRelativeSizeValue();
          if(domainPct<newPct) newPct=domainPct;

          SurfaceCustomMeshControl tmpTrimmerCntrl;
          SurfaceCustomMeshControl tmpSurfCntrl;
          
          // Custom mesh setting for a Main Wing
          String wingSurfaceControlTitle =  "" + globalNames.getMainWingID() +" MainWing";
          tmpSurfCntrl
                  = MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(), wingSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpSurfCntrl,1);
          MeshOpTool.surfCustTargSize(tmpSurfCntrl,"Relative",12.5);
          MeshOpTool.surfCustMinSize(tmpSurfCntrl, "Relative", 3.125);
          tmpTrimmerCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainVolumeMeshOp(), wingSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpTrimmerCntrl,1);
          MeshOpTool.surfPrismNumLayers(tmpTrimmerCntrl, 10);
          MeshOpTool.surfPrismNearWall(tmpTrimmerCntrl, 1.0E-4);
          MeshOpTool.surfPrismThick(tmpTrimmerCntrl, "Absolute", 0.02);     
          
          // Custom mesh setting for a Flaps.
          String flapsSurfaceControlTitle =  "" + globalNames.getFlapsID() +" Flaps";
          tmpSurfCntrl
                  = MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(), flapsSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpSurfCntrl,1);
          MeshOpTool.surfCustTargSize(tmpSurfCntrl,"Relative",12.5);
          MeshOpTool.surfCustMinSize(tmpSurfCntrl, "Relative", 3.125);
          tmpTrimmerCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainVolumeMeshOp(), flapsSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpTrimmerCntrl,1);
          MeshOpTool.surfPrismNumLayers(tmpTrimmerCntrl, 10);
          MeshOpTool.surfPrismNearWall(tmpTrimmerCntrl, 1.0E-4);
          MeshOpTool.surfPrismThick(tmpTrimmerCntrl, "Absolute", 0.02);
          
          // Custom mesh setting for a Pylon.
          String pylonSurfaceControlTitle =  "" + globalNames.getPylonID() +" Pylons";
          tmpSurfCntrl
                  = MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(), pylonSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpSurfCntrl,1);
          MeshOpTool.surfCustTargSize(tmpSurfCntrl,"Relative",25.0);
          MeshOpTool.surfCustMinSize(tmpSurfCntrl, "Relative",3.125);
          tmpTrimmerCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainVolumeMeshOp(), pylonSurfaceControlTitle);
          MeshOpTool.surfFilter(tmpTrimmerCntrl,1);
          MeshOpTool.surfPrismNumLayers(tmpTrimmerCntrl, 10);
          MeshOpTool.surfPrismThick(tmpTrimmerCntrl, "Absolute", 0.05);

          // Custom surface and prism layer controls for farfield.
          tmpSurfCntrl = MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(),"00X Domain");
          MeshOpTool.surfCustTargSize(tmpSurfCntrl, "Relative", newPct);
          tmpTrimmerCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainVolumeMeshOp(),"00X Domain");
          MeshOpTool.surfFilter(tmpTrimmerCntrl,2);
          MeshOpTool.surfCustTargSize(tmpTrimmerCntrl, "Relative", newPct);
          MeshOpTool.surfPrismOverride(tmpTrimmerCntrl, true);
          MeshOpTool.surfPrismNumLayers(tmpTrimmerCntrl, 2);
          MeshOpTool.surfPrismNearWall(tmpTrimmerCntrl, 0.5*newPct*baseSize);
          MeshOpTool.surfPrismThick(tmpTrimmerCntrl, "Relative", newPct);

          //  WT interface for each RBM rotor.
          for(Rotor tmpRotor:allRBMRotorObjects){
              String rotorName = tmpRotor.getName();
             
              // Rotor surface and volume mesh operations.
              Domain rotorDomain = tmpRotor.getRotatingDomain();
              SurfaceCustomMeshControl rtrSurfOpSurfCntrl = 
                      MeshOpTool.surfControl(rotorDomain.getDomainSurfMeshOp(),"009 Sliding Int");
             
              // Calculate surface size on opposite rotor interface and default
              //  setting surface size. adjust WT surface sizes and prism to
              //  match rotor settings as close as feasible
              //  (if mismatched base sizes)
              double rtrSurfPct=MeshOpTool.getSurfCustTargetSize(rtrSurfOpSurfCntrl);
              double rtrSurfBase=mainDomain.getDomainSurfMeshOp().getDefaultValues()
                      .get(BaseSize.class).getInternalValue();
              double rtrSurfAbsValue = rtrSurfPct/100.0*rtrSurfBase;
              double wtSurfBaseSize = mainDomain.getDomainSurfMeshOp().getDefaultValues()
                      .get(BaseSize.class).getInternalValue();
              double surfIntTargSize=100.0;
              if((rtrSurfAbsValue-wtSurfBaseSize*surfIntTargSize/100.0)<1e-6){ //if smaller on rotor side shrink
                  simu.println("RTR PRE: WT Int size larger than rotor side. Grow.");
                  while((rtrSurfAbsValue-wtSurfBaseSize*surfIntTargSize/100.0)<1e-6){
                      surfIntTargSize=surfIntTargSize/2.0;
                  }
              }else if((rtrSurfAbsValue-wtSurfBaseSize*surfIntTargSize/100.0)>1e-6){ //if larger on rotor side grow
                  simu.println("RTR PRE: WT Int size larger than rotor side. Shrink.");
                  while((rtrSurfAbsValue-wtSurfBaseSize*surfIntTargSize/100.0)>1e-6){
                      surfIntTargSize=surfIntTargSize*2.0;
                  }
              }

              // SURFACE MESH Setup get interface surfaces and assign to the controls
              SurfaceCustomMeshControl wtIntSurfCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainSurfMeshOp(),"INT "+rotorName);
              MeshOpTool.surfCustTargSize(wtIntSurfCntrl, "Relative", surfIntTargSize);
              ArrayList<PartSurface> tmpList = new ArrayList();
              for(PartSurface tmpSurf:mainDomain.getDomainPart().getPartSurfaces()){
                  if(tmpSurf.getPresentationName()
                          .contains("INT_"+rotorName.substring(4)+"."+globalNames.getSlidingStr())){
                      tmpList.add(tmpSurf);
                  }
              }
              wtIntSurfCntrl.getGeometryObjects().addObjects(tmpList);
              MeshOpTool.surfNCircle(wtIntSurfCntrl, 360.0);
              MeshOpTool.surfGrowthRate(wtIntSurfCntrl, 1.05);

              // VOLUME MESH 
              // Need to generate 2 prism layers on the Interface Boundaries
              //  interface surface size *on wind tunnel side*
              SurfaceCustomMeshControl volIntCntrl
                  =MeshOpTool.surfControl(mainDomain.getDomainVolumeMeshOp(),"INT "+rotorName);
              volIntCntrl.getGeometryObjects().addObjects(tmpList);
              MeshOpTool.surfCustTargSize(volIntCntrl, "Relative", surfIntTargSize); //adjust interface region
              MeshOpTool.surfPrismOverride(volIntCntrl,true); //default sliding is slip
              MeshOpTool.surfPrismNumLayers(volIntCntrl, 2);
              PrismThickness prismThickObj = MeshOpTool.surfPrismThick(volIntCntrl, "Relative", surfIntTargSize * 2.0);

              double nearWallPrism = prismThickObj.getAbsoluteSizeValue().getInternalValue() / 2.0;
 
              MeshOpTool.surfPrismNearWall(volIntCntrl, nearWallPrism);

              // Allow rotor based volume controls to adjust prism layer settings on interface
              AutoMeshOperation volMesh = mainDomain.getDomainVolumeMeshOp();
              Collection<CustomMeshControl> allCntrls 
                      = (Collection<CustomMeshControl>) volMesh.getCustomMeshControls().getObjects();
              simu.println(allCntrls);
              for(CustomMeshControl tmpCntrl:allCntrls){
                  String tmpName = tmpCntrl.getPresentationName();
                  if(tmpName.startsWith(tmpRotor.getName().substring(4))){ //special rotor volume control
                      //get relative size of isotropic refinement
                      VolumeCustomMeshControl rtrCntrl 
                              = MeshOpTool.getVolumeControl(volMesh, tmpName);
                      MeshOpTool.setVCNumPrisms(rtrCntrl, 2);
                      double volumeIsoCellPct
                              = MeshOpTool.getCustVCIsoRelSize(rtrCntrl);
                      double wtVolBaseSize = mainDomain.getDomainVolumeMeshOp().getDefaultValues()
                          .get(BaseSize.class).getInternalValue();
                      MeshOpTool.setVCPrismNearWallThick(rtrCntrl,wtVolBaseSize*volumeIsoCellPct/100.);
                      MeshOpTool.setVCPrismTotalRelativeThickness(rtrCntrl, volumeIsoCellPct*2.0);
                  }
              }
          }

          //
          // REGIONS CLEANUP
          //
          //Clean up the Default boundary (this should be empty anyways)
          simu.println("RTR PRE: Regions all set up. Removing empty boundaries.");
          cleanupEmptyBoundaries(allWTRegs);

        }
    }


    private void initAnnotations(){
        Annotation tmpAnn;
        //annotations
        tmpAnn=getTextAnnotation("Conditions","Conditions:",0.035,new double[]{0.1986, 0.89, 0.0},true);
        tmpAnn.setBackground(false);
        standardAnnotations.add(tmpAnn);
        tmpAnn=getTextAnnotation("Wind Speed","\n\u2022 Wind Speed: "+refVel+ " m/s",0.06,new double[]{.21, 0.81, 0.0},true);
        tmpAnn.setBackground(false);
        standardAnnotations.add(tmpAnn);
        tmpAnn=getTextAnnotation("Body Alpha","\n\u2022 Body Alpha: "+angleOfAttack+" deg",0.06,new double[]{.21, 0.775, 0.0},true);
        tmpAnn.setBackground(false);
        standardAnnotations.add(tmpAnn);
        tmpAnn=getTextAnnotation("Body Beta", "\n\u2022 Body  Beta: " +sideSlipAngle+" deg",0.06,new double[]{.21, 0.740,0.},true);
        tmpAnn.setBackground(false);
        standardAnnotations.add(tmpAnn);
        IterationAnnotation iterationAnnotation = 
            ((IterationAnnotation) simu.getAnnotationManager().getObject("Iteration"));
        iterationAnnotation.setShowTimeStep(true);
        iterationAnnotation.setShowPhysicalTime(true);
        iterationAnnotation.setDefaultHeight(0.15);
        iterationAnnotation.setDefaultPosition(new DoubleVector(new double[] {0.660, 0.075, 0.}));
        standardAnnotations.add(iterationAnnotation);
        tmpAnn = caseNameAnnotation(caseAnnotationName);
        standardAnnotations.add(tmpAnn);
    }
    
    // annotations
    private SimpleAnnotation getTextAnnotation(String tmpName, String txtString,double defH, double[] newPosition,boolean needBckgrnd){
        SimpleAnnotation tmpAnn;
        try{
            tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().getObject(tmpName);
        }catch(NeoException e){
            tmpAnn = simu.getAnnotationManager().createSimpleAnnotation();
            tmpAnn.setPresentationName(tmpName);
        }
        tmpAnn.setText(txtString);
        tmpAnn.setDefaultHeight(defH);
        tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
        tmpAnn.setBackground(needBckgrnd);
       
        return tmpAnn;
    }
    private ImageAnnotation2D getImgAnnotation(String tmpName, String imgPath,double defH, double[] newPosition){
        ImageAnnotation2D tmpAnn;
        try{
            tmpAnn = (ImageAnnotation2D) simu.getAnnotationManager().getObject(tmpName);
        }catch(NeoException e){
            tmpAnn = simu.getAnnotationManager().createImageAnnotation2D();
            tmpAnn.setPresentationName(tmpName);
        }
        tmpAnn.setFilePath(imgPath);
        tmpAnn.setDefaultHeight(defH);
        tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
        return tmpAnn;
    }
    private Annotation caseNameAnnotation(String caseName) {
      SimpleAnnotation caseAnnotation;
      try{
          caseAnnotation = (SimpleAnnotation) simu.getAnnotationManager().getObject("Case Name");
      }catch(NeoException e){
          caseAnnotation = 
              simu.getAnnotationManager().createSimpleAnnotation();
          caseAnnotation.setPresentationName("Case Name");
      }
      caseAnnotation.setText("Case: "+caseName);
      caseAnnotation.setDefaultHeight(0.05);
      caseAnnotation.setDefaultPosition(new DoubleVector(new double[] {0.0075, 0.0075, 0.0}));
      return caseAnnotation;
    }    
    public void setCaseAnnotationName(String newStr){
        caseAnnotationName=newStr;
    }

    private void applyRotorStandardMeshSettings(ArrayList<Rotor> allObjects){

            for(Rotor tmpRotor:allObjects){
                // custom Surface Mesh controls per rotor
                //   Best practice values for the big parts
                double bigAbsTargetSize = tmpRotor.getBladeRadius()/20.0;
                double bigGrowthRate = 1.05;
                //   Best practice values for the blades
                double bladeTargetPct   =  50.0;
                double bladeMinPct      =  3.125;
                double bladeNPtsCircle  =  54.0;
                double bladeTETargetPct =   bladeTargetPct*0.25;
                double bladeTEMinPct    =   bladeTargetPct*0.125;
                double bladeAbsPrismThick  = lengthScale*0.025;
                AutoMeshOperation tmpSurfOp =tmpRotor.getRotatingDomain()
                        .getDomainSurfMeshOp();
                AutoMeshOperation tmpVolOp  =tmpRotor.getRotatingDomain()
                        .getDomainVolumeMeshOp();
                tmpVolOp.getDefaultValues().get(PrismThickness.class)
                  .setAbsoluteSize(bladeAbsPrismThick, MeshOpTool.getPrefUVec(simu));
                //Adjust Default Values
                tmpSurfOp.getDefaultValues().get(SurfaceGrowthRate.class)
                        .setGrowthRate(bigGrowthRate);

                //SURFACE MESH SETTINGS
                //main blade surface control
                SurfaceCustomMeshControl surfSurfControl = tmpRotor.getSurfMeshCustomControl();
                tmpRotor.setSurfMeshCustomControl(surfSurfControl);
                MeshOpTool.surfFilter(      surfSurfControl,2);
                MeshOpTool.surfCustTargSize(surfSurfControl,"Relative",bladeTargetPct);
                MeshOpTool.surfCustMinSize( surfSurfControl,"Relative",bladeMinPct);
                MeshOpTool.surfNCircle(     surfSurfControl,bladeNPtsCircle);

                //trailing edges
                SurfaceCustomMeshControl surfSurfControl_TE = tmpRotor.getSurfMeshCustomTEControl();
                tmpRotor.setSurfMeshCustomTEControl(surfSurfControl_TE);
                MeshOpTool.surfFilter_TE(  surfSurfControl_TE,2);
                MeshOpTool.surfCustMinSize(surfSurfControl_TE,"Relative",bladeTEMinPct);
                MeshOpTool.surfEdgeProx(   surfSurfControl_TE,1);

                //interface surface size *on rotor side*
                double globalBase =tmpSurfOp.getDefaultValues()
                        .get(BaseSize.class).getInternalValue();
                double recMeshSize=tmpRotor.getRecommendedMeshSize();
                double bigTargPct=100.0;
                String bndyName = globalNames.getBndyName(globalNames.getSlidingStr());
                SurfaceCustomMeshControl intCntrl=MeshOpTool.surfControl(tmpSurfOp,bndyName);
                Collection<PartSurface> intSurfaces = 
                        tmpRotor.getParentRegion().getBoundaryManager()
                                .getBoundary(bndyName).getPartSurfaceGroup().getObjects();
                intCntrl.getGeometryObjects().addObjects(intSurfaces);
                //calculate correct percentage for interface
                if(bigAbsTargetSize>globalBase*bigTargPct/100.0){
                    while(bigAbsTargetSize>globalBase*bigTargPct/100.0){
                        bigTargPct=bigTargPct*2.0;
                    }
                    bigTargPct=bigTargPct/2.0;
                }
                
                tmpSurfOp.getDefaultValues().get(PartsTargetSurfaceSize.class).setRelativeSize(bigTargPct);
                MeshOpTool.surfCustTargSize(intCntrl,"Relative",bigTargPct);
                MeshOpTool.surfNCircle(intCntrl, 360.0);
                
                //VOLUME MESH SETTINGS
                // Defaults
                int nPrismLayers = 8; //use even #s
                double prismThickness=lengthScale*0.025;
                double firstCellThick=lengthScale*1e-5;

                //Custom volume control settings
                SurfaceCustomMeshControl volSurfControl= tmpRotor.getVolMeshCustomControl();
                tmpRotor.setVolMeshCustomControl(volSurfControl);
                //main body control
                MeshOpTool.surfFilter(volSurfControl,2);
                MeshOpTool.surfPrismThick( volSurfControl,"Absolute",prismThickness);
                MeshOpTool.surfPrismNearWall( volSurfControl,firstCellThick);
                MeshOpTool.surfPrismOverride( volSurfControl, true);
                MeshOpTool.surfPrismNumLayers(volSurfControl,nPrismLayers);
                tmpRotor.setVolMeshCustomControl(volSurfControl);
                //trailing edges
                SurfaceCustomMeshControl volSurfControl_TE = tmpRotor.getVolMeshCustomTEControl();
                MeshOpTool.surfFilter_TE(  volSurfControl_TE,2);
                MeshOpTool.surfPrismThick( volSurfControl_TE,"Absolute",prismThickness*0.25);
                MeshOpTool.surfPrismNearWall( volSurfControl_TE,firstCellThick*4.0);
                MeshOpTool.surfPrismOverride( volSurfControl_TE, true);
                MeshOpTool.surfPrismNumLayers(volSurfControl_TE,nPrismLayers);
                tmpRotor.setVolMeshCustomTEControl(volSurfControl_TE);
                
                //Need to generate 2 prism layers on the Interface Boundaries
                // interface surface size *on rotor side*
                String volBndyName = globalNames.getBndyName(globalNames.getSlidingStr());
                SurfaceCustomMeshControl volIntCntrl=MeshOpTool.surfControl(tmpVolOp,volBndyName);
                Collection<PartSurface> volIntSurfaces = 
                        tmpRotor.getParentRegion().getBoundaryManager()
                                .getBoundary(bndyName).getPartSurfaceGroup().getObjects();
                volIntCntrl.getGeometryObjects().addObjects(volIntSurfaces);
                MeshOpTool.surfCustTargSize(volIntCntrl, "Relative", bigTargPct);
                MeshOpTool.surfPrismOverride(volIntCntrl,true); //default sliding is slip
                MeshOpTool.surfPrismNumLayers(volIntCntrl, 2);
                PrismThickness prismThickObj = MeshOpTool.surfPrismThick(volIntCntrl, "Relative", bigTargPct*2.0);
                double nearWallPrism=prismThickObj.getRelativeSizeValue()/2.0;
                MeshOpTool.surfPrismNearWall(volIntCntrl,nearWallPrism);

                //Use the rotor as its own volume control to keep the volume
                // inside the rotor itself to base size/2 (1/4 blade chord)
                double rtrVcPCT=200.0;
                if(!tmpRotor.isBEM()){
                    VolumeCustomMeshControl tmpVC=MeshOpTool.getVolumeControl(tmpVolOp, "Standard Rotor VC");
                    tmpVC.getGeometryObjects().add(tmpRotor.getRotatingDomain().getDomainPart());
                    MeshOpTool.custVCIsoSize(tmpVC, "Relative", rtrVcPCT);
                }
                
            }
        }

    private void addSurfaceToRegionGroupBoundary(Region tmpReg,String bndyID,ArrayList<PartSurface> geomSurfs){
        Boundary tmpBndy = setUpBoundary(tmpReg,globalNames.getBndyName(bndyID));
        //add necessary surfaces to the boundary
        Collection<PartSurface> tmpColl = tmpBndy.getPartSurfaceGroup().getObjects();
        tmpColl.addAll(geomSurfs);
        tmpBndy.getPartSurfaceGroup().setObjects(tmpColl);
        //Set BC Type
        tmpBndy.setBoundaryType(WallBoundary.class);
    }
    
    // BOUNDARIES
    private void cleanupDefaultBoundary(Collection<Region> allRegs){
      simu.println("Cleaning up Default Boundaries");
      for(Region tmp:allRegs){

        String regName =tmp.getPresentationName();
        simu.println("for region:"+regName);

        for(Boundary tmpBnd:tmp.getBoundaryManager().getBoundaries()){
          if(tmpBnd.getPresentationName().equals("Default")){
            tmp.getBoundaryManager().remove(tmpBnd);
            break;
            }
          }
      }
    }
    private ArrayList<PartSurface> getAllInUsePartSurfaces(Region tmpReg){
        simu.println("RTR MSG: getting all in use part surfaces");
        simu.println("From : "+tmpReg.getPresentationName());
        ArrayList<PartSurface> inUseSurfaceList=new ArrayList();
        tmpReg.getBoundaryManager().getBoundaries();
        simu.println(tmpReg.getBoundaryManager().getBoundaries());

        for(Boundary tmpBnd:tmpReg.getBoundaryManager().getBoundaries()){
          simu.println(tmpBnd.getPartSurfaceGroup().getObjects());
          simu.println(tmpBnd.getPartSurfaceGroup().getObjects().isEmpty());
          simu.println(tmpBnd.getPartSurfaceGroup().getObjects().size());
          if(tmpBnd.getPartSurfaceGroup().getObjects().size()>0){
            inUseSurfaceList.addAll(tmpBnd.getPartSurfaceGroup().getObjects());
          }
        }
        simu.println("RTR MSG: retrieved "+inUseSurfaceList.size()+" in use surfaces");
        return inUseSurfaceList;
    }
    public void instantiateExistingObjects(){
        //Instantiate simulation objects
        
        //Instantiate main WT domain
        simu.println("RTR MSG: Checking for existing WT Domains.");
        instantiateWTDomain();
        simu.println("RTR RUNNER: AFTER WTDOMAIN MESHING UP TO DATE: "
                        +checkMeshOperationsUpToDate());
        //Instantiate overset domains
        instantiateOversetDomains();
        simu.println("RTR MSG: Overset Domain(s) instantiated.");

        //Instantiate rotating domains
        instantiateRotatingDomains();
        instantiateRotatingDomainSurfaceMeshOps();
        instantiateRotatingDomainVolumeMeshOps(); 
        simu.println("RTR MSG: Rotating Domain(s) instantiated.");

        //Rotors
        instantiateRotorObjects();
        instantiateRotorSurfaceMeshOps();
        instantiateRotorVolumeMeshOps();
        simu.println("RTR MSG: Associating custom mesh settings.");
        for(Rotor tmpRotor:allRBMRotorObjects){
            //
            simu.println("RTR MSG: Rotor "+tmpRotor.getName());
            tmpRotor.getAnySurfaceMeshCustomControlSettings();
            tmpRotor.getAnyVolumeMeshCustomControlSettings();
        }

    }
    
    public void initAllRBMRotorSpeeds(){
        for(Rotor tmpRotor:allRBMRotorObjects){
            tmpRotor.updateRotorSpeed(tmpRotor.getRotorSpeed());
        }
        
    }
    
    //GEOMETRY
    private GeometryPart getRawDomainGeometryPart(){
        /* Method to determine which part contains the domain */
        ArrayList<String> domainIdent = new ArrayList();
        domainIdent.add(inStr);    // inlet
        domainIdent.add(fsStr);    // freestream
        return MeshOpTool.findGeometryPart(simu,allGeomParts,domainIdent);
    }
    private Collection<GeometryPart> removeMeshOpParts(Collection<GeometryPart> theseGeomParts){
        ArrayList<MeshOperationPart> meshOpParts=new ArrayList();
        for(GeometryPart tmpPart:theseGeomParts){ //remove mesh operation parts
            if(tmpPart instanceof MeshOperationPart){
                meshOpParts.add((MeshOperationPart) tmpPart);
            }
        }
        theseGeomParts.removeAll(meshOpParts);
        return theseGeomParts;
    }
    private ArrayList<GeometryPart> getRawOversetParts(){
        Collection<GeometryPart> rawGeomList = allGeomParts;
        ArrayList<String> oversetIdent = new ArrayList();
        oversetIdent.add(globalNames.getOversetStr()); // oversetprefix
        return MeshOpTool.findGeometryParts(simu,rawGeomList,oversetIdent);
    }
    private ArrayList<GeometryPart> getRawRotatingDomainParts(){
        Collection<GeometryPart> rawGeomList = simu.getGeometryPartManager().getObjects();
        ArrayList<GeometryPart> retGeomList = new ArrayList();
        ArrayList<String> slidingIdent = new ArrayList();
        slidingIdent.add(globalNames.getSlidingStr()); // rotatingprefix
        retGeomList.addAll(MeshOpTool.findGeometryParts(simu,rawGeomList,slidingIdent));
        // Need to remove any raw Geometry Parts + wtDomainParts;
        //  only want CAD operation Parts.
        retGeomList.remove(getRawDomainGeometryPart());
        ArrayList<GeometryPart> removalList = (ArrayList<GeometryPart>) retGeomList.clone();
        for(GeometryPart tmpPart:retGeomList){
            if(!(tmpPart instanceof MeshOperationPart)){
                removalList.remove(tmpPart);
            }
        }
        retGeomList=removalList;
        return retGeomList;
    }
    private void createMainTunnelDomain(GeometryPart wtPart){
        mainDomain = new Domain(simu,wtPart);
        simu.println("RTR MSG: Setting up windtunnel region.");
        mainDomain.setUpRegion(simu);
        simu.println("RTR MSG: Setting up windtunnel boundaries.");
        mainDomain.initPartBoundaries();
        Region wtRegion=mainDomain.getRegion();
        allWTRegs.add(wtRegion);
    }
    private String getVCGroupID(GeometryPart tmpPart){
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
        simu.println("Found VC: "+tmpPartName);
        return tmpPartName.substring(firstIndx+1,secondIndx);//removes _
    }
    private String getVCPctStr(GeometryPart tmpPart,String groupID){
        char sepChar='_';
        int subIndx;       //start of PCT portion of string
        String tmpPctStr; //PCT and possible description part of string
        String tmpPartName=tmpPart.getPresentationName();
        simu.println("Group ID: "+groupID);

        String tmpStr = "";
        if(tmpPartName.startsWith(globalNames.getVCPreFix()+"_")){
            tmpStr = globalNames.getVCPreFix()+sepChar+groupID+sepChar;
        }else if(tmpPartName.startsWith(globalNames.getStatVCPreFix()+"_")){
            tmpStr = globalNames.getStatVCPreFix()+sepChar+groupID+sepChar;
        }
        
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
        simu.println("Found VC Pct : "+tmpPctStr);
        //convert tmpPctStr to a percentage value
        return tmpPctStr;
    }
    private ArrayList<GeometryPart> createRotatingDomainParts(ArrayList<GeometryPart> intParts,ArrayList<GeometryPart> rotorParts,ArrayList<GeometryPart> staticParts){
        if(intParts.size()!=rotorParts.size()){
            simu.println("WT ERR: Fail - rotating Parts from unequal lists.");
            return null;
        }
        ArrayList<GeometryPart> allRotatingDomainParts=new ArrayList();
        for(GeometryPart tmpRotPart:rotorParts){
            String rotPartName=tmpRotPart.getPresentationName();
            for(GeometryPart tmpIntPart:intParts){
                String intPartName=tmpIntPart.getPresentationName();
                if(intPartName.substring(4).equals(rotPartName)){
                    simu.println("Making rotor with :" +rotPartName);
                    String newPartName = "RTR"+" "+rotPartName;
                    ArrayList<GeometryPart> tmpPartsList=new ArrayList();
                    tmpPartsList.add(tmpRotPart);
                    tmpPartsList.add(tmpIntPart);
                    tmpPartsList.addAll(staticParts);
                    //Parts are a match, make new MeshOpPart
                    SubtractPartsOperation tmpOp=
                            MeshOpTool.subtractOp(simu,newPartName, newPartName, tmpIntPart,tmpPartsList);
                    allRotatingDomainParts.add(MeshOpTool.getSubtractOpPart(simu,tmpOp));
                }
            }
        }
        return allRotatingDomainParts;
    }
    private RotatingDomain createRotatingDomain(GeometryPart rotPart){
        RotatingDomain rotDomain = new RotatingDomain(simu,rotPart);
        simu.println("RTR MSG: Setting up rotating region from Domain "
                +rotPart.getPresentationName());
        rotDomain.setUpRegion(simu);
        simu.println("RTR MSG: Setting up rotating region boundaries.");
        rotDomain.initPartBoundaries();
        Region rotRegion=rotDomain.getRegion();
        allWTRegs.add(rotRegion);
        return rotDomain;
    }

    
    public void generateSurfaceMesh(){
        //only want to remesh the main domain's surface mesh
        // overset will always be out of date based on region rotation
        mainDomain.getDomainSurfMeshOp().execute();
        simu.getMeshPipelineController().generateSurfaceMesh();
    }
    public void generateVolumeMesh(){
        //only want to remesh the main domain's surface mesh
        // overset will always be out of date based on region rotation
        mainDomain.getDomainVolumeMeshOp().execute();
        simu.getMeshPipelineController().generateVolumeMesh();
        Representation volMesh = simu.getRepresentationManager().getObject("Volume Mesh");
        SimTool.getSimProxy(simu,globalNames.getProxyName()).setRepresentation(volMesh);
    }
    public void modifyAlphaAndBeta(double[] newAngles){
        // If there is a turn table angle of the assembly that is being rotated
        //   we also need to rotate BodyCsys to the same angle
        // For rotors, this is only called during the meshing phase. So resetting
        //   the alpha and beta coordinate systems is allowable
        
        angleOfAttack = newAngles[0];
        sideSlipAngle = newAngles[1];
        
        // Need to reset the angle of the beta csys for the mesh operation
        //   then rotate about Z-axis to get the stability axis system
        CartesianCoordinateSystem assmbCoord_beta=makeCADZeroBetaAngleBodyCSys();
        if(Math.abs(newAngles[1])>1.e-6){
            SimTool.rotateLabBasedCoordinateSystem(simu, newAngles[1], new double[] {0.0,0.0,-1.0}, assmbCoord_beta);
        }
        
        boolean alphaRotationExists = false;
        boolean betaRotationExists = false;
        String assemblyAlphaOpName = "Rotate Assembly";
        String assemblyBetaOpName = "Rotate Assembly - Beta";
        try{
          MeshOpTool.setRotationAngle(simu,assemblyBetaOpName,newAngles[1]);
          MeshOpTool.setRotationAxis(simu,assemblyBetaOpName,new double[] {0.0,0.0,-1.0});
          betaRotationExists = true;

          MeshOpTool.setRotationAngle(simu,assemblyAlphaOpName,newAngles[0]);
          MeshOpTool.setRotationAxis(simu,assemblyAlphaOpName,yVector);
          alphaRotationExists = true;
        }catch(NeoException e){
            simu.println("WT MSG: No geometric mesh angle of attack found.");
        }
        
        //Force the Body Coordinate to align with stability coordinate and
        //  rotate to alpha
        bodyCsys.setBasis0(assmbCoord_beta.getBasis0());
        bodyCsys.setBasis1(assmbCoord_beta.getBasis1());
        //Rotate the body coordinate upward alpha
        SimTool.rotateCoordinateSystem( simu,newAngles[0],yVector,bodyCsys,bodyCsys);
        
        //set wind opposite of body, same y vector
        inletCsys.setBasis0(new DoubleVector(new double[] {-1.0,0.0,0.0}));
        inletCsys.setBasis1(new DoubleVector(new double[] {0.0,1.0,0.0}));

        //non-freestream cases, we always counter rotate the Velocity Inlet from
        //the **CAD ZERO** body Csys using standard Euler rotations
        if(!isFS){
            SimTool.rotateCoordinateSystem(simu,-newAngles[0],yVector,bodyCsys,inletCsys);
            SimTool.rotateCoordinateSystem(simu, newAngles[1],zVector,labCsys,inletCsys);
        }

        // At large rotor mesh angles, we would like the mesh outside of the
        // axial wake refinement zone to capture downstream convection of
        // tip vorticity; currently set to 20 degrees
        ArrayList<GeometryPart> highAngleMeshVCs = new ArrayList();
        if(Math.abs(angleOfAttack) >= 20.0){
          for(Rotor tmpRotor:allRBMRotorObjects){
            CartesianCoordinateSystem tmpCsys;
            CartesianCoordinateSystem rtrCsys;
            rtrCsys = tmpRotor.getRotorCoordSys();
            //transform coordinate from local rotor to lab csys
            double[] originVectorInBody = rtrCsys.getOriginVector().toDoubleArray();
            double[][] b2sa = SimTool.getR_Y(-angleOfAttack*Math.PI/180.);
            double[][] stab2aw = SimTool.getR_Z(sideSlipAngle*Math.PI/180.);
            double[] origin_in_stab = SimTool.transformVector(SimTool.get3x3Transpose(b2sa), originVectorInBody);
            double[] origin_in_wind = SimTool.transformVector(SimTool.get3x3Transpose(stab2aw),origin_in_stab);

            //identifier for each plane name "Lab Y Normal-"+rotorname+"origin"
            String vcName = globalNames.getVCPreFix()+"_"+tmpRotor.getName().substring(4)+" HighAngle_200.0";
            double alphaAngleRads = angleOfAttack*Math.PI/180.0;
            double bladeRadius = tmpRotor.getBladeRadius();
            double[] corner1 = {2.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[0],
                                2.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[1],
                                5.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[2]};
            double[] corner2 = {-5.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[0],
                                -2.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[1],
                                -2.0 * Math.sin(alphaAngleRads)*bladeRadius + origin_in_wind[2]};
            highAngleMeshVCs.add(GeometryTool.makeBlock(simu, vcName, labCsys, corner1, corner2));
          }
        } else{ // not a high angle mesh/rotor - get rid of the VC
          for(Rotor tmpRotor:allRBMRotorObjects){
            CartesianCoordinateSystem tmpCsys;
            CartesianCoordinateSystem rtrCsys;
            //identifier for each plane name "Lab Y Normal-"+rotorname+"origin"
            String vcName = globalNames.getVCPreFix()+"_"+tmpRotor.getName().substring(4)+" HighAngle_200.0";
            GeometryTool.deletePart(simu,vcName);
          }
        }
        
        // remove any of the high angle VCs from the rotation operations
        TransformPartsOperation alphaRotation;
        TransformPartsOperation betaRotation;
        if(alphaRotationExists){
          alphaRotation = MeshOpTool
                  .getRotationOperation(simu, assemblyAlphaOpName);
          Collection<GeometryObject> previousList = alphaRotation
                  .getInputGeometryObjects().getObjects();
          previousList.removeAll(highAngleMeshVCs);
          alphaRotation.getInputGeometryObjects().setObjects(previousList);
        }
        if(betaRotationExists){
          betaRotation = MeshOpTool
                  .getRotationOperation(simu, assemblyBetaOpName);
          Collection<GeometryObject> previousList = betaRotation
                  .getInputGeometryObjects().getObjects();
          previousList.removeAll(highAngleMeshVCs);
          betaRotation.getInputGeometryObjects().setObjects(previousList);
        }
        // Inject these high alpha beta mesh volume controls into the wind tunnel mesh operation
        // should not be necessary for rotor domains as they are already pretty finely meshed
        AutoMeshOperation pBMO = mainDomain.getDomainVolumeMeshOp();
        for(GeometryPart tmpVCPart : highAngleMeshVCs){
          String groupID=getVCGroupID(tmpVCPart);
          String vcPctStr=getVCPctStr(tmpVCPart,groupID);
          VolumeCustomMeshControl groupVC = MeshOpTool.getVolumeControl(pBMO, groupID + " " + vcPctStr);
          groupVC.getGeometryObjects().add(tmpVCPart);
          MeshOpTool.custVCIsoSize(groupVC, "Relative", 200.0);  
        }
    }

    //
    private Rotor getRotorWithName(String rtrName){
        Rotor retRotor=null;
        for(Rotor tmpRotor:allRotorObjects){
            simu.println("This one: "+tmpRotor.getName()+" vs. this one: "+rtrName);
            if(tmpRotor.getName().substring(4).equals(rtrName)){
                retRotor=tmpRotor;
                simu.println("Found rotor: "+retRotor.getName());
                break;
            }
        }
      return retRotor;
    }
    
    public ArrayList<String> getAllRotorNames(){
      ArrayList<String> retList = new ArrayList();
      for(Rotor tmpRotor:allRBMRotorObjects){
        retList.add(tmpRotor.getName());
      }
      return retList;
    }
    
    //
    public void setRotorSurfaceMeshSettings(String[] custRTR_MeshName,
        ArrayList<Double>  af_TargetSize, ArrayList<Double> af_MinSize,
        ArrayList<Double> af_NPtsOnCircle){
        //

        simu.println("Number of rotors: "+custRTR_MeshName.length);
        for(int i=0; i<custRTR_MeshName.length;i++){
            String customName=custRTR_MeshName[i];
            simu.println("RTR MSH: "+customName+" applying custom surface mesh.");
            Rotor tmpRotor=getRotorWithName(customName);
            simu.println("RTR MSH: Found rotor: "+tmpRotor.getName());
            simu.println(af_TargetSize);
            simu.println(af_MinSize);
            simu.println(af_NPtsOnCircle);
            if(!customName.equals("AutoDefault")&&!tmpRotor.isBEM()){
                //Set custom surface sizes
                tmpRotor.setCustomTargetSurfaceSize(af_TargetSize.get(i));
                tmpRotor.setCustomMinSurfaceSize(af_MinSize.get(i));
                tmpRotor.setCustomNPtsOnCircle(af_NPtsOnCircle.get(i));
            }
        }
    }
    //
    public void setRotorVolumeMeshSettings(String[] custRTR_MeshName,
        ArrayList<Boolean> af_LowYPlus,ArrayList<Boolean> af_HighYPlus,
        ArrayList<Double>  af_PrismAbsThick, ArrayList<Double> af_1stCellThick,
        ArrayList<Integer> af_NPrisms){
        //
        for(int i=0; i<custRTR_MeshName.length;i++){
            String customName=custRTR_MeshName[i];
            Rotor tmpRotor=getRotorWithName(customName);
            if(!customName.equals("AutoDefault")&&!tmpRotor.isBEM()){
                //Set custom prism mesh settings
                tmpRotor.setCustomFirstCellThickness(af_1stCellThick.get(i));
                tmpRotor.setCustomPrismAbsThickness(af_PrismAbsThick.get(i));
                tmpRotor.setCustomNumPrismLayers(af_NPrisms.get(i));
            }
        }
    }
    public void updateRotorSpeeds(String[] rtrNames, double[] newSpeeds){
        if(rtrNames.length != newSpeeds.length){
           simu.println("RTR ERR: updateRotorSpeeds. Arguments are not same length");
        }else{
            for(int i=0; i<rtrNames.length;i++){
                String customName = rtrNames[i];
                Rotor tmpRotor = getRotorWithName(customName);
                simu.println("RTR MSG: Getting Rotor " + customName);
                if(tmpRotor == null){
                    simu.println("Rotor is null!");
                }else{
                    simu.println("RTR MSG: Updating Rotor " + customName + " to speed: " + newSpeeds[i]);
                    tmpRotor.updateRotorSpeed(newSpeeds[i]);
                }
            }
        }
    }
    
    public boolean checkMeshOperationsUpToDate(){
        boolean upToDate=true;
        for(MeshOperation tmpOp:simu.get(MeshOperationManager.class).getObjects()){
            if(tmpOp instanceof AutoMeshOperation){
                upToDate= checkVolumeMeshUpToDate(tmpOp.getPresentationName());
                return upToDate;
            }
        }
        return upToDate;
    }
    
    private boolean checkVolumeMeshUpToDate(String opName){
        boolean retVal=!((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName)).isDirty();
        return retVal;
    }
    //PHYSICS
    public void instantiateExistingPhysics(String physicsName){
        if(wtPhysics==null){
            wtPhysics=new CFD_Physics(simu,physicsName);
        }
        for(Rotor tmpRotor:allRBMRotorObjects){
            isUns = CFD_Physics.isUnsteady(tmpRotor.getParentRegion().getPhysicsContinuum());
            if(isUns){
                break;
            }
        }
    }
    public void createOversetInterfaces(){
        /* method createOversetInterfaces
             creates all overset interfaces. Assumes that Physics continuua
             already exists
        */
        //Interface to all required domains
        for(Domain tmpDomain:allDomainObjects){
            if(tmpDomain instanceof OversetDomain){
                simu.println("WT MSG: Creating Overset interface for "+tmpDomain.getName());
                //interface to background domain
                ((OversetDomain) tmpDomain).interfaceToDomain(mainDomain);
                //extension to overlapping overset domains goes here
            }
        }
    }
    public void createSlidingInterfaces(){
        /* method createOversetInterfaces
             creates all overset interfaces. Assumes that Physics continuua
             already exists
        */
        
        //Interface to all required domains
        for(Domain tmpDomain:allDomainObjects){
            if(tmpDomain instanceof RotatingDomain){
                simu.println("WT MSG: Creating sliding interface for "+tmpDomain.getName());
                //interface to background domain
                ((RotatingDomain) tmpDomain).slidingInterfaceToDomain(mainDomain);
            }
        }
    }
    //
    //REGIONS
    public void applyBoundaryConditions(double refVel,double refTemp,
                                         double refMa,double fsPress){
        mainDomain.setAllInletBCs(refVel, refTemp, refMa,fsTVR,fsTi);
        mainDomain.setAllOutletBCs(fsPress,refTemp,fsTVR,fsTi);
    }

    //GEOMETRIC OBJECTS
    private void instantiateWTDomain(){
        /* Method instantiateWTDomains figures out what existing Domain objects
            already exist in the simulation and instantiates their objects.
            This method is strictly for extraction of existing objects in a sim.
            If there already domain objects in memory, the process is skipped.

            Domains include:
              WT Main Domain

        */
        if(allDomainObjects.isEmpty()){
            simu.println("WT MSG: Instantiating Domain Objects...");
            //Main WT Domain
            Collection<Region> allRegions=getRegions();
            GeometryPart mainWTPart=null;
            try{
                Region mainWTRegion = getMainWTRegion(allRegions);
                mainWTPart = getRegionPart(mainWTRegion);
            }catch(NeoException e){
                simu.println("WT MSG: Found pre-existing regions, no WT region detected.");
            }
            mainDomain = new Domain(simu,mainWTPart);
            mainDomainName = mainDomain.getName(); // let WT runner know you are here
            //
            mainDomain.setUpRegion(simu);
            mainDomain.getExistingDomainBoundaries();
            
            // Figure out type of domain
            isFS = mainDomain.isFreeStreamDomain();
            
            //Associate with Mesh Operations
            String meshOpName="SM "+mainDomain.getName();
            mainDomain.setDomainSurfMeshOp(
                    MeshOpTool.getAutoMeshOp(simu,meshOpName));
            meshOpName="VM "+mainDomainName;
            mainDomain.setDomainVolumeMeshOp(
                    MeshOpTool.getAutoMeshOp(simu,meshOpName));

            //Make sure WT Object knows you're here!
            allDomainObjects.add(mainDomain);

        }
        //Signal Domain objects that exist in this simulation
        for(Domain tmp:allDomainObjects){
            simu.println("WT MSG: Domain "+tmp.getName()+" instantiated.");
        }

    }
    private void instantiateRotorSurfaceMeshOps(){
        for(Rotor tmpRotor:allRBMRotorObjects){
            Domain tmpDomain=tmpRotor.getRotatingDomain();
            AutoMeshOperation domSurfMeshOp=tmpDomain.getDomainSurfMeshOp();
            String cntrlTitleStr=globalNames.getRotorBladeID()+"00 Rotor Blades";
            //Let airfoil know about its surface mesh controls
            SurfaceCustomMeshControl surfSurfControl
                    = MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr);
            tmpRotor.setSurfMeshCustomControl(surfSurfControl);
            SurfaceCustomMeshControl surfSurfControl_TE
                    = MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr+" TE");
            tmpRotor.setSurfMeshCustomTEControl(surfSurfControl_TE);
        }
    }
    private void instantiateRotorVolumeMeshOps(){
        
        for(Rotor tmpRotor:allRBMRotorObjects){
            
            Domain tmpDomain=tmpRotor.getRotatingDomain();
            //get associated automated surface and volume mesh operations
            //  that the domain is associated with
            AutoMeshOperation domVolMeshOp=tmpDomain.getDomainVolumeMeshOp();

            // Meshing
            String cntrlTitleStr=globalNames.getRotorBladeID()+"00 Rotor Blades";
            //Let airfoil know about its volume mesh controls
            SurfaceCustomMeshControl volSurfControl=MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr);
            tmpRotor.setVolMeshCustomControl(volSurfControl);
            SurfaceCustomMeshControl volSurfControlTE=MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr+" TE");
            tmpRotor.setVolMeshCustomTEControl(volSurfControlTE);
            
        }
        
        
    }

    private void instantiateRotorObjects(){
        /* Method instantiateRotorObjects()
           Used to instantiate Rotor Objects from a simulation
           Can be extracted from the following conditions:
            1) A coordinate system w/ the BEM_ prefix
            2) A rotating RBM Domain
        */
        if(allRBMRotorObjects.isEmpty()){
            simu.println("RTR MSG: Instantiating Rotor Objects...");
            String rotorPreFix=globalNames.getRotorID().substring(0,1);

            // Set up RBM Rotors from Rotating Domains (if they exist)
            for(RotatingDomain tmpDomain:allRotatingDomains){
                simu.println("RTR MSG: There are "+allRotatingDomains.size()+
                        " domains to search for RBM Rotor Objects.");
                simu.println("RTR MSG: Looking in Domain "+tmpDomain.getName());
                GeometryPart domainPart=tmpDomain.getDomainPart();

                // If domainPart is created from a boolean mesh operation
                //   we can know the parent parts of the mesh operation which means we know:
                //   1) The name of the Rotor
                //   2) everything about the geometry manipulations from
                //      the Body Csys orientation 
                //   3) The custom controls and settings
                //
                // Look through the Domain PartSurface list to take out any interfaces
                simu.println("RTR PRE: Looking through Domain PartSurface list.");
                Collection<PartSurface> rotorSurfaces=domainPart.getPartSurfaces();
                Collection<PartSurface> filteredSurfaces = new ArrayList();
                for(PartSurface tmpSurf:rotorSurfaces){
                    String tmpName = tmpSurf.getPresentationName();
                    String osIntID = globalNames.getOversetStr();
                    String slideIntID = globalNames.getSlidingStr();
                    boolean nameHasSliding = tmpName.startsWith(slideIntID)||
                            tmpName.contains(slideIntID);
                    boolean nameHasOS = tmpName.startsWith(osIntID)||
                            tmpName.contains(osIntID);
                    if(!(nameHasSliding||nameHasOS)){
                        filteredSurfaces.add(tmpSurf);
                    }
                }
                rotorSurfaces=filteredSurfaces;

                if(!rotorSurfaces.isEmpty()){
                    simu.println("RTR PRE: Found surfaces.");                                  
                    GeometryPart cadParent=null;
                    String parentPartName="";
                    // domain Part must be from a boolean operation,
                    // find the specific parent CAD part and get coordinate system
                    if(domainPart instanceof MeshOperationPart){
                        simu.println("RTR PRE: Part is from MeshOperationPart.");   
                        // get the operation that created the domain Part
                        MeshOperation domainOp=((MeshOperationPart) domainPart).getOperation();
                        parentPartName=domainOp.getPresentationName();
                        Collection<GeometryPart> domainParts
                                =domainOp.getInputGeometryObjects()
                                        .getLeafParts();
                        // get the all possible rotor parts
                        cadParent=(GeometryPart) MeshOpTool.filterOutGeometryParts(simu,domainParts, rotorPreFix).toArray()[0];
                        if(parentPartName.equals("")||cadParent==null){
                            simu.println("RTR MSG: cadCsysName or cadParent is empty!");
                        }
                        //===========================
                        //    Create Rotor Object
                        //===========================
           
                        String cadParentName = cadParent.getPresentationName();
                        simu.println("RTR OBJ: cadParent is "+cadParentName);   
                        CartesianCoordinateSystem cadCSys = SimTool.getLabBasedRotorCoordinate(simu, cadParentName);
                        // Begin
                        simu.println("RTR OBJ: Creating Rotor Object: "+cadParentName);
                        Rotor newRotor = 
                            new Rotor(simu,rotorPreFix.length(),cadCSys);
                        newRotor.setName(cadParentName);
                        newRotor.setIsBEM(false); //came from physical geometry
                        simu.println("RTR OBJ: Rotor is not a BEM");
                        allRotorObjects.add(newRotor);
                        allRBMRotorObjects.add(newRotor);
                        //
                        ArrayList<PartSurface> tmpPartSurfList = new ArrayList(rotorSurfaces);
                        newRotor.setPhysicalPartSurfaces(tmpPartSurfList);
                        
                        // Get requisite rotor information
                        simu.println("RTR OBJ: Getting rotor blade number.");                
                        newRotor.getNBlades();
                        simu.println("RTR OBJ: Getting rotor blade speed."); 
                        newRotor.getRotorSpeed();
                        simu.println("RTR OBJ: Getting rotor blade radius."); 
                        newRotor.getBladeRadius();
                        
                        //Domain link
                        newRotor.setRotatingDomain((RotatingDomain) tmpDomain);
                        newRotor.getRotatingDomain().assignRotor(newRotor);
                        //Parent Region
                        newRotor.setParentRegion(tmpDomain.getRegion());
                        newRotor.setUniqueBoundary(rotorPreFix); //put every surface into the Rotor boundary
                        //newRotor.setUniqueBladeBoundary(); //put the blades into their own approriate boundary for y+

                        simu.println("RTR MSG: Time to make reports!");
                        try{//only need the catch in case there are no physics
                            //Make sure airfoil knows about its Reports
                            simu.println("RTR MSG: Setting reference values");
                            newRotor.setReferenceValues(refRho,refVel);
                            simu.println("RTR MSG: making reports");
                            newRotor.makeReports();
                            simu.println("RTR MSG: Made reports for "+newRotor.getName());
                            //
                            //Make sure airfoil knows about its Monitors
                            int nPlotSamples=(int) 1.e5;
                            int itMonFreq = 1;
                            int itMonStart= 1;
                            newRotor.makeIterationMonitors(simu,nPlotSamples,itMonFreq,itMonStart);
                            simu.println("RTR MSG: Made iteration monitors for "+newRotor.getName());
                        }catch(NeoException e){

                        }
                    }
                }
            }   
            //Display information on found/created Rotors
            for(Rotor tmp:allRotorObjects){
                simu.println("RTR MSG: Rotor "+tmp.getName()+" instantiated.");
                simu.println("  General Properties:");
                simu.println("    Blades: "+tmp.getNBlades());
                simu.println("    Radius: "+tmp.getBladeRadius());
                simu.println("     Speed: "+tmp.getRotorSpeed());
            }
        }
        for(Rotor tmp:allRBMRotorObjects){
            tmp.updatePhysicalPartSurfaces();
        }
        for(Rotor tmp:allRBMRotorObjects){
            allWTRegs.add(tmp.getRotatingDomain().getRegion());
        }
    }
    public double getAverageRotorTipSpeed(){
        double retVal;
        double nVal = allRBMRotorObjects.size();
        double tmpVal=0;
        for(int i=0; i<nVal;i++){
            tmpVal+=allRBMRotorObjects.get(i).getBladeRadius()
                    *allRBMRotorObjects.get(i).getRotorSpeed();
        }
        retVal=tmpVal/nVal;
        return retVal;
    }
    
    public void forceRotorsToMRF(){
        for(Rotor tmpRotor:allRotorObjects){
            boolean isBEM = tmpRotor.isBEM();
            if(!isBEM) tmpRotor.applyRotorMRF();
        }
    }
    public void forceRotorsToRBM(){
        for(Rotor tmpRotor:allRotorObjects){
            boolean isBEM = tmpRotor.isBEM();
            if(!isBEM) tmpRotor.applyRotorRBM();
        }
    }
    public void initializeObjectICs(){
        
        for(Rotor tmpRotor:allRBMRotorObjects){
            tmpRotor.initLocalRotorICs(simu.getFieldFunctionManager().getFunction("vel_ic"));
        }
        
        
        
    }
    
    //SOLUTIONS
    public void clearSolution(){
        simu.clearSolution();
    }
    public void turnOnAutoTimeStep(){
        simDriver.turnOnAutoTimeStep();
    }
    public double setTimeStep(double newTimeStep){
        SolverDriver.setTimeStep(simu,newTimeStep);
        return newTimeStep;
    }
    public void createUnsteadyObjectReporting(){
        int nPlotSamples=(int) 1e4;
        int tsMonFreq = 1;
        int tsMonStart= 1;
        
        simu.println("Making unsteady reports");
        //Rotors
        if(allRotorObjects.size()>0){
            for(Rotor tmp:allRotorObjects){
                tmp.makeUnsteadyMonitors(nPlotSamples, tsMonFreq, tsMonStart);
            }
        }
    }
    public void setInnerIterations(int newInnerIterations){
        SolverDriver.setInnerIts(simu,newInnerIterations);
    }
    public void setFinalSolutionTime(double newValue){
        SolverDriver.setFinalSolutionTime(simu,newValue);
    }
    
    public double getRecommendedTotalRunTime(){
        /* Returns the recommended final solution time based on 10 periodic
           flow overs, i.e.: Lengthscale/Reference Velocity
        */
        double recommendedTime=0.0;
        //Rotors
        //have rotor time scales
        int nSpins=10;
        int nStartSpins=3;
        double recRotorTime=0;
        if(allRotorObjects.size()>0){
            int nTotSpin=nSpins+nStartSpins;
            for(Rotor tmp:allRotorObjects){
                recRotorTime=1./Math.abs(tmp.getRotorSpeed()/Math.PI)*nTotSpin;
                if(recRotorTime>recommendedTime){
                    recommendedTime=recRotorTime;
                }
            }
        }
        
        NumberFormat formattedDouble = new DecimalFormat("0.#E0");
        String truncatedTimeStep=formattedDouble.format(recommendedTime);
        return Double.parseDouble(truncatedTimeStep);
    }
    public void deactivateMaxStepsCriteria(){
        SolverDriver.deactivateMaxStepsCriteria(simu);
    }
    public void runSolver(int solverSteps){
        /* Runs the Windtunnel under the current settings and configuration
        */
        SolverDriver.setSolverContinuityInit(simu, true,3); // experimental!
        SolverDriver.setKWBoundaryLayerInit(simu,true);   // experimental!
        SolverDriver.stepSimulation(simu,wtPhysics.getContinuum(),solverSteps,simDriver.useAutoTimeStep());
    }
    
    public void activateSolverLinearRamp(int lastRampIt){
        SolverDriver.useLinearRampOnSegregatedSolver(simu,lastRampIt);
    }
    
    
    //
    // MONITORS
    public void cleanUpMonitors(int newFreq){
        //Clear it out
        simu.getMonitorManager().setPrintedMonitors();
        
        ArrayList<Monitor> printedMonList = new ArrayList();
        // Collect physics monitors
        printedMonList.addAll(CFD_Physics.getPhysicsMonitorList(simu));
        
        //get seconds per iteration monitor/report
        String repName = "sec/it";
        try{
            Monitor tmpMon = simu.getMonitorManager().getMonitor(repName);
            printedMonList.add(tmpMon);
        }catch(NeoException e){
            try{
                Monitor tmpMon = ((IteratorElapsedTimeReport) simu.getReportManager().getReport(repName)).createMonitor();
                printedMonList.add(tmpMon);
            }catch(NeoException j){
                IteratorElapsedTimeReport itPerSecRep = 
                    simu.getReportManager().createReport(IteratorElapsedTimeReport.class);
                itPerSecRep.setPresentationName(repName);
                Monitor tmpMon = itPerSecRep.createMonitor();
                printedMonList.add(tmpMon);
            }
        }
        
        
        // Collect any additional Rotor thrust and torque
        for(Rotor tmpRotor:allRotorObjects){
            printedMonList.add(tmpRotor.getThrustMonitor(isUns));
            printedMonList.add(tmpRotor.getTorqueMonitor(isUns));
        }
        
        // these are what we want to print out
        simu.getMonitorManager().setPrintedMonitors(printedMonList);
        simu.getMonitorManager().setHeadingPrintFrequency(newFreq);
        
    }
    
    public double getDomainInletSpeed(){
      //Get velocity at inlet
      VelocityMagnitudeProfile vMP = 
        mainDomain.getAnInletBoundary().getValues().get(VelocityMagnitudeProfile.class);
      return vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getInternalValue();
    }
    
    public double getAngleOfAttack(){
        String assemblyAlphaOpName="Rotate Assembly";
        angleOfAttack=MeshOpTool.getRotationAngle(simu,assemblyAlphaOpName);
        return angleOfAttack;
    }
    public double getSideSlipAngle(){
        String assemblyBetaOpName="Rotate Assembly - Beta";
        sideSlipAngle=MeshOpTool.getRotationAngle(simu,assemblyBetaOpName);
        return sideSlipAngle;
    }
    
    
    //OUTPUT
    public String getFileString(){
        String retStr="";
        NumberFormat formatbig = new DecimalFormat("0.##E0");
        NumberFormat formatsmall = new DecimalFormat("0.##");
        
        // Unsteady file string
        retStr=retStr+CFD_Physics.getTurbulenceModelName(wtPhysics.getContinuum());

        //Inlet Speed
        retStr=retStr+"_V";
        retStr=retStr+formatbig.format(refVel);

        //Alpha
        retStr=retStr+"_A";
        retStr=retStr+formatsmall.format(angleOfAttack);
        
        //Beta
        retStr=retStr+"_B";
        retStr=retStr+formatsmall.format(sideSlipAngle);
       
        // Individual Rotor Element Information
        for(Rotor tmpRtr:allRotorObjects){
            int nameEndIndx=21;
            int nameLength= tmpRtr.getName().substring(4).length();
            if(nameLength<nameEndIndx-1){
                nameEndIndx=nameLength+1;
            }
            String shortRtrName= tmpRtr.getName().substring(4,nameEndIndx);
            double rtrSpeed=tmpRtr.getRotorSpeed();
            //mrf, rbm, bem
            String rtrOrBEM="RTR";
            if(tmpRtr.getParentRegion()==null) rtrOrBEM="BEM";
            retStr=retStr+"_"+rtrOrBEM+"-"+shortRtrName+"oz"+formatsmall.format(rtrSpeed);
        }
        return retStr;
    }
    public String makeSheetsCSVData(String studyName,String cfdVer,String javaVer){
        String retString;
        
        //CFD Simulation Stats
        //Study Name
        //Code version
        retString=studyName+","+cfdVer+","+javaVer;
        //
        //Number of Iterations
        int currentItLvl=SolverDriver.getCurrentIterationLevel(simu);
        retString=retString+","+currentItLvl;
        //Number of Steps
        int currentStepLvl=SolverDriver.getCurrentStepLevel(simu);
        retString=retString+","+currentStepLvl;
        //Time step (if exists)
        if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
            retString=retString+","+SolverDriver.getTimeStep(simu);
        }else{
            retString=retString+","+"NaN";
        }
        //Solution Time (if exists)
        if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
            retString=retString+","+SolverDriver.getCurrentTime(simu);
        }else{
            retString=retString+","+"NaN";
        }
        //Stopping Criteria Satisfied
        retString=retString+","+SolverDriver.getSatisfiedStoppingCriteriaNames(simu);
        //
        // Simulation Type (can be more descriptive later)
        retString=retString+","+"3D";
        //steady or unsteady
        if(CFD_Physics.isUnsteady(wtPhysics.getContinuum())){
            retString=retString+","+"Transient";
        }else{
            retString=retString+","+"Steady";
        }
        //turbulence model
        retString=retString+","+CFD_Physics.getTurbulenceModelName(wtPhysics.getContinuum());
        //
        //  get primary boundary condition information
        //  --> dictates freestream or velocity inlet
        //  if inlet exists, it is a velocity inlet type of case
        //  else it is a freestream case
        // 
        Boundary mainBndy=mainDomain.getAnInletBoundary();
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
            // TuI
            TurbulenceIntensityProfile tIP = 
                mainBndy.getValues().get(TurbulenceIntensityProfile.class);
            bcTi=tIP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            
            // TVR
            TurbulentViscosityRatioProfile tVRP = 
                mainBndy.getValues().get(TurbulentViscosityRatioProfile.class);
            bcTVR=tVRP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            
            // Calculate Re, Mach
            double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
            double molAir=.0289645; // [kg/mol] molar gas constant of air
            double refPress=CFD_Physics.getReferencePressure(wtPhysics.getContinuum()); // [Pa] reference pressure for air
            
            // Get dynamic viscosity from WT Physics
            bcMu=CFD_Physics.getMu(wtPhysics.getContinuum());
            bcRho=refPress/bcTemp/(airR/molAir);
            bcVel = bcMa/Math.sqrt(1.4*(airR/molAir)*bcTemp);
            bcRe=lengthScale*bcVel*bcRho/bcMu;

        }else{
            // Velocity
            VelocityMagnitudeProfile vMP = 
                mainBndy.getValues().get(VelocityMagnitudeProfile.class);
            bcVel=vMP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            // Temperature
            StaticTemperatureProfile sTP = 
              mainBndy.getValues().get(StaticTemperatureProfile.class);
            bcTemp=sTP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            //
            // TuI
            TurbulenceIntensityProfile tIP = 
                mainBndy.getValues().get(TurbulenceIntensityProfile.class);
            bcTi=tIP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            //
            // TVR
            TurbulentViscosityRatioProfile tVRP = 
                mainBndy.getValues().get(TurbulentViscosityRatioProfile.class);
            bcTVR=tVRP.getMethod(ConstantScalarProfileMethod.class).getQuantity().getRawValue();
            //
            // Get dynamic viscosity from WT Physics
            bcMu=CFD_Physics.getMu(wtPhysics.getContinuum());
            //
            // Calculate Re, Mach
            double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
            double molAir=.0289645; // [kg/mol] molar gas constant of air
            double refPress=CFD_Physics.getReferencePressure(wtPhysics.getContinuum()); // [Pa] reference pressure for air
            //
            //Temp=refPress/refRho/(airR/molAir)
            bcRho=refPress/bcTemp/(airR/molAir);
            bcRe=lengthScale*bcVel*bcRho/bcMu;
            bcMa = bcVel/Math.sqrt(1.4*(airR/molAir)*bcTemp);
        }

        
        retString=retString+","+lengthScale+","+bcVel+","+bcTemp+","
                +bcRho+","+bcMu+","+bcTVR+","+bcTi;

        simu.println("RTR CSV: Getting Alpha/Beta");
        //Total Rotor Configuration Performance (If it exists)
        //Alpha, Beta
        retString=retString+","+angleOfAttack+","+sideSlipAngle;
        
        simu.println("RTR CSV: Compiling total performance data");
        //Total rotor forces and moments in body
        double bodyFX=0;double bodyFY=0;double bodyFZ=0;
        double bodyMX=0;double bodyMY=0;double bodyMZ=0;
        
        if(allRotorObjects.size()>0){
            for(Rotor tmp:allRotorObjects){
                bodyFX+=tmp.getBodyFX();
                bodyFY+=tmp.getBodyFY();
                bodyFZ+=tmp.getBodyFZ();
                bodyMX+=tmp.getBodyMX();
                bodyMY+=tmp.getBodyMY();
                bodyMZ+=tmp.getBodyMZ();
            }
            retString=retString+","+bodyFX+","+bodyFY+","+bodyFZ;
            retString=retString+","+bodyMX+","+bodyMY+","+bodyMZ;
        }
        
        //Output global mesh settings
        simu.println("RTR CSV: WT Mesh Settings...");
        retString=retString+"," + getMainTunnelMeshString();
        
        //SPECIFIC OBJECT POST-PROCESSING INFORMATION
       // Individual Airfoil Element Information
        for(Rotor tmpRotor:allRotorObjects){
            simu.println("RTR CSV: "+tmpRotor.getName());
            retString=retString+","+tmpRotor.csvData(isUns);
        }
        
        return retString;
    }
    private String getMainTunnelMeshString(){
        String retString="";
        // get the volume mesh model name
        AutoMeshOperation wtSM=MeshOpTool.getAutoMeshOp(simu,"SM "+mainDomainName);
        AutoMeshOperation wtVM=MeshOpTool.getAutoMeshOp(simu,"VM "+mainDomainName);
        try{
            wtVM.getMeshers().getObject("Trimmed Cell Mesher");
            retString=retString+"Trim";
        }catch(NeoException e){
            retString=retString+"Poly";
        }
        //Base size
        retString=retString+","+wtVM.getDefaultValues().get(BaseSize.class).getRawValue();
        
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
        retString=retString+","+prismThickness.getAbsoluteSizeValue().toString();
        
        //Prism First Cell Thickness
        retString=retString+","+wtVM.getDefaultValues().get(PrismWallThickness.class).getSIValue();

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
    //REGIONS
    private Region getMainWTRegion(Collection<Region> allRegs){
        for(Region tmpReg:allRegs){
            Collection<Boundary> allBndy=tmpReg.getBoundaryManager().getBoundaries();
            //Main WT Region will contain Inlet or FS
            for(Boundary tmpBndy:allBndy){
                if((tmpBndy.getBoundaryType() instanceof InletBoundary) ||
                   (tmpBndy.getBoundaryType() instanceof FreeStreamBoundary)){
                    return tmpReg;
                }
            }
        }
        return null;
    }
    private Boundary setUpBoundary(Region tmpReg,String bndyName){
        Boundary tmpBndy;
        try{
            tmpBndy = tmpReg.getBoundaryManager().getBoundary(bndyName);
        }catch(NeoException e){
            tmpBndy = 
                tmpReg.getBoundaryManager().createEmptyBoundary(bndyName);
        }
        return tmpBndy;
    }

    // PRIVATE METHODS
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
    private void setRefArea(double refVal){
        refArea=refVal;
    }
    private void setRefRe(double refVal){
        refRe=refVal;
    }
    private CartesianCoordinateSystem makeCADZeroBodyCSys(){
        //
        // Geometry based turn table
        //
        // The Body Coordinate System *MUST* align with LAB X,Y,Z otherwise
        //   angles >= 90 degrees will screw up the detection for orientation
        CartesianCoordinateSystem assmbCoord
                =SimTool.getLabBasedCoordinate(simu,"CAD Zero "+bodyCsys.getPresentationName());
        assmbCoord.copyProperties(bodyCsys);
        //opposite direction of body, same y vector
        double oldxValue=bodyCsys.getBasis0().toDoubleArray()[0];
        double oldyValue=bodyCsys.getBasis1().toDoubleArray()[1];
        if(Math.abs(oldxValue-1.0)>1e-6){
            assmbCoord.setBasis0(new DoubleVector(new double[] {1.0,0.0,0.0}));
        }

        if(Math.abs(oldyValue-1.0)>1e-6){
            assmbCoord.setBasis1(new DoubleVector(new double[] {0.0,1.0,0.0}));
        }

        return assmbCoord;
    }
    private CartesianCoordinateSystem makeCADZeroBetaAngleBodyCSys(){
        //
        // Geometry based turn table
        //
        // must duplicate Body to Body_Rotate, otherwise meshOp will go out of
        // date on rotation of main coordinate system and PBM pipeline will show
        // not-up-to-date
        CartesianCoordinateSystem assmbCoord
                =SimTool.getLabBasedCoordinate(simu,"CAD Zero "+bodyCsys.getPresentationName()+ " Beta Angle");
        assmbCoord.copyProperties(bodyCsys);
        //opposite direction of body, same y vector
        double oldxValue=bodyCsys.getBasis0().toDoubleArray()[0];
        double oldyValue=bodyCsys.getBasis1().toDoubleArray()[1];
        if(Math.abs(oldxValue-1.0)>1e-6){
            assmbCoord.setBasis0(new DoubleVector(new double[] {1.0,0.0,0.0}));
        }
        if(Math.abs(oldyValue-1.0)>1e-6){
            assmbCoord.setBasis1(new DoubleVector(new double[] {0.0,1.0,0.0}));
        }
        return assmbCoord;
    }
    
    // MESHING
    private ArrayList<GeometryPart> getVolumeControlParts(){
        /* Figures out what Geometry Parts are volume control parts
        
        */
        ArrayList<GeometryPart> retArr= new ArrayList();
        Collection<GeometryPart> rawGeomList = allGeomParts;
        for(GeometryPart tmpPart:allGeomParts){
            String tmpPartName=tmpPart.getPresentationName();
            if(tmpPartName.startsWith(globalNames.getVCPreFix())){
                retArr.add(tmpPart);
            }
        }
        return retArr;
    }
    
    private ArrayList<GeometryPart> getStationaryVolumeControlParts(){
        /* Figures out what Geometry Parts are stationary volume control parts
            These parts do not join in any mesh operation rotation games!        
        */
        ArrayList<GeometryPart> retArr= new ArrayList();
        Collection<GeometryPart> rawGeomList = allGeomParts;
        for(GeometryPart tmpPart:allGeomParts){
            String tmpPartName=tmpPart.getPresentationName();
            if(tmpPartName.startsWith(globalNames.getStatVCPreFix())){
                retArr.add(tmpPart);
            }
        }
        return retArr;
    }
    
    
    private void createGeometryPartRotateCADAngle(GeometryPart tmpPart,
            CartesianCoordinateSystem tmpCsys,double[] rotVector){
        //  Airfoils: 
        //    allow all tmpPart to rotate on the turntable
        //
        String partName = tmpPart.getPresentationName();
        simu.println("FC MSG: Operation set for: "+tmpPart);
        MeshOpTool.rotationTransform(simu,"Rotate "+partName,tmpPart,tmpCsys,rotVector);
    }
    
    
    private void createPrimaryDomain(GeometryPart wtPart){
        mainDomain = new Domain(simu,wtPart);
        simu.println("RTR MSG: Setting up windtunnel region.");
        mainDomain.setUpRegion(simu);
        simu.println("RTR MSG: Setting up windtunnel boundaries.");
        mainDomain.initPartBoundaries();
        Region wtRegion=mainDomain.getRegion();
        allWTRegs.add(wtRegion);
        allDomainObjects.add(mainDomain);
    }
    private void instantiatePrimaryDomain(){
        /* Method instantiateWTDomains figures out what existing Domain objects
            already exist in the simulation and instantiates their objects.
            This method is strictly for extraction of existing objects in a sim.
            If there already domain objects in memory, the process is skipped.
        
            Domains include:
              WT Main Domain

        */
        if(mainDomain==null){
            simu.println("RTR MSG: Instantiating primary Domain...");
            //Figure out which geometry creates the main region
            Collection<Region> allRegions=getRegions();
            GeometryPart mainWTPart=null;
            try{
                Region mainWTRegion = getPrimaryRegion(allRegions);
                mainWTPart = getRegionPart(mainWTRegion);
            }catch(NeoException e){
                simu.println("RTR MSG: Found pre-existing regions, no WT region detected.");
            }
            //Create Domain objet
            mainDomain = new Domain(simu,mainWTPart);
            mainDomainName = mainDomain.getName(); // let FC runner know you are here
            
            //region setup/manipulation
            mainDomain.setUpRegion(simu);
            mainDomain.getExistingDomainBoundaries();
            allWTRegs.add(mainDomain.getRegion()); // let FC runner know this region exists
            
            //Associate with Mesh Operations
            String meshOpName="SM "+mainDomain.getName();
            mainDomain.setDomainSurfMeshOp(
                    MeshOpTool.getAutoMeshOp(simu,meshOpName));
            meshOpName="VM "+mainDomain.getName();
            mainDomain.setDomainVolumeMeshOp(
                    MeshOpTool.getAutoMeshOp(simu,meshOpName));

        }
        //Signal Domain objects that exist in this simulation
        simu.println("RTR MSG: Domain "+mainDomain.getName()+" instantiated.");

    }
    private OversetDomain createOversetDomain(GeometryPart osPart){
        OversetDomain osDomain = new OversetDomain(simu,osPart);
        simu.println("RTR MSG: Setting up overset region from Domain "
                +osPart.getPresentationName());
        osDomain.setUpRegion(simu);
        simu.println("RTR MSG: Setting up overset region boundaries.");
        osDomain.initPartBoundaries();
        Region osRegion=osDomain.getRegion();
        allWTRegs.add(osRegion);
        return osDomain;
    }
    private void instantiateOversetDomains(){
        simu.println("RTR MSG: Setting up overset Domain(s).");
        if(allOversetDomains.size()<1){
            simu.println("RTR MSG: There are no OversetDomain objects in memory...");
            simu.println("RTR MSG: ... checking for existing Overset regions.");
            // if Regions exist, see if we can't find some existing Overset objects
            //   that were already set up by the preprocessor
            for(Region tmpReg:simu.getRegionManager().getObjects()){
                String regName = tmpReg.getPresentationName();
                if(regName.startsWith("OS")){
                    simu.println("RTR MSG: Region "+regName+" should be OversetDomain...");
                    simu.println("RTR MSG: ... creating OversetDomain from Region.");
                    GeometryPart osPart =getRegionPart(tmpReg);
                    //Create domain
                    OversetDomain osDomain = createOversetDomain(osPart);
                    //get local coordinate system & angle
                    osDomain.setOversetCoordinateSystem(
                            SimTool.getLabBasedCoordinate(simu,
                                    osPart.getPresentationName()));
                    osDomain.determineDomainAngle();

                    // Affiliate with existing mesh operations
                    String meshOpName="SM "+osDomain.getName();
                    osDomain.setDomainSurfMeshOp(
                            MeshOpTool.getAutoMeshOp(simu,meshOpName));
                    meshOpName="VM "+osDomain.getName();
                    osDomain.setDomainVolumeMeshOp(
                            MeshOpTool.getAutoMeshOp(simu,meshOpName));
                    
                    // make sure WT analysis knows this Overset domain exists!
                    allOversetDomains.add(osDomain);
                    allDomainObjects.add(osDomain);
                    // output window
                    simu.println("RTR MSG: Overset domain "+osDomain.getName()+" was found.");
                }
            }
        }
        if(allOversetDomains.size()<1){
            simu.println("RTR MSG: There are still no OversetDomain objects in memory...");
            simu.println("RTR MSG: ... checking for GeometryPart Overset Parts.");
            // we are probably setting up from scratch or there are no overset
            // domains
            for(GeometryPart tmpOSPart:getRawOversetParts()){
                simu.println("RTR MSG: GeometryPart "+tmpOSPart.getPresentationName()+" should be OversetDomain...");
                simu.println("RTR MSG: ... creating OversetDomain from this GeometryPart.");
                //Create domain
                OversetDomain osDomain = createOversetDomain(tmpOSPart);
                osDomain.setOversetCoordinateSystem(
                        SimTool.getLabBasedCoordinate(simu,
                                tmpOSPart.getPresentationName()));
                
                // Surface Mesh Operation
                String meshOpName="SM "+osDomain.getName();
                AutoMeshOperation osSurfaceOp=MeshOpTool
                    .surfMeshOp(simu,meshOpName,baseSize, 100.0,10.0,32.,1.3,false);
                osDomain.setDomainSurfMeshOp(osSurfaceOp);
                osSurfaceOp.getInputGeometryObjects().setQuery(null);
                osSurfaceOp.getInputGeometryObjects().setObjects(
                        osDomain.getDomainPart());
                // Volume Mesh Operation
                CFD_TrimmerModel osVolumeMeshOp = new CFD_TrimmerModel(simu,
                    "VM "+osDomain.getName(),labCsys, baseSize);
                osVolumeMeshOp.addPart(tmpOSPart);
                osDomain.setDomainVolumeMeshOp(osVolumeMeshOp.getVMOp());
               
                // make sure WT analysis knows this Overset domain exists!
                allOversetDomains.add(osDomain);
                allDomainObjects.add(osDomain);
            }
        }
        simu.println("RTR MSG: There are "+allOversetDomains.size()+" overset domains.");
    }
    
    
    private void instantiateRotatingDomainSurfaceMeshOps(){
        for(RotatingDomain rotDomain:allRotatingDomains){
            // Surface Mesh Operation
            String meshOpName="SM "+rotDomain.getName();
            AutoMeshOperation rotSurfaceOp;
            try{
                rotSurfaceOp=MeshOpTool.getAutoMeshOp(simu,meshOpName);
                rotDomain.setDomainSurfMeshOp(rotSurfaceOp);
            }catch(NeoException e){
                rotSurfaceOp=MeshOpTool
                .surfMeshOp(simu,meshOpName,baseSize, 100.0,10.0,32.,1.3,false);
                rotDomain.setDomainSurfMeshOp(rotSurfaceOp);
                rotSurfaceOp.getInputGeometryObjects().setQuery(null);
                rotSurfaceOp.getInputGeometryObjects().setObjects(
                    rotDomain.getDomainPart());
            }
        }
    }
    
    private void instantiateRotatingDomainVolumeMeshOps(){
        
        for(RotatingDomain rotDomain:allRotatingDomains){
            GeometryPart tmpRotPart = rotDomain.getDomainPart();
            
            String meshOpName="VM "+rotDomain.getName();
            AutoMeshOperation tmpOp;
            try{
                tmpOp=MeshOpTool.getAutoMeshOp(simu,meshOpName);
                simu.println("RTR MSG: op w/ name : "
                        +tmpOp.getPresentationName() +" found");
            }catch(NeoException e){
                CFD_TrimmerModel rotVolumeMeshOp = new CFD_TrimmerModel(simu,
                    meshOpName,labCsys, baseSize);
                rotVolumeMeshOp.addPart(tmpRotPart);
                rotDomain.setDomainVolumeMeshOp(rotVolumeMeshOp.getVMOp());
                tmpOp=rotDomain.getDomainVolumeMeshOp();
                simu.println("RTR MSG: making vm op : "+tmpOp.getPresentationName());

            }
            //v1.07 - prism to core ratio
            //prism layer to core ratio to 5.0
            tmpOp.getDefaultValues()
                    .get(MaxTrimmerSizeToPrismThicknessRatio.class)
                    .setLimitCellSizeByPrismThickness(true);
            tmpOp.getDefaultValues()
                    .get(MaxTrimmerSizeToPrismThicknessRatio.class)
                    .getSizeThicknessRatio()
                    .setNeighboringThicknessMultiplier(5.0);

        }
        
    }
    
    private void instantiateRotatingDomains(){
        /* Method instantiateRotatingDomains()
           Used to instantiate Rotating Domains from a simulation
           Can be extracted from the following domains:
            Windtunnel Domain
        */
        if(allRotatingDomains.isEmpty()){
            simu.println("RTR MSG: Setting up RBM Rotor Domain(s).");
            if(allRotatingDomains.size()<1){
                simu.println("RTR MSG: There are no RotatingDomain objects in memory...");
                simu.println("RTR MSG: ... checking for existing Rotating regions.");
                // if Regions exist, see if we can't find some existing Rotating objects
                //   that were already set up by the preprocessor
                Collection<Region> rotRegions=gatherRotatingBodyRegions();
                if(rotRegions.size()>0){
                    for(Region tmpReg:rotRegions){
                        String regName = tmpReg.getPresentationName();
                        simu.println("RTR MSG: Region "+regName+" should be RotatingDomain...");
                        simu.println("RTR MSG: ... creating RotatingDomain from Region.");
                        GeometryPart rotPart =getRegionPart(tmpReg);
                        //Create domain
                        RotatingDomain rotDomain = createRotatingDomain(rotPart);

                        // Affiliate with existing mesh operations
                        String meshOpName="SM "+rotDomain.getName();
                        rotDomain.setDomainSurfMeshOp(
                                MeshOpTool.getAutoMeshOp(simu,meshOpName));
                        meshOpName="VM "+rotDomain.getName();
                        rotDomain.setDomainVolumeMeshOp(
                                MeshOpTool.getAutoMeshOp(simu,meshOpName));

                        // make sure WT analysis knows this Rotating domain exists!
                        allRotatingDomains.add(rotDomain);
                        allDomainObjects.add(rotDomain);
                        // output window
                        simu.println("RTR MSG: Rotating domain "+rotDomain.getName()+" was found.");
                    }
                }
            }
            
            // failed to build from existing Regions, try Parts
            if(allRotatingDomains.size()<1){
                simu.println("RTR MSG: There are still no OversetDomain objects in memory...");
                simu.println("RTR MSG: ... checking for GeometryPart Rotating Parts.");
                // we are probably setting up from scratch or there are no overset
                // domains
                simu.println("RTR MSG: ... these are the raw GeometryParts");
                simu.println(getRawRotatingDomainParts());
                for(GeometryPart tmpRotPart:getRawRotatingDomainParts()){
                    simu.println("RTR MSG: GeometryPart "+tmpRotPart.getPresentationName()+" should be RotatingDomain...");
                    simu.println("RTR MSG: ... creating RotatingDomain from this GeometryPart.");
                    //Create domain
                    RotatingDomain rotDomain = createRotatingDomain(tmpRotPart);
                    // make sure WT analysis knows this Overset domain exists!
                    allRotatingDomains.add(rotDomain);
                    allDomainObjects.add(rotDomain);
                }
            }
            simu.println("RTR MSG: There are "+allRotatingDomains.size()+" rotating domains.");
        }
    }

    //PHYSICS
    public void initPhysics(String physName,String physModel){
      simu.println("RTR MSG: Initializing physics continuum");
      //if(!wtPhysics.doesPhysicsExist()){
      wtPhysics = new CFD_Physics(simu,physName);
      //Physics model selection
      PhysicsContinuum tmpPhys=wtPhysics.getContinuum();
      double wallDistToFS = wtPhysics.getWallDistanceToFreeStream();
      if(physModel.equals("kwSST")){
          simu.println("RTR MSG: ... kwSST Model Selected.");
          isUns=false;
          CFD_Physics.set_RANS_KwSST(tmpPhys,true, true, true,false, false,wallDistToFS);
          //Physics Solver Best Practice Settings
          CFD_Physics.setKWSSTURF(simu,0.6);
          CFD_Physics.setKWSSTViscosity(simu,0.8,1.0E20);
      }else if(physModel.equals("kwSSTGRT")){
          simu.println("RTR MSG: ... kwSST GRT Model Selected.");
          isUns=false;
          CFD_Physics.set_RANS_KwSST(tmpPhys,true, true, true,false, true,wallDistToFS);
          //Physics Solver Best Practice Settings
          CFD_Physics.setKWSSTURF(simu,0.6);
          CFD_Physics.setKWSSTViscosity(simu,0.8,1.0E20);
      }else if(physModel.equals("kwSSTDES")){
          isUns=true;
          CFD_Physics.set_DES_KwSST(tmpPhys,true, true, true,false, false,wallDistToFS);
          //set up time step information
          SolverDriver.set2ndOrderTimeDisc(simu);
      }else if(physModel.equals("kwSSTGRTDES")){
          simu.println("RTR MSG: ... kwSST GRT DES Model Selected.");
          isUns=true;
          CFD_Physics.set_DES_KwSST(tmpPhys,true, true, true,false, true,wallDistToFS);
          //set up time step information
          SolverDriver.set2ndOrderTimeDisc(simu);
      }

      //Special region stuff
      CFD_Physics.setMinimumReferenceWallDistance(tmpPhys,1.0e-8);
      CFD_Physics.setMinimumAllowableTemperature(tmpPhys, 10.0  );
      CFD_Physics.setMaximumAllowableTemperature(tmpPhys, 1000.0);

      //Apply to all applicable regions
      Collection<Region> allRegs=simu.getRegionManager().getObjects();
      for(Region tmpReg:allRegs){
          tmpReg.setPhysicsContinuum(wtPhysics.getContinuum());
      }
    }
    public void setInitialConditions(){
        CFD_Physics.set_VelocityIC(wtPhysics.getContinuum(),inletCsys,refVel,lengthScale*.01,lengthScale*0.10);
        double refPress=wtPhysics.getContinuum().getReferenceValues()
                .get(ReferencePressure.class).getSIValue();
        CFD_Physics.set_PressureIC(wtPhysics.getContinuum(),refVel,refPress,lengthScale*0.10);
        CFD_Physics.set_TemperatureIC(wtPhysics.getContinuum(),refPress, refRho);

    }
    public void initRotorMotions(){
        // Rotors
        for(Rotor tmpRotor:allRotorObjects){
            tmpRotor.initializeRotorMotions();
        }
    }
    
    public double getRecommendedTimeStep(double manualTimeStep,double nDiscretizations){
        double minTimeStep=manualTimeStep;
        
        //have rotor time scales
        for(Rotor tmp:allRotorObjects){
            double rtrDT=tmp.getRecommendedTimeStep();
            simu.println("Rotor :"+tmp.getName()+ " dt_reccommend= "+rtrDT);
            if(minTimeStep>rtrDT){
                minTimeStep=rtrDT;
            }
        }

        simu.println("RTR DT: Final auto time step: "+minTimeStep);
        NumberFormat formattedDouble = new DecimalFormat("0.#E0");
        String truncatedTimeStep=formattedDouble.format(minTimeStep);
        return Double.parseDouble(truncatedTimeStep);
    }
    public boolean isUnsteady(){
        return isUns;
    }
    
    //SOLVER
    public void wallDistanceSolverTrickStep(){
        simu.getSolverManager().getSolver(WallDistanceSolver.class).setFrozen(false);
        simu.getSimulationIterator().step(1);
        //freeze wall distance solver for rotors
        simu.getSolverManager().getSolver(WallDistanceSolver.class).setFrozen(true);
    }
    
    //REGIONS
    private GeometryPart getRegionPart(Region tmpReg){
        Collection<GeometryPart> allRegPart = tmpReg.getPartGroup().getObjects();
        return(GeometryPart) allRegPart.toArray()[0];
    }
    private Collection<Region> getRegions(){
        return simu.getRegionManager().getRegions();
    }
    private Collection<Region> gatherRotatingBodyRegions(){
        Collection<Region> rotRegions = new ArrayList();
        for(Region tmp:simu.getRegionManager().getRegions()){
            simu.println("In Region: "+tmp.getPresentationName());
            if(!(tmp.getPhysicsContinuum()==null)){
                //Get motion and reference frame
                Motion tmpMotion=tmp.getValues().get(MotionSpecification.class).getMotion();
                ReferenceFrameBase tmpRefFrame=tmp.getValues().get(MotionSpecification.class).getReferenceFrame();
                
                // if Region is of type Rotation, or if Regions is Stationary and Reference Frame is of type rotation
                boolean refFrameIsRotating=tmpRefFrame instanceof RotatingReferenceFrame;
                boolean motionIsRotation  =tmpMotion instanceof RotatingMotion;
                boolean motionIsStationary=tmpMotion instanceof StationaryMotion;                
                if(motionIsRotation
                    ||(refFrameIsRotating&&motionIsStationary)){
                rotRegions.add(tmp);
            }
            }
        }
        return rotRegions;
    }
    private Region getPrimaryRegion(Collection<Region> allRegs){
        for(Region tmpReg:allRegs){
            Collection<Boundary> allBndy=tmpReg.getBoundaryManager().getBoundaries();
            //Main WT Region will contain Inlet or FS
            for(Boundary tmpBndy:allBndy){
                if((tmpBndy.getBoundaryType() instanceof InletBoundary) ||
                   (tmpBndy.getBoundaryType() instanceof FreeStreamBoundary)){
                    return tmpReg;
                }
            }
        }
        return null;
    }

    // BOUNDARIES
    private void cleanupEmptyBoundaries(Collection<Region> allRegs){
        for(Region tmp:allRegs){
            String regName =tmp.getPresentationName();
            Collection<Boundary> defaultBndys = tmp.getBoundaryManager()
                    .getBoundaries();
            for(Boundary tmpBndy:defaultBndys){
                String bndyName=tmpBndy.getPresentationName();
                if(tmpBndy.getPartSurfaceGroup().getObjects().size()<1){
                    try{
                      tmp.getBoundaryManager().remove(tmpBndy);
                    }catch(NeoException e){
                        simu.println("RTR MSG: Boundary " +
                            bndyName+" has a mesh!!!");
                    }
                }
            }
        }
    }

    //
    //POST PROCESSING
    public void initPostProc(String outputPath,boolean isUnsteady,int unsStepFreq){
        simu.println("RTR PST: Setting up Annotations:");
        // Set CaseName Annotation
        initAnnotations();

        simu.println("RTR PST: Is this an unsteady case:" +isUnsteady);
        simu.println("RTR PST: Rotor Specific Post");
        for(Rotor tmpRotor:allRBMRotorObjects){
            simu.println("RTR PST: Checking reports");
            tmpRotor.makeReports();
            simu.println("RTR PST: Checking iteration monitors");
            tmpRotor.makeIterationMonitors(simu,10000,1, 1);
            if(isUnsteady){
                simu.println("RTR PST: Checking unsteady monitors");
                tmpRotor.makeUnsteadyMonitors(5000,1, 1);
            }
            simu.println("RTR PST: Setting Up plots");
            tmpRotor.initRotorPlots(isUnsteady);
            simu.println("RTR PST: Setting Up scenes");
            tmpRotor.initRotorScenes();
            simu.println("RTR PST: Setting annotations....");
            
            for(Scene tmpScene:tmpRotor.getAllScenes()){
                SceneTool.removeDefaultLogo(tmpScene);
                for(Annotation tmpAnn:standardAnnotations){
                    SceneTool.addAnnotation(tmpScene,tmpAnn);
                }
                if(isUnsteady){
                    String userPath = outputPath + File.separator + "USER";
                     SystemTool.touchDirectory(userPath);
                    int lastIndex = tmpRotor.getName().length();
                    String rtrShortName = tmpRotor.getName().substring(4, lastIndex);
                    String rtrPath = userPath + File.separator +rtrShortName;
                    SystemTool.touchDirectory(rtrPath);
                    String moviePath = rtrPath + File.separator + "MOVIE";
                    SystemTool.touchDirectory(moviePath);
                    simu.println("RTR PST: Case is Unsteady. Setting unsteady Scenes...");
                    SceneTool.setSceneUnsteadyOutput(tmpScene, moviePath,
                            inStr, unsStepFreq,outputRes[0],  outputRes[1] );
                }
            }
        }
        //================
        // Create Scenes for post processing
        //================
        Scene tmpScene;
        Scene tmpCFDScene;
        ScalarDisplayer scalarDisp;
        PartDisplayer partDisp;
        VectorMagnitudeFieldFunction vMFF;
        PrimitiveFieldFunction pFF;
        double[] zeroOrigin = {0.0, 0.0, 0.0};
        double[] viewUp= {0.,0.,-1.0};
        
        //VIEW Setup 
        ArrayList<VisView> rtr2DViewList = new ArrayList();
        double[] yNormCameraPos = {0.0,-200.0*viewAngleDistanceOffset,0.0};
        double[] yNormFocalPt = {yNormCameraPos[0],0.,0.};
        VisView yNormView = SceneTool.getView(simu, "RTR 2D Auto View Y Nomral", labCsys,yNormFocalPt , yNormCameraPos, viewUp, true);
        rtr2DViewList.add(yNormView);
        for(Rotor tmpRotor:allRBMRotorObjects){
            double[] wt2DCameraPos = {tmpRotor.getBladeRadius(), 0.0, 1.0*tmpRotor.getBladeRadius()*viewAngleDistanceOffset};
            double[] wt2DfocalPt={wt2DCameraPos[0],0.,0.};
            VisView tmpView=SceneTool.getView(simu,"RTR 2D Auto View"+tmpRotor.getName(), labCsys,wt2DfocalPt, wt2DCameraPos, viewUp, true);
            tmpView.getParallelScale().setValue(1.0*tmpRotor.getBladeRadius()*2.0);  
            Units units_0 = ((Units) simu.getUnitsManager().getObject("m"));
            tmpView.getFocalPointCoordinate().setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {wt2DCameraPos[0], 0.0, 0.0}));
            tmpView.getViewUpCoordinate().setCoordinate(units_0, units_0, units_0, new DoubleVector(viewUp));
           tmpView.getPositionCoordinate().setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {wt2DCameraPos[0], -wt2DCameraPos[2],0.0}));
            rtr2DViewList.add(tmpView);
        }
        //List of all Scenes and boundaries
        ArrayList<Scene> allRTRScenes = new ArrayList();        

        ArrayList<Boundary> outlineBoundaries = new ArrayList();
        for(Domain tmpDomain:allDomainObjects){
          for(Boundary tmpBndy:tmpDomain.getRegion().getBoundaryManager().getBoundaries()){
            if(tmpBndy.getBoundaryType() instanceof SymmetryBoundary){
              outlineBoundaries.add(tmpBndy);
            }
          }
        }
        //New collection of Regions because previous was not putting all in derived part
        ArrayList<Region> allRegions = new ArrayList();
        for(Domain tmpR:allDomainObjects){
            allRegions.add(tmpR.getRegion());
        }
                
        //Derived Parts
        double[] centerY = {0.,0.,0.};
        PlaneSection labCsysYNorm = DerivedPartTool.singlePlane(simu,allRegions,"Lab Y=0 NormalY",labCsys,centerY,yVector);
        ArrayList<PlaneSection> yNormColl = new ArrayList();
        ArrayList<Double> refVelList = new ArrayList();
        yNormColl.add(labCsysYNorm);
        refVelList.add(new Double(refVel));
        for(Rotor tmpRotor:allRBMRotorObjects){
            CartesianCoordinateSystem tmpCsys;
            CartesianCoordinateSystem rtrCsys;
            rtrCsys =tmpRotor.getRotorCoordSys();
            
            //transform coordinate from local rotor to lab csys
            double[] originVectorInBody = rtrCsys.getOriginVector().toDoubleArray();
            double[][] b2sa = SimTool.getR_Y(-angleOfAttack*Math.PI/180);
            double[][] stab2aw = SimTool.getR_Z(sideSlipAngle*Math.PI/180);
            double[] origin_in_stab = SimTool.transformVector(SimTool.get3x3Transpose(b2sa),originVectorInBody);
            double[] origin_in_wind = SimTool.transformVector(SimTool.get3x3Transpose(stab2aw),origin_in_stab);
            
            String sectName = "Lab Y Normal "+tmpRotor.getName()+" origin";
            PlaneSection tmpY = DerivedPartTool.singlePlane(simu,allRegions,sectName,labCsys,origin_in_wind,yVector);
            yNormColl.add(tmpY);
            refVelList.add(tmpRotor.getTipSpeed());
        }
        
        // VORTICITY MAGNTIDUE SCENE
        simu.println("RTR POST: Setting Vorticity Magnitude Scene");
        int i = 0;
        for(PlaneSection tmpSect:yNormColl){
            tmpScene   = SceneTool.getScene(simu,"AUTO Vorticity Magnitude - "+tmpSect.getPresentationName());
            scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "omega_mag",  Collections.singleton(tmpSect));
            VorticityVectorFunction vortVF = 
              ((VorticityVectorFunction) simu.getFieldFunctionManager().getFunction("VorticityVector"));
            vMFF= ((VectorMagnitudeFieldFunction) vortVF.getMagnitudeFunction());
            double ts = refVelList.get(i).doubleValue();
            SceneTool.setScalarDisplayerField(scalarDisp,vMFF.getInternalName(),"Off","Off",0.0,ts*ts,proxyRep);
            partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
            SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
            scalarDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
            for(Annotation tmpAnn:standardAnnotations){
                SceneTool.addAnnotation(tmpScene,tmpAnn);
            }
            tmpScene.getCurrentView().setView(rtr2DViewList.get(i));
            if(i==0){
                tmpScene.resetCamera();
            }
            SceneTool.removeDefaultLogo(tmpScene);
            SceneTool.setScalarColormap(scalarDisp,"blue-red",32,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
            tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_omega_mag");
            tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
            tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(outputRes[0]);
            tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(outputRes[1]);
            //add to group
            allRTRScenes.add(tmpScene);

            if(!SceneTool.doesSceneExist(simu,"CFD Vorticity Magnitude")){
              tmpCFDScene = SceneTool.getScene(simu,"CFD Vorticity Magnitude - "+tmpSect.getPresentationName());
              tmpCFDScene.copyProperties(tmpScene);
              scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "omega_mag",Collections.singleton(tmpSect));
              //turn on the mesh and disable smoothing
              scalarDisp.setDisplayMesh(1); 
              scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
              SceneTool.setScalarColormap(scalarDisp,"blue-red",32,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
              scalarDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
              tmpScene.getCurrentView().setView(rtr2DViewList.get(i));
              tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("cfd_omega_mag");
              allRTRScenes.add(tmpCFDScene);
            }
            i++;
        }
        // VELOCITY MAGNTIDUE SCENE
        simu.println("RTR POST: Setting Velocity Magnitude Scene");
        int j=0;
        for(PlaneSection tmpSect:yNormColl){
            tmpScene   = SceneTool.getScene(simu,"AUTO Velocity Magnitude - "+tmpSect.getPresentationName());
            scalarDisp = SceneTool.getScalarDisplayer(tmpScene, "v_mag",  Collections.singleton(tmpSect));
            pFF = ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
            vMFF= ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
            SceneTool.setScalarDisplayerField(scalarDisp,vMFF.getInternalName(),"Off","Off",0.0,60.0,proxyRep);
            partDisp   = SceneTool.getPartDisplayer(tmpScene, "objs",outlineBoundaries, proxyRep);
            SceneTool.setPartDisplayerField(partDisp, true, false, false, false, 0);
            scalarDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
            for(Annotation tmpAnn:standardAnnotations){
                SceneTool.addAnnotation(tmpScene,tmpAnn);
            }
            tmpScene.getCurrentView().setView(rtr2DViewList.get(j));
            SceneTool.removeDefaultLogo(tmpScene);
            SceneTool.setScalarColormap(scalarDisp,"high-contrast",32,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
            tmpScene.getSceneUpdate().setAnimationFilenameBase("auto_vel_mag");
            tmpScene.getSceneUpdate().getHardcopyProperties().setUseCurrentResolution(false);
            tmpScene.getSceneUpdate().getHardcopyProperties().setOutputWidth(outputRes[0]);
            tmpScene.getSceneUpdate().getHardcopyProperties().setOutputHeight(outputRes[1]);
            //add to group
            allRTRScenes.add(tmpScene);

            if(!SceneTool.doesSceneExist(simu,"CFD Velocity Magnitude")){
              tmpCFDScene = SceneTool.getScene(simu,"CFD Velocity Magnitude - "+tmpSect.getPresentationName());
              tmpCFDScene.copyProperties(tmpScene);
              scalarDisp = SceneTool.getScalarDisplayer(tmpCFDScene, "v_mag",Collections.singleton(tmpSect));
              //turn on the mesh and disable smoothing
              scalarDisp.setDisplayMesh(1); 
              scalarDisp.setFillMode(ScalarFillMode.CELL_FILLED);
              SceneTool.setScalarColormap(scalarDisp,"high-contrast",32,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
              scalarDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
              tmpScene.getCurrentView().setView(rtr2DViewList.get(j));
              tmpCFDScene.getSceneUpdate().setAnimationFilenameBase("auto_vel_mag");
              allRTRScenes.add(tmpCFDScene);
            }
            j++;
        }
        //STREAMLINES
        simu.println("RTR POST: Setting Streamline Scenes");
        
        CartesianCoordinateSystem tmpCsys;
        CartesianCoordinateSystem tmpCsys2;
        CartesianCoordinateSystem rtrCsys;        
        tmpCsys =(CartesianCoordinateSystem) labCsys.getLocalCoordinateSystemManager().getObject(bodyCsys.getPresentationName());
                 
        ArrayList<StreamPart> rtrStreamlines = getRTRStreamLines(allRegions);
        int k=0;
        for(StreamPart tmpStream:rtrStreamlines){
            Rotor tmpRotor = allRBMRotorObjects.get(k);
                      
            tmpScene   = SceneTool.getScene(simu,"Streamlines - "+tmpStream.getPresentationName());
            pFF = ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
            vMFF = ((VectorMagnitudeFieldFunction) pFF.getMagnitudeFunction());
            StreamDisplayer tmpStreamDisp = SceneTool.getStreamLineDisplayer(tmpScene, "StreamLines-RTR", allRegions);
            tmpStreamDisp.getScalarDisplayQuantity().setFieldFunction(vMFF);
            SceneTool.setStreamScalarDisplayerProperties(tmpStreamDisp, "Off", "Off",40.0, 80.0, proxyRep); // kite speed is nominally 60 m/s
            SceneTool.setStreamColormap(tmpStreamDisp,"cool-warm",32,6,false,"Left Side"); 
            tmpStreamDisp.getInputParts().setObjects(tmpStream);
            
            //View for streamlines
            double[] wt2DCameraPos = {tmpRotor.getBladeRadius(), 0.0, 1.0*tmpRotor.getBladeRadius()*viewAngleDistanceOffset};
            double[] wt2DfocalPt={wt2DCameraPos[0],0.,0.};
            VisView tmpViewSL=SceneTool.getView(simu,"RTR SL View", tmpCsys,wt2DfocalPt, wt2DCameraPos, viewUp, true);
            Units units_0 = ((Units) simu.getUnitsManager().getObject("m"));
            tmpViewSL.setCoordinateSystem(tmpCsys);
            tmpViewSL.getFocalPointCoordinate().setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {wt2DCameraPos[0], 0.0, 0.0}));
            tmpScene.setViewOrientation(new DoubleVector(new double[] {-1.0, -1.0, 1.0}), new DoubleVector(new double[] {0.0, 0.0, -1.0}));
            CurrentView tCV = tmpScene.getCurrentView();
            ParallelScale tPS = tCV.getParallelScale();
            tPS.setValue(2.0*tmpRotor.getBladeRadius());
            Region rtrRegion = simu.getRegionManager().getRegion(tmpRotor.getName());
            ArrayList<Boundary> rtrDisp = new ArrayList();
            for(Boundary tmpB:rtrRegion.getBoundaryManager().getBoundaries()){
                if(tmpB.getPresentationName().contains(globalNames.getRotorID())){
                    rtrDisp.add(tmpB);
                }
            }
            PartDisplayer pD = SceneTool.getPartDisplayer(tmpScene, "Geometry", rtrDisp, proxyRep);
            SceneTool.setPartDisplayerField(pD, false, false, false, true, 0, 1);
            pD.getInputParts().setObjects(rtrDisp);
            pD.initialize();
            pD.setSurface(true);
            
            for(Annotation tmpAnn:standardAnnotations){
              SceneTool.addAnnotation(tmpScene,tmpAnn);
            }
            SceneTool.removeDefaultLogo(tmpScene);
            allRTRScenes.add(tmpScene);
            k++;
        }
        SceneTool.groupScenes(simu, "Tunnel Scenes", allRTRScenes);
        simu.println("RTR PST: Init Complete");
    }
    public void setPostProc(){
        int nPlotSamples=(int) 1.e5;
        int itMonFreq = 1;
        int itMonStart= 1;
        //
        //DERIVED PART SETUP
        //
        simu.println("RTR MSG: Generating Derived Parts.");
        // mesh metrics
        setupMeshMetrics(allWTRegs);

        // x-y plane for 3D
        double[] centerCutO = {0.,0.,0.0};
        PlaneSection xyCutPlane=singlePlane(allWTRegs,"Center Cut",bodyCsys,centerCutO,yVector);
    
        // default probe line for TVR/Ti
        LinePart lineProbe=getLineProbe(bodyCsys,"Inlet Line",100);

        // IsoSurface for reversed flow
        PrimitiveFieldFunction pFF = 
            ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
        VectorComponentFieldFunction vCFF = 
            ((VectorComponentFieldFunction) pFF.getFunctionInCoordinateSystem(inletCsys).getComponentFunction(0));
        getSingleIsoSurfPart(allWTRegs,"Reversed Flow", vCFF,-1.0);
        //
        // RELEVANT FIELD FUNCTIONS
        // pressure coefficient
        double refPress=wtPhysics.getContinuum().getReferenceValues()
                .get(ReferencePressure.class).getSIValue();
        SimTool.setPressureCoefficientFF(simu,refRho,0.0,refVel);

        // skin friction coefficient
        SimTool.setSkinFrictionCoefficientFF(simu,refRho,refVel);

        //
        //REPORTS & MONITORS
        //
        simu.println("RTR MSG: Setting residual monitors to absolute.");
        //setResidualsToAbsolute();
        
        simu.println("RTR MSG: Generating Reports & Monitors for Objects.");

        // Any Airfoils
        // REPORTS
        //
        //
        //
        //
        //
        // PLOTS
        //
        //
        // MONITOR Plots
        MonitorPlot tmpPlot;
        
        // Adjust residual plots
        PlotTool.adjustResidualPlot(simu,false);

    }
    public void runAllPostProc(String mainOutputDir){
        /* runAllPostProc runs through the output of csv, plot, and scene data
            arg1: mainOutputDir should be the base post-processing directory
                  e.g. /tmp/RotorStudy/AERO_POST/CaseName/
        */
        String userOut = mainOutputDir + File.separator + "USER";
        simu.println("in run post proc - useroutput file directory: "+userOut);
        String cfdOut  = mainOutputDir  +File.separator + "CFD";
        simu.println("in run post proc - cfd ouput  file directory: "+cfdOut);
        
        SystemTool.touchDirectory(userOut);
        SystemTool.touchDirectory(cfdOut);
        
        // track all monitors, plots, and scenes in the simulation
        //  take out any object specific ones and you have the user-defined
        //  ones!
        Collection<Monitor> customizedMonitors=simu.getMonitorManager()
                .getObjects();
        simu.println("number of Collection monitors: "+customizedMonitors.size());
        Collection<StarPlot> customizedPlots=simu.getPlotManager().getObjects();
        simu.println("number of Collection plots: "+customizedPlots.size());
        Collection<Scene> customizedScenes=simu.getSceneManager().getObjects();
        simu.println("number of scenes in collection: "+customizedScenes.size());
        // FROM ROTOR ANALYSIS ITSELF
        
        // FROM INDIVIDUAL ROTORS
        // Print out the Consumer Data
        for(Rotor tmpRotor:allRBMRotorObjects){
            simu.println("Rotor being processed: "+tmpRotor.getName());
            // CSV data files

            // Plots
            postProcessPlots(userOut,customizedPlots);
           // postProcessPlots(userOut,tmpRotor.getAllConsumerPlots());
            customizedPlots.removeAll(tmpRotor.getAllConsumerPlots());
            // Scenes
            postProcessScenes(userOut,customizedScenes);
            //postProcessScenes(userOut,tmpRotor.getAllConsumerScenes());
            postProcessScenes(userOut,tmpRotor.getAllConsumerVRScenes());
            customizedScenes.removeAll(tmpRotor.getAllConsumerScenes());
            customizedScenes.removeAll(tmpRotor.getAllConsumerVRScenes());
        }

        // Print out CFD Expert Data
        for(Rotor tmpRotor:allRBMRotorObjects){
          String rtrName = tmpRotor.getName();
          String rtrFileStr = cfdOut + File.separator + rtrName;
          SystemTool.touchDirectory(rtrFileStr);
          simu.println("Rotor file location: " + rtrFileStr);
          // CSV data files

          //These do not get the lists correctly tmpRotor.getAllxx()
          // Plots
          postProcessPlots(rtrFileStr, tmpRotor.getAllRotorPlots());

          // Scenes
          postProcessScenes(rtrFileStr, tmpRotor.getAllScenes());
        }

    }

    //PLOTS
    public void postProcessPlots(String outputFolder, Collection<StarPlot> outputPlots){
        for(StarPlot tmpPlot:outputPlots){
            simu.println("current plot to export is: "+tmpPlot.getTitle());
                tmpPlot.encode(outputFolder+File.separator+tmpPlot.getTitle()+".png"
                           , "png", outputRes[0], outputRes[1]);
                
        }
    }
    //SCENES
    public void postProcessScenes(String outputFolder, Collection<Scene> outputScenes){
        for(Scene tmpScene:outputScenes){
            simu.println("current scene to export is: "+tmpScene.getPresentationName());
            tmpScene.printAndWait(outputFolder+File.separator+tmpScene.getPresentationName()+".png",
                    1,outputRes[0], outputRes[1]);
           
        }
    }
    
    //DERIVED PARTS
    private void setupMeshMetrics(Collection<Region> myRegs){
        highValueCells(myRegs,"Metric High Skewness Angle","SkewnessAngle",95.0);
        lowValueCells(myRegs,"Metric Low Cell Quality","CellQuality",1e-4);
        lowValueCells(myRegs,"Metric Low Cell Volume","Volume",1e-4);
        lowValueCells(myRegs,"Metric Low Face Validity","FaceValidity",0.95);
        highValueCells(myRegs,"Prism Layer Cells","PrismLayerCells",0.0);
    }
    private ThresholdPart highValueCells(Collection<Region> bodyParts,String partName, String functionName, double highValue){
        ThresholdPart myPart;
        try{
            myPart = (ThresholdPart) simu.getPartManager().getObject(partName);
        }catch(NeoException e){
            Units units_0 = 
              ((Units) simu.getUnitsManager().getObject("m"));
            NullFieldFunction nullFieldFunction_0 = 
              ((NullFieldFunction) simu.getFieldFunctionManager().getFunction("NullFieldFunction"));
            myPart = 
              (ThresholdPart) simu.getPartManager().createThresholdPart(new NeoObjectVector(new Object[] {}), new DoubleVector(new double[] {0.0, 1.0}), units_0, nullFieldFunction_0, 0);
            myPart.setPresentationName(partName);
        }
        myPart.getInputParts().setQuery(null);
        myPart.getInputParts().setObjects(bodyParts);
        myPart.setFieldFunction(simu.getFieldFunctionManager().getFunction(functionName));
        myPart.setMode(ThresholdMode.ABOVE_TAG);
        myPart.getRangeQuantities().setArray(new DoubleVector(new double[] {highValue, highValue}));
        return myPart;
    }
    private ThresholdPart lowValueCells(Collection<Region> bodyParts,String partName, String functionName, double lowValue){
        ThresholdPart myPart;
        try{
            myPart = (ThresholdPart) simu.getPartManager().getObject(partName);
        }catch(NeoException e){
            Units units_0 = 
              ((Units) simu.getUnitsManager().getObject("m"));
            NullFieldFunction nullFieldFunction_0 = 
              ((NullFieldFunction) simu.getFieldFunctionManager().getFunction("NullFieldFunction"));
            myPart = 
              (ThresholdPart) simu.getPartManager().createThresholdPart(new NeoObjectVector(new Object[] {}), new DoubleVector(new double[] {0.0, 1.0}), units_0, nullFieldFunction_0, 0);
            myPart.setPresentationName(partName);
        }
        myPart.getInputParts().setQuery(null);
        myPart.getInputParts().setObjects(bodyParts);
        myPart.setFieldFunction(simu.getFieldFunctionManager().getFunction(functionName));
        myPart.setMode(ThresholdMode.BELOW_TAG);
        myPart.getRangeQuantities().setArray(new DoubleVector(new double[] {lowValue, lowValue}));
        return myPart;
    }
    private PlaneSection getPlane(String planeName){
        return ((PlaneSection) simu.getPartManager().getObject(planeName));
    }
    private IsoPart getIso(String planeName){
        return ((IsoPart) simu.getPartManager().getObject(planeName));
    }
    private LinePart getLineProbe(CoordinateSystem probeCsys,String probeName,int nPts){
        LinePart lineProbe;
        try{
            lineProbe=(LinePart) simu.getPartManager().getObject(probeName);
        }catch(NeoException e){
            lineProbe=simu.getPartManager().createLinePart(
                    new NeoObjectVector(new Object[] {}),
                    new DoubleVector(new double[] {0.0, 0.0, 0.0}),
                    new DoubleVector(new double[] {1.0, 0.0, 0.0}), 20);
            lineProbe.setPresentationName(probeName);
        }
          
        lineProbe.setCoordinateSystem( probeCsys);
        lineProbe.setResolution(nPts);
        lineProbe.getInputParts().setObjects(allWTRegs);

        Coordinate coordinate_0 = 
           lineProbe.getPoint2Coordinate();

        Units units_0 = 
          ((Units) simu.getUnitsManager().getObject("m"));

        coordinate_0.setCoordinate(units_0, units_0, units_0,
                new DoubleVector(new double[] {lengthScale*10.0, 0.0, 0.0}));
        return lineProbe;
    }
    private PlaneSection singlePlane(Collection<Region> myRegs, String planeName,CoordinateSystem myCsys, double[] newOrigin, double[] newNormal){
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
        Coordinate myOrigin = myPlane.getOriginCoordinate();
        myOrigin.setCoordinate(units_0, units_0, units_0, new DoubleVector(newOrigin));
        myPlane.setPresentationName(planeName);
        myPlane.getInputParts().setObjects(myRegs);
        return myPlane;
    }
    private PlaneSection multiPlane(Collection<Region> myRegs, String planeName,CoordinateSystem myCsys,double[] newOrigin, double[] newNormal, int nSec, double minVal,double maxVal){
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
    private IsoPart getSingleIsoSurfPart(Collection<Region> myRegs, String isoName, FieldFunction tmpFF,double newValue){
        IsoPart retPart;
        try{
            retPart = (IsoPart) simu.getPartManager().getObject(isoName);
        }catch(NeoException e){
            NullFieldFunction nullFieldFunction_0 = 
              ((NullFieldFunction) simu.getFieldFunctionManager().getFunction("NullFieldFunction"));
            retPart = 
              simu.getPartManager().createIsoPart(new NeoObjectVector(new Object[] {}), nullFieldFunction_0);
        }
        retPart.setPresentationName(isoName);
        retPart.setMode(IsoMode.ISOVALUE_RANGE);
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
    private ArrayList<StreamPart> getRTRStreamLines(Collection<Region> myRegs){
        // Rotor specific streamline parts
        ArrayList<StreamPart> retParts = new ArrayList();
        Units mUnit = ((Units) simu.getUnitsManager().getObject("m"));
        StreamPart thisSL = null;
        PrimitiveFieldFunction pFF = 
           ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("Velocity"));
           
        for(Rotor tmpRtr:allRBMRotorObjects){
            Region rtrRegion = simu.getRegionManager().getRegion(tmpRtr.getName());
            //Get "009" Boundaries to use as seed Parts for streamline
            ArrayList<Boundary> seedParts = new ArrayList();
            for(Boundary tmpB:rtrRegion.getBoundaryManager().getBoundaries()){
                if(tmpB.getPresentationName().contains(globalNames.getSlidingStr())){
                    seedParts.add(tmpB);
                }
            }
            thisSL = simu.getPartManager().createStreamPart(new Vector(myRegs), new NeoObjectVector(seedParts.toArray()), pFF, 5, 5, 2);
            thisSL.setPresentationName("SL-"+tmpRtr.getName());
            retParts.add(thisSL);
        }
       return retParts;        
    }

    //COORDINATE SYSTEMS
    private ArrayList<CartesianCoordinateSystem> getPartNamedCoordinateSystems(CoordinateSystem tmpCsys,ArrayList<GeometryPart> namedParts){
        ArrayList<CartesianCoordinateSystem> retArrList=new ArrayList();
        for(GeometryPart tmpPart:namedParts){
            String crdName=tmpPart.getPresentationName();
            try{
                retArrList.add((CartesianCoordinateSystem) tmpCsys.getLocalCoordinateSystemManager().getObject(crdName));
            }catch(NeoException e){
                simu.println("FC MSG: named coordinate "+crdName+" not found. Will likely fail!");
            }
        }
        return retArrList;
    }
    
}

