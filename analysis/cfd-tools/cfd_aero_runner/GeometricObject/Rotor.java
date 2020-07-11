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
package GeometricObject;

import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.meshing.*;
import star.base.report.*;
import star.flow.ForceReport;
import star.flow.MomentReport;
import star.flow.ForceCoefficientReport;
import star.flow.MomentCoefficientReport;

import Tools.*;
import Naming.*;
import MeshTools.*;
import star.base.neo.NeoException;
import star.meshing.SurfaceCustomMeshControl;
import Domain.RotatingDomain;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import star.energy.StaticTemperatureProfile;
import star.flow.InitialPressureProfile;
import star.flow.VelocityProfile;
import star.motion.*;
import star.vis.*;

/**
 * Class: Rotor
 * 
 * Purpose: This class allows the definition of *any* generic rotor: 
 *          RBM, Overset, or BEM
 * 
 * Usage: The rotor itself is created by the presence of either a CAD system
 *        with the PreFix "BEM" for a BEM rotor model, or in the presence of a
 *        physical part containing the PartSurface w/ associated PreFix names
 *
 */
public class Rotor{
    Simulation simu;
    Representation proxyRep;
    
    NamingConvention globalNames;
    SimTool simTool;

    
    // FINAL
    final double SMALL_EPS = 1.0E-6;
    

    //Geometry Information
    String rotorName="";

    
    GeometryPart geomPart;
    Collection<PartSurface> geomSurfs = new ArrayList();
    Collection<PartSurface> bladeSurfs = new ArrayList();
    Collection<PartSurface> extraSceneBodyParts = new ArrayList();

    String preFix = "xxxx";
    int preFixSize = preFix.length();

    //Domain info
    RotatingDomain parentRotatingDomain;
    
    boolean isBEMRotor=false;
    
    //Region
    Region parentRegion;
    
    //Derived Parts
    PlaneSection zNormalInterfacePlane;
    
    // Coordinate Systems
    String bodyCsysName;
    String rotorCsysName;
    
    LabCoordinateSystem labCsys;
    CartesianCoordinateSystem cadCsys;
    CartesianCoordinateSystem bodyCsys;
    CartesianCoordinateSystem rotorCsys;
    CylindricalCoordinateSystem thetaCsys;
    CartesianCoordinateSystem motionRotorCsys;
    
    double[][] rotor2Body = new double[3][3];
    double[][] body2Lab = new double[3][3];
    

    // Motion stuff
    StationaryMotion stationaryMotion;
    RotatingMotion rotorMotion;
    RotatingReferenceFrame rotorRotatingFrame;
    MotionSpecification rotorMotionSpecification;
    LabReferenceFrame labFrame;

    //Monitors
    ArrayList<Monitor> allConsumerMonitors = new ArrayList();
    ArrayList<Monitor> allCFDMonitors = new ArrayList();
    
    //Plots
    ArrayList<StarPlot> allRotorPlots = new ArrayList();

    //Scenes
    ArrayList<Scene> allConsumerScenes = new ArrayList();
    ArrayList<Scene> allConsumerVRScenes = new ArrayList();
    ArrayList<Scene> allCFDScenes = new ArrayList();
    
    //axis shorthand
    double[] zeroOrigin = {0.0, 0.0, 0.0};
    double[] xVector    = {1.0, 0.0, 0.0};
    double[] yVector    = {0.0, 1.0, 0.0};
    double[] zVector    = {0.0, 0.0, 1.0};
    
    // Reference Values
    // global
    double[] refMomR; //      [m] reference Moment Radius about {x,y,z}
    double refArea;   //     [m2] reference Area
    double refVel;    //    [m/s] reference Velocity
    double refRho;    //  [kg/m3] reference Density
    double refLengthScale;  //  [m] reference Length Scale
    double refSpeedOfSound; //[m/s] speed of sounds

    // angles
    double alphaAngle; // [deg] alpha angle
    double betaAngle; // [deg] beta angle
    
    //rotor specific
    int nBlades;          //       [#] number of real/virtual blades on Rotor
    double bladeRadius;   //       [m] avg. radius of rotor blades
    double bladeArea;     //     [m^2] total swept area of the blades
    double rotorSpeed;    //   [rad/s] rotor speed
    double tipSpeed;      //     [m/s] rotor tip speed
    double tipMach;       //       [#] Mach number at blade tip
    double bladePassFreq; //      [Hz] blade passing freq (cam control mesh/dt)
    double recommendedTimeStep; // [s] recommended time step based on bladePassFreq

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
    
    //Reporting Types
    String bodyStr="Body";
    String rotorStr="Rotor";
    
    // Necessary Reports
    ArrayList<Report> rotorReports = new ArrayList();
    ArrayList<Report> bodyReports = new ArrayList();
    ArrayList<Report> propellerReports = new ArrayList();
    ArrayList<Report> helicopterReports = new ArrayList();
    ArrayList<Report> wallYplusReports = new ArrayList();

    //Monitors
    String itPostFix;
    String unsPostFix;
    
    // Necessary Iteration Monitors
    ArrayList<Monitor> rotorIterMonitors = new ArrayList();
    ArrayList<Monitor> bodyIterMonitors = new ArrayList();
    ArrayList<Monitor> wallYplusIterMonitors = new ArrayList();
    ArrayList<Monitor> helicopterIterMonitors = new ArrayList();
    ArrayList<Monitor> propellerIterMonitors = new ArrayList();

    // Optional Unsteady Monitors
    ArrayList<Monitor> rotorUnsMonitors = new ArrayList();
    ArrayList<Monitor> bodyUnsMonitors = new ArrayList();
    ArrayList<Monitor> wallYplusUnsMonitors = new ArrayList();
    ArrayList<Monitor> helicopterUnsMonitors = new ArrayList();
    ArrayList<Monitor> propellerUnsMonitors = new ArrayList();
    
    //Statistics
    int statSamples=500;
    
    public Rotor(Simulation simu,int preFixSize,
            CartesianCoordinateSystem cadCsys){
        //Generic sim tool builders
        //
        //Familiarize
        this.simu=simu;
        this.globalNames = new NamingConvention();
        this.proxyRep=SimTool.getSimProxy(simu,"Proxy");
      
        //Monitor stuff
        this.itPostFix=globalNames.getItPostFix();
        this.unsPostFix=globalNames.getUnsPostFix();
        
        //Coordinate System Stuff
        this.labCsys=simu.getCoordinateSystemManager()
                .getLabCoordinateSystem();
        this.bodyCsysName = globalNames.getBodyCsysName();
        this.bodyCsys=(CartesianCoordinateSystem) labCsys
                .getLocalCoordinateSystemManager().getObject(bodyCsysName);
        this.cadCsys=cadCsys;
        //
        this.rotorCsys=SimTool.labBasedCoordToBodyCoord(simu,cadCsys,bodyCsys);
        SimTool.useLabBasisForLocalBasis(cadCsys,rotorCsys); //stay aligned w/ rotated body
        this.rotorCsysName=rotorCsys.getPresentationName();
        //
        this.thetaCsys=SimTool.getNestedCylindricalCoordinate(rotorCsys, "ThetaCsys");
        this.rotor2Body = SimTool.getBasisSet(rotorCsys);
        this.body2Lab =  SimTool.getBasisSet(bodyCsys);
        simu.println("body2Lab");
        SimTool.simPrint3x3Matrix(simu,body2Lab);
        simu.println("rotor2body");
        SimTool.simPrint3x3Matrix(simu,rotor2Body);
        
        
    }
    //==========================
    // PUBLIC METHODS
    //==========================
    // ROTOR OBJECT INFO
    public String getName(){
        return rotorName;
    }
    public void   setName(String newName){
        rotorName=newName;
    }
    public boolean isBEM(){
        return isBEMRotor;
    }
    public void setIsBEM(boolean isBEM){
        isBEMRotor=isBEM;
    }
    //
    // Parent domain relationship
    public void setRotatingDomain(RotatingDomain tmpDom){
        parentRotatingDomain=tmpDom;
        parentRotatingDomain.setCoordinateSystem(rotorCsys);
    }
    public RotatingDomain getRotatingDomain(){
        return parentRotatingDomain;
    }
    //
    // Intrinsic Rotor Charactersistics
    public int    getNBlades(){
        nBlades=getNBladesFromCsys();
        return nBlades;
    }
    public double getBladeRadius(){
        bladeRadius = getBladeRadiusFromCsys();
        bladeArea   = Math.PI*bladeRadius*bladeRadius;
        return bladeRadius;
    }
    public double getRotorSpeed(){
        rotorSpeed = getOmegaFromCsys();
        return rotorSpeed;
    }
    public double getTipSpeed(){
        return tipSpeed;
    }
    public double getRecommendedTimeStep(){
        return 1.0/(bladePassFreq*20.0);
    }
    public double getRecommendedMeshSize(){
        //Note, there is an assumption of an advection speed of 40/ms in this
        // calculation. This was explored in a technical report.
        double V_ax=40.0;
        double lambda=V_ax/bladePassFreq;
        return lambda/20.0;
    }
    //
    // Update Rotor Characteristic
    public void updateRotorSpeed(double newVal){
      //Update to rotor itself
      boolean updateRotorMotionSpeed = false;
      if(Math.abs(rotorSpeed - newVal) > SMALL_EPS) updateRotorMotionSpeed = true;
      
      simu.println("updateRotorSpeed: " + updateRotorMotionSpeed);
      rotorSpeed = newVal;
      // need to update the coordinate system name
      if(parentRotatingDomain != null){
        simu.println("parentDomain: "+updateRotorMotionSpeed);
        if(updateRotorMotionSpeed){
          simu.println("time to update! "+updateRotorMotionSpeed);
          rotorMotion.getRotationRate().setValue(newVal);
        }
      }

      //Update to propeller and helicopter references
      setPropellerReferences( newVal,2.0*bladeRadius,refRho);
      setHelicopterReferences(newVal,    bladeRadius,refRho);

      //Need to modify *all*coordinate system names
      //  because on reload, the coordinate system name
      //  sets the omega value
      updateRotorCoordinateSystemSpeeds(newVal);
    }
    //
    // Reference Values
    // - sets
    public void setSpeedofSound(double newVal){
        refSpeedOfSound=newVal;
    }
    public double   getRefVel(){
        return refVel;
    }
    public double   getRefRho(){
        return refRho;
    }
    public double[] getRefMomR(){
        return refMomR;
    }
    public double   getRefArea(){
        return refArea;
    }
    // - gets
    public void setRefVel(double newVal){
      refVel=newVal;
      ArrayList<Report> bothAeroBodyReports=new ArrayList();
      bothAeroBodyReports.addAll(rotorReports);
      bothAeroBodyReports.addAll(bodyReports);
      for(Report tmpReport:bothAeroBodyReports){
          if(tmpReport instanceof ForceCoefficientReport){
              ((ForceCoefficientReport) tmpReport).getReferenceVelocity().setValue(newVal);
          }
          if(tmpReport instanceof MomentCoefficientReport){
              ((MomentCoefficientReport) tmpReport).getReferenceVelocity().setValue(newVal);
          }
      }
    }
    public void setRefRho(double newVal){
        refRho=newVal;
        ArrayList<Report> bothAeroBodyReports=new ArrayList();
        bothAeroBodyReports.addAll(rotorReports);
        bothAeroBodyReports.addAll(bodyReports);
        for(Report tmpReport:bothAeroBodyReports){
            if(tmpReport instanceof ForceCoefficientReport){
                ((ForceCoefficientReport) tmpReport).getReferenceDensity().setValue(newVal);
            }
            if(tmpReport instanceof MomentCoefficientReport){
                ((MomentCoefficientReport) tmpReport).getReferenceDensity().setValue(newVal);
            }
        }
    }
    public void setRefMomR(double[] newVals){
        refMomR=newVals;
        ArrayList<Report> bothAeroBodyReports=new ArrayList();
        bothAeroBodyReports.addAll(rotorReports);
        bothAeroBodyReports.addAll(bodyReports);
        for(Report tmpReport:bothAeroBodyReports){
            if(tmpReport instanceof MomentCoefficientReport){
                double xAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(0);
                double yAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(1);
                double zAxisVal=((MomentCoefficientReport) tmpReport).getDirection().getInternalVector().getComponent(2);
                if(Math.abs(xAxisVal-1.0)>1e-5){
                    ((MomentCoefficientReport) tmpReport).getReferenceRadius().setValue(newVals[0]);
                }else if(Math.abs(yAxisVal-1.0)>1e-5){
                    ((MomentCoefficientReport) tmpReport).getReferenceRadius().setValue(newVals[1]);
                }else if(Math.abs(zAxisVal-1.0)>1e-5){
                    ((MomentCoefficientReport) tmpReport).getReferenceRadius().setValue(newVals[2]);
                }
            }
        }
    }
    public void setRefArea(double newVal){
        refArea=newVal;
        ArrayList<Report> bothAeroBodyReports=new ArrayList();
        bothAeroBodyReports.addAll(rotorReports);
        bothAeroBodyReports.addAll(bodyReports);
        
        for(Report tmpReport:bothAeroBodyReports){
            if(tmpReport instanceof ForceCoefficientReport){
                ((ForceCoefficientReport) tmpReport).getReferenceArea().setValue(newVal);
            }
            if(tmpReport instanceof MomentCoefficientReport){
                ((MomentCoefficientReport) tmpReport).getReferenceArea().setValue(newVal);
            }
        }
    }
    public void setReferenceValues(double refRho,double refVel){
        // Reference Values
        setRefVel(refVel);
        setRefRho(refRho);
        
        // Internal rotor specific stuff
        tipSpeed=Math.abs(rotorSpeed)*bladeRadius;
        tipMach=tipSpeed/refSpeedOfSound;
        bladePassFreq=Math.abs(Math.abs(rotorSpeed)*nBlades/(2.0*Math.PI));
        //helicopter normalizations
        setHelicopterReferences(Math.abs(rotorSpeed),bladeRadius*2,refRho);
        //propeller normalizations
        setPropellerReferences(Math.abs(rotorSpeed),bladeRadius,refRho);
        
    }
    public void setPropellerReferences(double rotorOmega,double bladeDiameter,double newRho){
        rotorSpeed=rotorOmega;
        double aTilde = 0.25*rotorOmega*rotorOmega/(Math.PI*Math.PI);
        double vTilde = Math.pow(bladeDiameter,4);
        for(Report tmpReport:propellerReports){
            if(tmpReport instanceof ForceCoefficientReport){
                ((ForceCoefficientReport) tmpReport).getReferenceVelocity().setValue(vTilde);
                ((ForceCoefficientReport) tmpReport).getReferenceArea().setValue(aTilde);
                ((ForceCoefficientReport) tmpReport).getReferenceDensity().setValue(newRho);
            }
            if(tmpReport instanceof MomentCoefficientReport){
                ((MomentCoefficientReport) tmpReport).getReferenceVelocity().setValue(vTilde);
                ((MomentCoefficientReport) tmpReport).getReferenceArea().setValue(aTilde);
                ((MomentCoefficientReport) tmpReport).getReferenceRadius().setValue(bladeDiameter);
                ((MomentCoefficientReport) tmpReport).getReferenceDensity().setValue(newRho);
            }
        }
    }
    public void setHelicopterReferences(double rotorOmega,double bladeRadius,double newRho){
        // We utilize the V~ in the STAR-CCM+ Reports instead of making specials
        rotorSpeed=rotorOmega;
        double vTilde = 0.25*rotorOmega*bladeRadius;
        for(Report tmpReport:helicopterReports){
            if(tmpReport instanceof ForceCoefficientReport){
                ((ForceCoefficientReport) tmpReport).getReferenceVelocity().setValue(vTilde);
                ((ForceCoefficientReport) tmpReport).getReferenceDensity().setValue(newRho);
            }
            if(tmpReport instanceof MomentCoefficientReport){
                ((MomentCoefficientReport) tmpReport).getReferenceVelocity().setValue(vTilde);
                ((MomentCoefficientReport) tmpReport).getReferenceDensity().setValue(newRho);
            }
        }
    }
    //
    // GEOMETRY 
    public GeometryPart getGeometryPart(){
        return geomPart;
    }
    public void updatePhysicalPartSurfaces(){
        /* method updatePhysicalPartSurfaces is used to update part surfaces
            that get dropped out of MeshOperations due to their total removal
            after the result of a Part Operation
        */
        Collection<PartSurface> rotorSurfaces=geomPart.getPartSurfaces();
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
        geomSurfs=filteredSurfaces;
    }
    public void setPhysicalPartSurfaces(ArrayList<PartSurface> geomSurfs){
        //Geometry
        this.geomSurfs=geomSurfs;
        this.geomPart=geomSurfs.get(0).getPart(); //all surfs better have XXXX
        this.rotorName=geomPart.getPresentationName();

        setupBladeSurfs(); //fills the blade surface array
        
        //If there is a physical Part Surface, this puppy will rotate
        // need to create a second rotating coordinate system to keep pictures
        // oriented properly, or to look at specific blade moments
        this.motionRotorCsys=SimTool.labBasedCoordToBodyCoord(simu,cadCsys,rotorCsys);
    }
    public void setAlphaAngle(double newAngle){
        alphaAngle=newAngle;
    }
    public double getAlphaAngle(){
        return alphaAngle;
    }
    public void setBetaAngle(double newAngle){
        betaAngle=newAngle;
    }
    public double getBetaAngle(){
        return betaAngle;
    }
    public GeometryPart createRBMRotorWakePart(){
        String vcPreFix = globalNames.getVCPreFix();
        
        // if the volume control part exists, don't modify it
        for(GeometryPart tmpPart:simu.getGeometryPartManager().getObjects()){
            String tmpName = tmpPart.getPresentationName();
            boolean isNameVC = tmpName.startsWith("VC_")||tmpName.contains(".VC_");
            boolean isRTRVC = tmpName.contains(rotorName.substring(4));
            if(isNameVC && isRTRVC){ 
                return tmpPart;
            }
        }
        // otherwise we need to make the rotor VC via:
        // Two cylinders at origin oriented along Z-axis of extrude
        //   first is extrude distance of 0.5*b
        //   second is downstream distance of 5*b
        //   cone draft is 2.5 deg
        double foreEx=0.5*bladeRadius; //z-axis value
        double aftEx=-5.0*bladeRadius; //z-axis value
        double outerCylRad=1.2*bladeRadius;
        double innerCylRad=0.025*bladeRadius;
        double coneDraftatan=Math.atan(2.5*Math.PI/180.0); // arctan of draft out ange in deg
        double coneDownRad=outerCylRad+(0.0-aftEx)*coneDraftatan;
        SimpleCylinderPart outerCyl=GeometryTool.getSimpleCylinder(simu,cadCsys,foreEx,0.0,outerCylRad);
        SimpleCylinderPart innerCyl=GeometryTool.getSimpleCylinder(simu,cadCsys,foreEx,aftEx,innerCylRad);
        SimpleConePart downStreamCone=GeometryTool.getSimpleCone(simu,cadCsys, 0.0, aftEx, outerCylRad, coneDownRad);
        MeshActionManager meshActionManager = simu.get(MeshActionManager.class);
        CadPart bigVol = (CadPart) meshActionManager.uniteParts(new NeoObjectVector(new Object[] {outerCyl, downStreamCone}), "CAD");
        CadPart rtr_CV = 
            (CadPart) meshActionManager.subtractParts(new NeoObjectVector(new Object[] {innerCyl, bigVol}), bigVol, "CAD");
        bigVol.destroy();
        downStreamCone.destroy();
        innerCyl.destroy();
        outerCyl.destroy();
        rtr_CV.setPresentationName(vcPreFix+"_"+rotorName.substring(4)+"_200.0");
        return rtr_CV;
        
    }
    //
    // MESHING
    public void setSurfMeshCustomControl(SurfaceCustomMeshControl newControl){
        surfCustomMeshSettings=newControl;
    }
    public void setSurfMeshCustomTEControl(SurfaceCustomMeshControl newControl){
        surfCustomMeshSettingsTE=newControl;
    }
    public void setVolMeshCustomControl(SurfaceCustomMeshControl newControl){
        volCustomMeshSettings=newControl;
    }
    public void setVolMeshCustomTEControl(SurfaceCustomMeshControl newControl){
        volCustomMeshSettingsTE=newControl;
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
        customTargetSurfaceSize=MeshOpTool.getSurfCustTargetSize(surfCustomMeshSettings);
        customMinSurfaceSize=MeshOpTool.getSurfCustMinSize(surfCustomMeshSettings);
        customNPtsOnCircle=MeshOpTool.getSurfCustNPtsOnCircle(surfCustomMeshSettings);
    }
    public void getAnyVolumeMeshCustomControlSettings(){
        customFirstCellThickness=MeshOpTool.getCustomPrismNearWall(volCustomMeshSettings);
        customPrismThickness    =MeshOpTool.getCustomPrismThick(volCustomMeshSettings);
        customPrismLayers       =MeshOpTool.getCustomNumPrismLayers(volCustomMeshSettings);
    }
    //
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
        customTargetSurfaceSize=newVal;
        //
        simu.println("Rotor MSG: Setting Custom Target Surface Size of "+newVal+" on "+rotorName);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("...   using control: "+volCustomMeshSettings.getPresentationName());
        //
        MeshOpTool.surfCustTargSize(surfCustomMeshSettings  , "Relative", newVal);
        MeshOpTool.surfCustTargSize(surfCustomMeshSettingsTE, "Relative", newVal);
    }
    public void setCustomMinSurfaceSize(double newVal){
        customMinSurfaceSize=newVal;
        //msg
        simu.println("Rotor MSG: Setting Custom Minimum Surface Size of "+newVal+" on "+rotorName);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("...   using control: "+volCustomMeshSettings.getPresentationName());
        //
        MeshOpTool.surfCustMinSize(surfCustomMeshSettings  , "Relative", newVal);
        MeshOpTool.surfCustMinSize(surfCustomMeshSettingsTE, "Relative", newVal);
    }
    public void setCustomNPtsOnCircle(double newVal){
        customNPtsOnCircle=newVal;
        //msg
        simu.println("Rotor MSG: Setting Custom Number of Points "+newVal+" on "+rotorName);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("...   using control: "+volCustomMeshSettings.getPresentationName());
        //
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
        //
        customFirstCellThickness=newVal;
        //
        simu.println("Rotor MSG: Setting First Cell Thickness of "+newVal+"on "+rotorName);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("...   using control: "+volCustomMeshSettings.getPresentationName());
        //
        MeshOpTool.surfPrismNearWall(volCustomMeshSettings,newVal);
        //TE
        MeshOpTool.surfPrismNearWall(volCustomMeshSettingsTE,newVal*4.0);
    }
    public void setCustomPrismAbsThickness(double newVal){
        customPrismThickness=newVal;
        //msg
        simu.println("Rotor MSG: Setting Prism Layer Absolute Thickness for : "+rotorName+"at "+newVal);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("    ...using control: "+volCustomMeshSettings.getPresentationName());
        //
        MeshOpTool.surfPrismThick(volCustomMeshSettings,"Absolute",newVal);
        //TE
        MeshOpTool.surfPrismThick(volCustomMeshSettingsTE,"Absolute",newVal*0.25);
    }
    public void setCustomNumPrismLayers(int newVal){
        customPrismLayers=newVal;
        //msg
        simu.println("Rotor MSG: Setting Number of Prism Layers for : "+rotorName+"at "+newVal);
        simu.println("...in mesh operation: "+volCustomMeshSettings.getManager().getMeshOperation());
        simu.println("    ...using control: "+volCustomMeshSettings.getPresentationName());
        MeshOpTool.surfPrismNumLayers(volCustomMeshSettings,newVal);
        //TE
        MeshOpTool.surfPrismNumLayers(volCustomMeshSettingsTE,newVal/2);
    }
    //
    // REGIONS
    public Region getParentRegion(){
        return parentRegion;
    }
    public void setParentRegion(Region newRegion){
        parentRegion=newRegion;
    }
    public void setUniqueBoundary(String bndyID){
        Boundary tmpBndy = setUpBoundary(globalNames.getBndyName(bndyID));
        //add necessary surfaces to the boundary
        tmpBndy.getPartSurfaceGroup().setObjects(geomSurfs);
        //Set BC Type
        tmpBndy.setBoundaryType(WallBoundary.class);
    }
    public void setUniqueBladeBoundary(){
        String tmpRtrID = globalNames.getRotorBladeID();
        Boundary tmpBndy = setUpBoundary(globalNames.getBndyName(tmpRtrID+"00 Rotor Blades"));
        tmpBndy.getPartSurfaceGroup().addObjects(bladeSurfs);
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
    //
    //PHYSICS
   
    private FieldFunction getLabBasedVelocityICFF(FieldFunction globalLabVelocityICFF){
      String ffName = rotorName+" Lab Based Vel-IC";
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName(rotorName + "_lab_vel_ff");
      
      // Transform Rotors into Lab Basis
      // Get all the transformation components
      String rotorFFN = getLocalVelocityICFF().getFunctionName();
      String rotorX = "($${" + rotorFFN + "}[0])";
      String rotorY = "($${" + rotorFFN + "}[1])";
      String rotorZ = "($${" + rotorFFN + "}[2])";
      double[][] tMat = SimTool.multiply3x3Matrices(body2Lab, rotor2Body);
      
      String xComponent = "" + tMat[0][0] + "*" + rotorX + "+" + tMat[0][1] + "*" + rotorY + "+" + tMat[0][2] + "*" + rotorZ;
      String yComponent = "" + tMat[1][0] + "*" + rotorX + "+" + tMat[1][1] + "*" + rotorY + "+" + tMat[1][2] + "*" + rotorZ;
      String zComponent = "" + tMat[2][0] + "*" + rotorX + "+" + tMat[2][1] + "*" + rotorY + "+" + tMat[2][2] + "*" + rotorZ;      

      // Compute labVelocityICFF
      double[][] inlet2Body = SimTool.getBasisSet(SimTool.getNestedCoordinate(bodyCsys, globalNames.getInletCsysName()));
      double[][] inlet2LabT = SimTool.multiply3x3Matrices(body2Lab, inlet2Body);
      String wallDecayFN = "(-1.0*${"+getWallDecayFunctionFF().getFunctionName()+"}+1.0)";
      String globalVelFFN = globalLabVelocityICFF.getFunctionName();
      String globalVx = "$${" + globalVelFFN + "}[0]";
      String globalVy = "$${" + globalVelFFN + "}[1]";
      String globalVz = "$${" + globalVelFFN + "}[2]";
      String globalLabVx = "((" + inlet2LabT[0][0] + "*" + globalVx + "+ " + inlet2LabT[0][1] + "*" + globalVy + "+ " + inlet2LabT[0][2] + "*" + globalVz + ")*" + wallDecayFN + ")";
      String globalLabVy = "((" + inlet2LabT[1][0] + "*" + globalVx + "+ " + inlet2LabT[1][1] + "*" + globalVy + "+ " + inlet2LabT[1][2] + "*" + globalVz + ")*" + wallDecayFN + ")";
      String globalLabVz = "((" + inlet2LabT[2][0] + "*" + globalVx + "+ " + inlet2LabT[2][1] + "*" + globalVy + "+ " + inlet2LabT[2][2] + "*" + globalVz + ")*" + wallDecayFN + ")";

      uFF.setDefinition("[" + xComponent + "+" + globalLabVx + "," + yComponent + "+" + globalLabVy  + "," + zComponent + "+" + globalLabVz + "]");
      
      
      return uFF;
      
    }

    private FieldFunction getWallDecayFunctionFF(){
      String ffName = rotorName+" Wall Decay - IC";
      UserFieldFunction uFF;
      try{
        uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
        uFF = simu.getFieldFunctionManager().createFieldFunction();
        uFF.setPresentationName(ffName);
      }
      uFF.setFunctionName(rotorName+"_walldecay_ic");
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setDefinition("(1.0 + 1.0 * tanh(-${WallDistance}/ ("+bladeRadius+"*0.1*0.02) ) )");
      return uFF;
    }
    
    private FieldFunction getLocalVelocityICFF(){
      String ffName = rotorName+" Local Csys Vel-IC";
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName(rotorName + "_local_vel_ff");

      String wallDecayFN = getWallDecayFunctionFF().getFunctionName();
      String thetaCsysFQDN = bodyCsysName + "." + rotorCsysName + "." 
             +thetaCsys.getPresentationName();
      String xComponent = "-" + rotorSpeed + "*${" + wallDecayFN + "}*"
             + "$$Centroid(\"" + thetaCsysFQDN + "\")[0]*"
             + "sin($$Centroid(\"" + thetaCsysFQDN + "\")[1])";
      String yComponent = "" + rotorSpeed + "*${" + wallDecayFN + "}*"
             + "$$Centroid(\"" + thetaCsysFQDN + "\")[0]*"
             + "cos($$Centroid(\"" + thetaCsysFQDN + "\")[1])";
      String zComponent = "0.0";
      uFF.setDefinition("[" + xComponent + "," + yComponent + "," 
                            + zComponent + "]");

      return uFF;
    }

    private void setRegionBasedIC(){
        parentRegion.getConditions()
        .get(InitialConditionOption.class)
        .setSelected(InitialConditionOption.Type.REGION);
    }

    private void setLocalRotorVelocityIC(FieldFunction labVelocityICFF){
        VelocityProfile velocityProfile = 
        parentRegion.get(RegionInitialConditionManager.class)
                .get(VelocityProfile.class);
        velocityProfile.setMethod(FunctionVectorProfileMethod.class);
        velocityProfile.getMethod(FunctionVectorProfileMethod.class)
                .setFieldFunction(getLabBasedVelocityICFF(labVelocityICFF));
        velocityProfile.setCoordinateSystem(labCsys);
    }
    
    private void setLocalRotorPressureIC(FieldFunction labVelocityICFF){
      InitialPressureProfile initialPressureProfile = 
        parentRegion.get(RegionInitialConditionManager.class)
              .get(InitialPressureProfile.class);
      initialPressureProfile.setMethod(FunctionScalarProfileMethod.class);
      
      getLocalPressureICFF(labVelocityICFF);
      
      initialPressureProfile.getMethod(FunctionScalarProfileMethod.class)
              .setFieldFunction(getLocalPressureICFF(labVelocityICFF));
    }
    private FieldFunction getLocalPressureICFF(FieldFunction globalLabVelocityICFF){
      double idealR   = 8.3144598;  // [kJ/kmol.K]  ideal gas constant
      double molAir   = 0.0289645;  // [kg/mol] molar mass of air
      double airGamma = 1.4;        // [#] ratio of specific heats
      double airR = (idealR/molAir); // [kJ/kg.K] specific gas constant
  
      String labPressICFFN = "${pressure_ic}";
      
      String ffName = rotorName+" Pressure-IC";
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName(rotorName + "_pressure_ic");

      String velFFName = getLabBasedVelocityICFF(globalLabVelocityICFF).getFunctionName();
      uFF.setDefinition(""+labPressICFFN+"+0.5*"+refRho+"*pow(mag($${"
              +velFFName+"}),2)");

      return uFF;
    }
    

    private void setLocalRotorTemperatureIC(FieldFunction globalLabVelocityICFF){
      StaticTemperatureProfile staticTemperatureProfile = 
        parentRegion.get(RegionInitialConditionManager.class)
              .get(StaticTemperatureProfile.class);
      staticTemperatureProfile.setMethod(FunctionScalarProfileMethod.class);
      staticTemperatureProfile.getMethod(FunctionScalarProfileMethod.class)
              .setFieldFunction(getLocalRotorTemperatureICFF(globalLabVelocityICFF));
    }
    private FieldFunction getLocalRotorTemperatureICFF(FieldFunction globalLabVelocityICFF){
      double idealR   = 8.3144598;  // [kJ/kmol.K]  ideal gas constant
      double molAir   = 0.0289645;  // [kg/mol] molar mass of air
      double airGamma = 1.4;        // [#] ratio of specific heats
      double airR = (idealR/molAir); // [kJ/kg.K] specific gas constant


      double labTemp = parentRotatingDomain.getRegion().getPhysicsContinuum()
              .getInitialConditions().get(StaticTemperatureProfile.class)
              .getMethod(ConstantScalarProfileMethod.class).getQuantity()
              .getSIValue();

      double soundSpeed = Math.sqrt(airGamma*airR*labTemp);

      String ffName = rotorName+" Temperature-IC";
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) simu.getFieldFunctionManager().getObject(ffName));
      }catch(NeoException e){
          uFF = simu.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName(ffName);
      uFF.setFunctionName(rotorName + "_temperature_ic");

      String velFFName = getLabBasedVelocityICFF(globalLabVelocityICFF).getFunctionName();
      uFF.setDefinition(""+labTemp+"/(1.0+"+(1-airGamma)/2.
              +"*pow(mag($${"+velFFName+"})/"+soundSpeed+",2))");

      return uFF;
    }
    
    public void initLocalRotorICs(FieldFunction globalLabVelocityICFF){
      setRegionBasedIC();
      setLocalRotorVelocityIC(globalLabVelocityICFF);
      setLocalRotorPressureIC(globalLabVelocityICFF);
      setLocalRotorTemperatureIC(globalLabVelocityICFF);
    }
    
    //
    // REPORTS
    public void makeReports(){
        //Standard Reports
        generateForceAndMomentReports("Rotor",rotorCsys);
        generateForceAndMomentReports("Body",bodyCsys);
        generateHelicopterCoefficients();
        generatePropellerCoefficients();
        generateYPlusReports();
        
        //Groupings
        //ReportTool.groupReports(simu,rotorName,rotorReports);
        //ReportTool.groupReports(simu,rotorName,bodyReports);
        //ReportTool.groupReports(simu,rotorName,wallYplusReports);
        //ReportTool.groupReports(simu,rotorName,helicopterReports);
        //ReportTool.groupReports(simu,rotorName,propellerReports);
    }
    public void makeIterationMonitors(Simulation tmpSim, int plotSamplesLimit,int freq, int startIT){
        //Make monitors
        generateIterationMonitors(rotorReports,rotorIterMonitors,
                plotSamplesLimit,freq,startIT);
        generateIterationMonitors(bodyReports,bodyIterMonitors,
                plotSamplesLimit,freq,startIT);
        generateIterationMonitors(wallYplusReports,wallYplusIterMonitors,
                plotSamplesLimit,freq,startIT);
        generateIterationMonitors(helicopterReports,helicopterIterMonitors,
                plotSamplesLimit,freq,startIT);
        generateIterationMonitors(propellerReports,propellerIterMonitors,
                plotSamplesLimit,freq,startIT);
        
        //group monitors
        //MonitorTool.groupMonitors(tmpSim,rotorName,rotorIterMonitors);
        //MonitorTool.groupMonitors(tmpSim,rotorName,bodyIterMonitors);
        //MonitorTool.groupMonitors(tmpSim,rotorName,wallYplusIterMonitors);
        //MonitorTool.groupMonitors(tmpSim,rotorName,helicopterIterMonitors);
        //MonitorTool.groupMonitors(tmpSim,rotorName,propellerIterMonitors);
    }
    public void makeUnsteadyMonitors(int plotSamplesLimit,int freq,int startTS){
        //Make monitors
        generateUnsteadyMonitors(rotorReports,rotorUnsMonitors,
                plotSamplesLimit,freq,startTS);
        generateUnsteadyMonitors(bodyReports,bodyUnsMonitors,
                plotSamplesLimit,freq,startTS);
        generateUnsteadyMonitors(wallYplusReports,wallYplusUnsMonitors,
                plotSamplesLimit,freq,startTS);
        generateUnsteadyMonitors(helicopterReports,helicopterUnsMonitors,
                plotSamplesLimit,freq,startTS);
        generateUnsteadyMonitors(propellerReports,propellerUnsMonitors,
                plotSamplesLimit,freq,startTS);

        //group monitors
        //MonitorTool.groupMonitors(simu,rotorName,rotorUnsMonitors);
        //MonitorTool.groupMonitors(simu,rotorName,bodyUnsMonitors);
        //MonitorTool.groupMonitors(simu,rotorName,wallYplusUnsMonitors);
        //MonitorTool.groupMonitors(simu,rotorName,helicopterUnsMonitors);
        //MonitorTool.groupMonitors(simu,rotorName,propellerUnsMonitors);
    }
    //
    // BODY REPORTS
    public double getBodyFX(){
        return getMeanReportVal(((ForceReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - FX")),itPostFix,statSamples);
    }
    public double getBodyFY(){
        return getMeanReportVal(((ForceReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - FY")),itPostFix,statSamples);
    }
    public double getBodyFZ(){
        return getMeanReportVal(((ForceReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - FZ")),itPostFix,statSamples);
    }
    public double getBodyMX(){
        return getMeanReportVal(((MomentReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - MX")),itPostFix,statSamples);
    }
    public double getBodyMY(){
        return getMeanReportVal(((MomentReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - MY")),itPostFix,statSamples);
    }
    public double getBodyMZ(){
        return getMeanReportVal(((MomentReport) simu.getReportManager()
                .getReport("Body "+rotorName+" - MZ")),itPostFix,statSamples);
    }
    //
    // REPORTS
    public ArrayList<Report> getRotorReports(){
        return rotorReports;
    }
    public ArrayList<Report> getBodyReports(){
        return bodyReports;
    }
    //
    // MONITORS
    public Monitor getThrustMonitor(boolean isUns){
        String appEnd = "";
        if(isUns){
            appEnd = unsPostFix;
        }else{
            appEnd = itPostFix;
        }
        if(!isBEMRotor){
            return simu.getMonitorManager().getMonitor("Rotor "+rotorName+" - FZ"+appEnd);
        }else{
            return null;
        }
    }
    public Monitor getTorqueMonitor(boolean isUns){
        String appEnd = "";
        if(isUns){
            appEnd = unsPostFix;
        }else{
            appEnd = itPostFix;
        }
        if(!isBEMRotor){
            return simu.getMonitorManager().getMonitor("Rotor "+rotorName+" - MZ"+appEnd);
        }else{
            return null;
        }
    }
    
    // TOOLS
    // General
    public double getMeanReportVal(Report tmpRep, String itPostFix, int nSamples){
        String repName=tmpRep.getPresentationName();
        String monName=repName+itPostFix;
        double meanVal= MonitorTool.getLastMonitorSamplesMean(simu,monName,nSamples);
        return meanVal;
    }
    public double getMeanReportStddevVal(Report tmpRep, String itPostFix, int nSamples,double meanVal){
        String repName=tmpRep.getPresentationName();
        String monName=repName+itPostFix;
        double stdDev=MonitorTool.getLastMonitorSamplesStDev(simu,monName,nSamples,meanVal);
        return stdDev;
    }
    // steady
    public double getMeanItMoment(String preFix, int coefDir, int numSamp){
        String postFix = "";
        String appEnd = globalNames.getItPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - MX";
            }else if(coefDir==1){
                postFix=" - MY";
            }else if(coefDir==2){
                postFix=" - MZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cMRX";
            }else if(coefDir==1){
                postFix=" - cMRY";
            }else if(coefDir==2){
                postFix=" - cMQ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kQ1";
            }else if(coefDir==1){
                postFix=" - kQ2";
            }else if(coefDir==2){
                postFix=" - kQ3";
            }else{
                postFix=" - "+coefDir;
            }
        }

        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesMean(simu,monName,numSamp);
    }
    public double getStdDevItMoment(String preFix,int coefDir, int numSamples,double meanVal){
        String postFix = globalNames.getItPostFix();
        String appEnd = globalNames.getItPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - MX";
            }else if(coefDir==1){
                postFix=" - MY";
            }else if(coefDir==2){
                postFix=" - MZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cMRX";
            }else if(coefDir==1){
                postFix=" - cMRY";
            }else if(coefDir==2){
                postFix=" - cMQ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kQ1";
            }else if(coefDir==1){
                postFix=" - kQ2";
            }else if(coefDir==2){
                postFix=" - kQ3";
            }else{
                postFix=" - "+coefDir;
            }
        }
        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesStDev(simu,monName,numSamples,meanVal);
    }
    public double getMeanItForce(String preFix, int coefDir, int numSamp){
        String postFix = "";
        String appEnd = itPostFix;
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - FX";
            }else if(coefDir==1){
                postFix=" - FY";
            }else if(coefDir==2){
                postFix=" - FZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cH";
            }else if(coefDir==1){
                postFix=" - cRY";
            }else if(coefDir==2){
                postFix=" - cT";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kT1";
            }else if(coefDir==1){
                postFix=" - kT2";
            }else if(coefDir==2){
                postFix=" - kT3";
            }else{
                postFix=" - "+coefDir;
            }
        }

        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesMean(simu,monName,numSamp);
    }
    public double getStdDevItForce(String preFix,int coefDir, int numSamples,double meanVal){
        String postFix = globalNames.getItPostFix();
        String appEnd = globalNames.getItPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - FX";
            }else if(coefDir==1){
                postFix=" - FY";
            }else if(coefDir==2){
                postFix=" - FZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cH";
            }else if(coefDir==1){
                postFix=" - cRY";
            }else if(coefDir==2){
                postFix=" - cT";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kT1";
            }else if(coefDir==1){
                postFix=" - kT2";
            }else if(coefDir==2){
                postFix=" - kT3";
            }else{
                postFix=" - "+coefDir;
            }
        }
        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesStDev(simu,monName,numSamples,meanVal);
    }
    // unsteady
    public double getMeanUnsMoment(String preFix, int coefDir, int numSamp){
        String postFix = "";
        String appEnd = globalNames.getUnsPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - MX";
            }else if(coefDir==1){
                postFix=" - MY";
            }else if(coefDir==2){
                postFix=" - MZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cMRX";
            }else if(coefDir==1){
                postFix=" - cMRY";
            }else if(coefDir==2){
                postFix=" - cMQ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kQ1";
            }else if(coefDir==1){
                postFix=" - kQ2";
            }else if(coefDir==2){
                postFix=" - kQ3";
            }else{
                postFix=" - "+coefDir;
            }
        }

        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesMean(simu,monName,numSamp);
    }
    public double getStdDevUnsMoment(String preFix,int coefDir, int numSamples,double meanVal){
        String postFix = "";
        String appEnd = globalNames.getUnsPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - MX";
            }else if(coefDir==1){
                postFix=" - MY";
            }else if(coefDir==2){
                postFix=" - MZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cMRX";
            }else if(coefDir==1){
                postFix=" - cMRY";
            }else if(coefDir==2){
                postFix=" - cMQ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kQ1";
            }else if(coefDir==1){
                postFix=" - kQ2";
            }else if(coefDir==2){
                postFix=" - kQ3";
            }else{
                postFix=" - "+coefDir;
            }
        }
        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesStDev(simu,monName,numSamples,meanVal);
    }
    public double getMeanUnsForce(String preFix, int coefDir, int numSamp){
        String postFix = "";
        String appEnd = globalNames.getUnsPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - FX";
            }else if(coefDir==1){
                postFix=" - FY";
            }else if(coefDir==2){
                postFix=" - FZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cH";
            }else if(coefDir==1){
                postFix=" - cRY";
            }else if(coefDir==2){
                postFix=" - cT";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kT1";
            }else if(coefDir==1){
                postFix=" - kT2";
            }else if(coefDir==2){
                postFix=" - kT3";
            }else{
                postFix=" - "+coefDir;
            }
        }

        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesMean(simu,monName,numSamp);
    }
    public double getStdDevUnsForce(String preFix,int coefDir, int numSamples,double meanVal){
        String postFix = "";
        String appEnd = globalNames.getUnsPostFix();
        if(preFix.equals("Rotor")||preFix.equals("Body")){
            if(coefDir==0){
                postFix=" - FX";
            }else if(coefDir==1){
                postFix=" - FY";
            }else if(coefDir==2){
                postFix=" - FZ";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Heli")){
            if(coefDir==0){
                postFix=" - cH";
            }else if(coefDir==1){
                postFix=" - cRY";
            }else if(coefDir==2){
                postFix=" - cT";
            }else{
                postFix=" - "+coefDir;
            }
        }else if(preFix.equals("Prop")){
            if(coefDir==0){
                postFix=" - kT1";
            }else if(coefDir==1){
                postFix=" - kT2";
            }else if(coefDir==2){
                postFix=" - kT3";
            }else{
                postFix=" - "+coefDir;
            }
        }
        String monName=preFix+" "+rotorName+postFix+appEnd;
        return MonitorTool.getLastMonitorSamplesStDev(simu,monName,numSamples,meanVal);
    }
    //
    // DATA OUTPUT
    public String csvData(boolean isUnsteady){
        String tmpStr="";
        double meanVal;
        double stddevVal;
        
        //Name
        tmpStr=tmpStr+rotorName.substring(4);
        //Physics model
        if(parentRegion==null){
            tmpStr=tmpStr+","+"BEM";
        }else{
            tmpStr=tmpStr+","+getPhysicalRotorModelType();
        }
        //nblades, blade radius, blade swept area, angular speed
        tmpStr=tmpStr+","+nBlades+","+bladeRadius+","+bladeArea+","+rotorSpeed;
        
        //coordinate system stuff
        double[] origin = rotorCsys.getOriginVector().toDoubleArray();
        double[] xBasis = rotorCsys.getBasis0().toDoubleArray();
        double[] yBasis = rotorCsys.getBasis1().toDoubleArray();
        double[] zBasis = rotorCsys.getBasis2().toDoubleArray();
        String originStr = "{"+origin[0]+";"+origin[1]+";"+origin[2]+"}";
        String xBasisStr = "{"+xBasis[0]+";"+xBasis[1]+";"+xBasis[2]+"}";
        String yBasisStr = "{"+yBasis[0]+";"+yBasis[1]+";"+yBasis[2]+"}";
        String zBasisStr = "{"+zBasis[0]+";"+zBasis[1]+";"+zBasis[2]+"}";
        tmpStr=tmpStr+","+originStr+","+xBasisStr+","+yBasisStr+","+zBasisStr;
        
        // Body - Forces and Moments data
        ArrayList<String> repTypeList = new ArrayList();
        repTypeList.add("Body");
        repTypeList.add("Rotor");
        repTypeList.add("Prop");
        repTypeList.add("Heli");

        for(int j=0; j<repTypeList.size(); j++){
          String repType = repTypeList.get(j);
          for(int i=0; i <= 2; i++){
            if(!isUnsteady){
              meanVal=getMeanItForce(repType, i, statSamples);
              stddevVal=getStdDevItForce(repType, i, statSamples,meanVal);
              tmpStr=tmpStr+","+meanVal+","+stddevVal;
            }else{
              meanVal=getMeanUnsForce(repType,i,statSamples);
              stddevVal=getStdDevUnsForce(repType,i,statSamples,meanVal);
              tmpStr=tmpStr+","+meanVal+","+stddevVal;
            }
          }
          for(int i=0;i<=2;i++){
            if(!isUnsteady){
              meanVal=getMeanItMoment(repType,i,statSamples);
              stddevVal=getStdDevItMoment(repType,i,statSamples,meanVal);
              tmpStr=tmpStr+","+meanVal+","+stddevVal;
            }else{
              meanVal=getMeanUnsMoment(repType,i,statSamples);
              stddevVal=getStdDevUnsMoment(repType,i,statSamples,meanVal);
              tmpStr=tmpStr+","+meanVal+","+stddevVal;
            }
          }
        }
        //Meshing data
        tmpStr=tmpStr+","+getCustomTargetSurfaceSize();
        tmpStr=tmpStr+","+getCustomMinSurfaceSize();
        tmpStr=tmpStr+","+getNPtsOnCircle();
        tmpStr=tmpStr+","+getCustomNumPrismLayers();
        tmpStr=tmpStr+","+getCustomPrismAbsThickness();
        tmpStr=tmpStr+","+getCustomFirstCellThickness();

        //Wall y+ data
        for(Report tmpRep:wallYplusReports){
            meanVal=getMeanReportVal(tmpRep,itPostFix,statSamples);
            stddevVal=getMeanReportStddevVal(tmpRep,itPostFix,statSamples,meanVal);
            tmpStr=tmpStr+","+meanVal+","+stddevVal;
        }
        return tmpStr;
    }

    //
    // PLOTS
    public void initRotorPlots(boolean isUnsteady){
        // Figure out what kind of plot it needs to have
        String appEnd ="";
        if(isUnsteady){
            appEnd = unsPostFix;
        }else{
            appEnd = itPostFix;
        }
        //THRUST
        String monPreFix = "Rotor ";
        String monPostFix = " - FZ";
        String monName=monPreFix+rotorName+monPostFix+appEnd;
        //plot
        String plotInfo = rotorName.substring(4)+" Thrust"+appEnd;
        MonitorPlot tmpPlot=PlotTool.getMonitorPlot(simu,plotInfo,plotInfo);
        MonitorDataSet tmpMonData=PlotTool.addDataSet(simu,tmpPlot, monName, rotorName.substring(4));
        PlotTool.setDataLines(tmpMonData, "Solid", 2, PlotTool.getColor("red"));
        allRotorPlots.add(tmpPlot);
        //
        //TORQUE
        monPreFix = "Rotor ";
        monPostFix = " - MZ";
        monName=monPreFix+rotorName+monPostFix+appEnd;
        //plot
        plotInfo = rotorName.substring(4)+" Torque"+appEnd;
        tmpPlot=PlotTool.getMonitorPlot(simu,plotInfo,plotInfo);
        tmpMonData=PlotTool.addDataSet(simu,tmpPlot, monName, rotorName.substring(4));
        PlotTool.setDataLines(tmpMonData, "Solid", 2, PlotTool.getColor("red"));
        allRotorPlots.add(tmpPlot);
        
    }
    public ArrayList<StarPlot> getAllRotorPlots(){
        return allRotorPlots;
    }
    public Collection<StarPlot> getAllConsumerPlots(){
        return allRotorPlots;
    }
    //
    // SCENES
    private void initSkinFrictionCoefficientScenes(double[] rotorAxisCameraPos){
        // Viewpoint
        double[] negRotorAxisCameraPos=
            {0.,0.,-rotorAxisCameraPos[2]};
        Scene tmpScene;
        String tmpSceneName;
        String sceneTitle;
        Annotation sceneTitleAnn;
        
        // part displayer stuff
        String nonRotorGeomName = "Non Rotor Parts";
        PartDisplayer tmpPD;   // part displayer
        // scalar displayer stuff
        ScalarDisplayer tmpSD; // scalar displayer
        getZNormalInterfacePlane();
        //
        //========================================================
        //  Skin friction coefficient (front, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cf, Front-Side, Non-Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cf",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient",
                "Off","Off",0.0,0.005,proxyRep);
        SceneTool.setScalarColormap(tmpSD,"high-contrast",64,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.removeDefaultLogo(tmpScene);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Skin friction coefficient (back, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cf, Back-Side, Non-Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cf",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",
                0.0,0.005,proxyRep);
        SceneTool.setScalarColormap(tmpSD,"high-contrast",64,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Skin friction coefficient (front, rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cf, Front-Side, Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, motionRotorCsys, 1.0, 1);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cf",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",
                0.0,0.005,proxyRep);
        SceneTool.setScalarColormap(tmpSD,"high-contrast",64,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Skin friction coefficient (back, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cf, Back-Side, Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, motionRotorCsys, 1.0, 1);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cf",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"SkinFrictionCoefficient","Off","Off",
                0.0,0.005,proxyRep);
        SceneTool.setScalarColormap(tmpSD,"high-contrast",64,6,true,"Left Side").getLegend().setLabelFormat("%-#5.3f");
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
    }
    private void initPressureCoefficientScenes(double[] rotorAxisCameraPos){
        // Viewpoint
        double[] negRotorAxisCameraPos=
            {0.,0.,-rotorAxisCameraPos[2]};
        Scene tmpScene;
        String tmpSceneName;
        String sceneTitle;
        Annotation sceneTitleAnn;
        
        // part displayer stuff
        String nonRotorGeomName = "Non Rotor Parts";
        PartDisplayer tmpPD;   // part displayer
        // scalar displayer stuff
        ScalarDisplayer tmpSD; // scalar displayer
        getZNormalInterfacePlane();
        //
        //========================================================
        //  Pressure coefficient (front, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cp, Front-Side, Non-Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cp",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient",
                "Off","Off",-3.0,1.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getBlueDkGnRedString(simu),64,6,true,"Left Side");
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.removeDefaultLogo(tmpScene);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Pressure coefficient (back, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cp, Back-Side, Non-Rotating Frame";
        // scene setup
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cp",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient",
                "Off","Off",-3.0,1.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getBlueDkGnRedString(simu),64,6,true,"Left Side");
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.removeDefaultLogo(tmpScene);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Pressure coefficient (front, rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cp, Front-Side, Rotating Frame";
        // scene setup
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cp",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient",
                "Off","Off",-3.0,1.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getBlueDkGnRedString(simu),64,6,true,"Left Side");
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.removeDefaultLogo(tmpScene);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
        //
        //========================================================
        //  Pressure coefficient (back, non-rotating frame)
        //========================================================
        tmpSceneName=rotorName+" - Cp, Back-Side, Rotating Frame";
        // scene setup
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        sceneTitle = tmpSceneName.substring(4);
        sceneTitleAnn=SceneTool.getTextAnnotation(simu,sceneTitle,sceneTitle,0.035,new double[]{0.025, 0.94,0.},true);
        sceneTitleAnn.setBackground(false);
        SceneTool.addAnnotation(tmpScene,sceneTitleAnn);
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setAxesVisible(true);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"cp",geomSurfs);
        tmpSD.setFillMode(ScalarFillMode.NODE_FILLED);
        SceneTool.setScalarDisplayerField(tmpSD,"PressureCoefficient",
                "Off","Off",-3.0,1.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getBlueDkGnRedString(simu),64,6,true,"Left Side");
        SceneTool.setSceneView(tmpScene, zeroOrigin, negRotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 1);
        SceneTool.removeDefaultLogo(tmpScene);
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allConsumerScenes.add(tmpScene);
        tmpScene.close(true);
    }
    private void initWallYPlusScenes(double[] rotorAxisCameraPos){
        // Viewpoint
        double[] negRotorAxisCameraPos=
            {0.,0.,-rotorAxisCameraPos[2]};
        Scene tmpScene;
        String tmpSceneName;
        // part displayer stuff
        String nonRotorGeomName = "Non Rotor Parts";
        PartDisplayer tmpPD;   // part displayer
        // scalar displayer stuff
        ScalarDisplayer tmpSD; // scalar displayer
        getZNormalInterfacePlane();
        //
        //
        //
        tmpSceneName=rotorName+" - Wall Y+ Non Rotating Frame";
        // scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        // geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"y plus",geomSurfs);
        SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",
                0.0,300.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getTotalRangeWallYPlusString(simu),32,6,false,"Left Side");
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, rotorCsys, 1.0, 0);
        tmpScene.setView(new double[]{-1.0,0.0,0.0}, new double[]{0.0,0.0,-1.0}, zeroOrigin);
        tmpScene.resetCamera();
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allCFDScenes.add(tmpScene);
        tmpScene.close(true);
        //
        // Wall y+ rotating frame
        tmpSceneName=rotorName+" - Wall Y+ Rotating Frame";
        //  scene setup
        tmpScene=SceneTool.getScene(simu,tmpSceneName);
        SceneTool.setSceneView(tmpScene, zeroOrigin, rotorAxisCameraPos,
                xVector, motionRotorCsys, 1.0, 0);
        tmpScene.setView(new double[]{-1.0,0.0,0.0}, new double[]{0.0,0.0,-1.0}, zeroOrigin);
        tmpScene.resetCamera();
        //  geometry
        tmpPD = SceneTool.getPartDisplayer(tmpScene,nonRotorGeomName,
                extraSceneBodyParts,proxyRep);
        SceneTool.setPartDisplayerField(tmpPD,false,false,false,true,0,0.6);
        //  scalar values
        tmpSD=SceneTool.getScalarDisplayer(tmpScene,"y plus",geomSurfs);
        SceneTool.setScalarDisplayerField(tmpSD,"WallYplus","Off","Off",
                0.0,300.0,proxyRep);
        SceneTool.setScalarColormap(tmpSD,ColorMapTool.getTotalRangeWallYPlusString(simu),11,6,true,"Left Side");
        SceneTool.setVectorDisp(tmpScene, rotorName+ "vectDisplayer", Collections.singleton(zNormalInterfacePlane), rotorCsys, proxyRep);
        allCFDScenes.add(tmpScene);
        tmpScene.close(true);

    }
    public void initRotorScenes(){
        // Standard scalar scenes for any rigid body rotor
        Scene tmpScene;
        String tmpSceneName;
        // scalar displayer stuff
        ScalarDisplayer tmpSD; // scalar displayer
        //
        NumberFormat formattedDouble = new DecimalFormat("0.00");

        String annRotorSpeed=formattedDouble.format(rotorSpeed);
        String annBPF=formattedDouble.format(bladePassFreq);
        String annRad=formattedDouble.format(bladeRadius);
        String annTipMach=formattedDouble.format(tipMach);
        //
        simu.println("RTR MSG: Setting up Rotor Scenes");
        String annRotorName = rotorName.substring(4);
        String tmpStr;
        if(parentRegion==null){
            tmpStr="BEM";
        }else{
            tmpStr=getPhysicalRotorModelType();
        }
        String rotorSpecificInfoStr=
                 "Rotor Info: \n "+
                 "\n\u2022 Model:    "+tmpStr+"\n"
                +"\n\u2022 Speed:    "+annRotorSpeed+" rad/s\n"
                +"\n\u2022 BPF:        "+annBPF+" Hz\n"
                +"\n\u2022 Radius:    "+annRad+" m\n"
                +"\n\u2022 Tip Mach: "+annTipMach+"\n";

        Annotation rtrInfo = SceneTool
            .getTextAnnotation(simu,annRotorName+" Info",rotorSpecificInfoStr,0.35,
                    new double[]{0.7, 0.55, 0.0},true);
        rtrInfo.setBackground(false);
        
        if(!isBEMRotor){
            double viewAngle = Math.atan(15.0*Math.PI/180.0);
            double[] rotorAxisCameraPos={0.,0.,2.0*bladeRadius*viewAngle};
            
            //=========================
            //  DESIGNER SCENES
            //=========================
            // Skin friction coefficient:
            //   front/back, rotating & non-rotating frames
            simu.println("RTR MSG: Init Cf");
            initSkinFrictionCoefficientScenes(rotorAxisCameraPos);
            
            // Pressure coefficient:
            //   front/back, rotating & non-rotating frames
            simu.println("RTR MSG: Init Cp");
            initPressureCoefficientScenes(rotorAxisCameraPos);
            
            // Volume renderings of the wake
            
            //=========================
            // CFD Engineer Scenes
            //=========================
            //
            // Wall y+ non-rotating frame
            initWallYPlusScenes(rotorAxisCameraPos);
        }
        for(Scene tmp:getAllScenes()){
            SceneTool.addAnnotation(tmp, rtrInfo);
        }
        //SceneTool.groupScenes(simu,rotorName.substring(4), getAllScenes());
        
    }
    public ArrayList<Scene> getAllScenes(){
        ArrayList<Scene> allScenes = new ArrayList();
        allScenes.addAll(allConsumerScenes);
        allScenes.addAll(allConsumerVRScenes);
        allScenes.addAll(allCFDScenes);
        return allScenes;
    }
    public Collection<Scene> getAllConsumerScenes(){
        return allConsumerScenes;
    }
    public Collection<Scene> getAllConsumerVRScenes(){
        return allConsumerVRScenes;
    }

    //
    // TOOLS
    public CartesianCoordinateSystem getRotorCoordSys(){
        return rotorCsys;
    }
    //
    //======================
    //   PRIVATE METHODS
    //======================
    // Reports
    private void generateForceAndMomentReports(String preFix,CoordinateSystem relCsys){
        /* method generateForceAndMomentReports 
        
        */
        // Figure out what kind of plot it needs to have
        String rootName=preFix +" "+ rotorName;
        //Add to correct list
        ArrayList<Report> tmpRepList=new ArrayList();
        if(preFix.equals("Rotor")){
            tmpRepList=rotorReports;
        }else if(preFix.equals("Body")){
            tmpRepList=bodyReports;
        }
        //Instantiate
        ForceReport  fX; ForceReport  fY; ForceReport  fZ;
        MomentReport mX; MomentReport mY; MomentReport mZ;
        //Forces        
        fX=ReportTool.forceReport(simu,rootName+" - FX",relCsys,xVector,
                geomSurfs,proxyRep);
        fY=ReportTool.forceReport(simu,rootName+" - FY",relCsys,yVector,
                geomSurfs,proxyRep);
        fZ=ReportTool.forceReport(simu,rootName+" - FZ",relCsys,zVector,
                geomSurfs,proxyRep);
        if(!tmpRepList.contains(fX)) tmpRepList.add(fX);
        if(!tmpRepList.contains(fY)) tmpRepList.add(fY);
        if(!tmpRepList.contains(fZ)) tmpRepList.add(fZ);
        //
        //Moments
        mX=ReportTool.momentReport(simu,rootName+" - MX",relCsys,zeroOrigin,xVector,
                geomSurfs,proxyRep);
        mY=ReportTool.momentReport(simu,rootName+" - MY",relCsys,zeroOrigin,yVector,
                geomSurfs,proxyRep);
        mZ=ReportTool.momentReport(simu,rootName+" - MZ",relCsys,zeroOrigin,zVector,
                geomSurfs,proxyRep);
        if(!tmpRepList.contains(mX)) tmpRepList.add(mX);
        if(!tmpRepList.contains(mY)) tmpRepList.add(mY);
        if(!tmpRepList.contains(mZ)) tmpRepList.add(mZ);
    }
    private void generatePropellerCoefficients(){
        String rootName="Prop " +rotorName;
        //Add to correct list
        ArrayList<Report> tmpRepList=propellerReports;
        //Force & Moment  Coefficients
        ForceCoefficientReport kT1; MomentCoefficientReport kQ1;
        ForceCoefficientReport kT2; MomentCoefficientReport kQ2;
        ForceCoefficientReport kT3; MomentCoefficientReport kQ3;
        //
        // Propeller Force Coefficients
        //  Defn: F / (rho * n^2 * D^4)
        //  CCM+: F / (1/2 * rho * V^2 * A)
        //  Appr: Use sqrt(2) * n^1 D^2 where velocity would be and A=1.0
        //double n2D4  =Math.pow(rotorSpeed/(2*Math.PI),2)*Math.pow(bladeRadius*2.0,4);
        double n1 = Math.abs(rotorSpeed/(2*Math.PI));
        double n1D2  = Math.sqrt(2)*n1*Math.pow(bladeRadius*2.0,2);

        kT1=ReportTool.forceCoefReport(simu,rootName+" - kT1",rotorCsys, xVector,
                refRho,n1D2, 1.0,geomSurfs,proxyRep);
        kT2=ReportTool.forceCoefReport(simu,rootName+" - kT2",rotorCsys, yVector,
                refRho,n1D2, 1.0,geomSurfs,proxyRep);
        kT3=ReportTool.forceCoefReport(simu,rootName+" - kT3",rotorCsys, zVector,
                refRho,n1D2, 1.0,geomSurfs,proxyRep);
        if(!tmpRepList.contains(kT1)) tmpRepList.add(kT1);
        if(!tmpRepList.contains(kT2)) tmpRepList.add(kT2);
        if(!tmpRepList.contains(kT3)) tmpRepList.add(kT3);

        // Propeller Moment Coefficients
        //  Note: We use n^2 D^5 where velocity would be and A=1.0, r*=1.0
        kQ1=ReportTool.momentCoefReport(simu,rootName+" - kQ1",rotorCsys,zeroOrigin,
                xVector,refRho,n1D2, 1.0,bladeRadius*2.0, geomSurfs, proxyRep);
        kQ2=ReportTool.momentCoefReport(simu,rootName+" - kQ2",rotorCsys,zeroOrigin,
                yVector,refRho,n1D2, 1.0,bladeRadius*2.0, geomSurfs, proxyRep);
        kQ3=ReportTool.momentCoefReport(simu,rootName+" - kQ3",rotorCsys,zeroOrigin,
                zVector,refRho,n1D2, 1.0,bladeRadius*2.0, geomSurfs, proxyRep);
        if(!tmpRepList.contains(kQ1)) tmpRepList.add(kQ1);
        if(!tmpRepList.contains(kQ2)) tmpRepList.add(kQ2);
        if(!tmpRepList.contains(kQ3)) tmpRepList.add(kQ3);
    }
    private void generateHelicopterCoefficients(){

        String rootName="Heli " +rotorName;
        //Add to correct list
        ArrayList<Report> tmpRepList=helicopterReports;
        //Force & Moment  Coefficients
        ForceCoefficientReport cH;  MomentCoefficientReport cMRX;
        ForceCoefficientReport cRY; MomentCoefficientReport cMRY;
        ForceCoefficientReport cT;  MomentCoefficientReport cQ;
        //
        // Helicopter Force Coefficients
        //  Defn: F / (rho*A_disk( omega *R )^2
        //  CCM+: F / (1/2 * rho * V^2 * A)
        //  Appr: Use (sqrt(2) * omega*R) where velocity would be; A_disk for reference area

        double tipVel = Math.abs(Math.sqrt(2) * rotorSpeed * bladeRadius);
        cH =ReportTool.forceCoefReport(simu,rootName+ " - cH",rotorCsys, xVector,
                refRho,tipVel,bladeArea,geomSurfs,proxyRep);
        cRY=ReportTool.forceCoefReport(simu,rootName+" - cRY",rotorCsys, yVector,
                refRho,tipVel,bladeArea,geomSurfs,proxyRep);
        cT =ReportTool.forceCoefReport(simu,rootName+ " - cT",rotorCsys, zVector,
                refRho,tipVel,bladeArea,geomSurfs,proxyRep);
        if(!tmpRepList.contains(cH))  tmpRepList.add(cH);
        if(!tmpRepList.contains(cRY)) tmpRepList.add(cRY);
        if(!tmpRepList.contains(cT))  tmpRepList.add(cT);
        //
        // Propeller Moment Coefficients
        //  Defn: M / (rho*A_disk( omega *R )^2 * 'r'
        //  CCM+: M / (1/2 * rho * V^2 * A) * R
        //  Appr: Use (sqrt(2) * omega*R) where velocity would be; A_disk for reference area; blade radius for 'r'
        cMRX=ReportTool.momentCoefReport(simu,rootName+" - cMRX",rotorCsys,zeroOrigin,
                xVector,refRho,tipVel,bladeArea,bladeRadius,geomSurfs,proxyRep);
        cMRY=ReportTool.momentCoefReport(simu,rootName+" - cMRY",rotorCsys,zeroOrigin,
                yVector,refRho,tipVel,bladeArea,bladeRadius,geomSurfs,proxyRep);
        cQ  =ReportTool.momentCoefReport(simu,rootName+ " - cMQ",rotorCsys,zeroOrigin,
                zVector,refRho,tipVel,bladeArea,bladeRadius,geomSurfs,proxyRep);
        if(!tmpRepList.contains(cMRX)) tmpRepList.add(cMRX);
        if(!tmpRepList.contains(cMRY)) tmpRepList.add(cMRY);
        if(!tmpRepList.contains(cQ))   tmpRepList.add(cQ);
    }
    
    public Collection<PartSurface> getBladeSurfs(){
        return bladeSurfs;
    }
    
    private void setupBladeSurfs(){
        for(PartSurface tmpSurf:geomPart.getPartSurfaces()){
            String tmpName=tmpSurf.getPresentationName();
            String bladeID = globalNames.getRotorBladeID();
            if(tmpName.startsWith(bladeID)||tmpName.contains("." + bladeID)){
                bladeSurfs.add(tmpSurf);
            }
        }

    }
    
    private void generateYPlusReports(){
        String rootName="Wall Y+ (Blades) "+ rotorName;
        PrimitiveFieldFunction pFF = 
            ((PrimitiveFieldFunction) simu.getFieldFunctionManager().getFunction("WallYplus"));
        // For rotors, we still take only the blade boundary for y+
        //   the hub is irrelevant to performance
        // TSO Note: I'm not sure whether the GeometryPart interpolation works for 
        //   moving meshes as of 11.06.010
        MaxReport maxRep=ReportTool.maxReport(simu,rootName + " Max", pFF, bladeSurfs, proxyRep);
        MinReport minRep=ReportTool.minReport(simu,rootName + " Min", pFF, bladeSurfs, proxyRep);
        AreaAverageReport aveRep=ReportTool.surfAveReport(simu,rootName + " Surf Ave", pFF, bladeSurfs, proxyRep);
        //
        if(!wallYplusReports.contains(aveRep)) wallYplusReports.add(aveRep); //most useful
        if(!wallYplusReports.contains(maxRep)) wallYplusReports.add(maxRep); //watch for high values
        if(!wallYplusReports.contains(minRep)) wallYplusReports.add(minRep); //min is least useful
    }
    //
    // Monitors
    public void generateIterationMonitors(ArrayList<Report> reportList,ArrayList<Monitor> monList,
            int plotLimit,int itFreq,int startIt){
        for(Report tmp:reportList){
            Monitor tmpMon = MonitorTool.reportIterationMonitor(tmp.getSimulation(),tmp,plotLimit,itFreq,startIt);
            if(!monList.contains(tmpMon)) monList.add(tmpMon);
        }
    }
    public void generateUnsteadyMonitors(ArrayList<Report> reportList,ArrayList<Monitor> monList,
            int plotLimit,int tsFreq,int startTS){
        for(Report tmp:reportList){
            Monitor tmpMon = MonitorTool.reportUnsteadyMonitor(tmp.getSimulation(),tmp,plotLimit,tsFreq,startTS);
            if(!monList.contains(tmpMon)) monList.add(tmpMon);
        }
    }
    //
    // Motions & Reference Systems
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
    public void applyRotorMRF(){
        rotorMotionSpecification.setMotion(stationaryMotion);
        rotorMotionSpecification.setReferenceFrame(rotorRotatingFrame);
    }
    public String getPhysicalRotorModelType(){
        simu.println("1");
        MotionSpecification tmpMS = parentRegion.getValues().get(MotionSpecification.class);
        simu.println("2");
        if(tmpMS.getMotion() instanceof StationaryMotion &&
           tmpMS.getReferenceFrame() instanceof RotatingReferenceFrame){
            return "MRF";
        }else{
            return "RBM";
        }
    }
    
    public void applyRotorRBM(){
        //applies RBM to the Rotor
        rotorMotionSpecification.setReferenceFrame(labFrame);
        rotorMotionSpecification.setMotion(rotorMotion);
    }
    public void initializeRotorMotions(){
        // This must be called after a physics continuum exists!
        // Get Stationary stuff
        //Reference Frame for Lab
        labFrame=((LabReferenceFrame) 
                simu.get(ReferenceFrameManager.class)
                    .getObject("Lab Reference Frame"));
        stationaryMotion=
            (StationaryMotion) simu.get(MotionManager.class)
                    .getObject("Stationary");
        
        //Rotating Domain (RBM/MRF)
        if(!(parentRotatingDomain==null)){
            //Rotor object
            rotorMotion=getRotatingMotion(rotorName);
            rotorRotatingFrame=((RotatingReferenceFrame)
                    simu.get(ReferenceFrameManager.class)
                            .getObject("ReferenceFrame for "+rotorName));
            rotorMotionSpecification=parentRegion.getValues().get(MotionSpecification.class);
            //Convention is Rotor standard local Z-Axis of rotation
            rotorMotion.setCoordinateSystem(rotorCsys);
            rotorMotion.getRotationRate().setValue(rotorSpeed);
//                    .setDefinition(Double.toString(rotorSpeed));
            rotorMotion.getManagedCoordinateSystems()
                    .setObjects(motionRotorCsys);
            rotorMotion.getAxisDirection().setComponents(0.0, 0.0, 1.0);
            
            //If it's a new configuration, set to MRF
            if( (rotorMotionSpecification.getMotion() instanceof StationaryMotion)
              &&(rotorMotionSpecification.getReferenceFrame() instanceof LabReferenceFrame)){
                applyRotorMRF();
            }
        }else{
            //Need to add BEM when ready
        }

    }
    //
    // Coordinate System Stuff
    private int getNBladesFromCsys(){
        String tmpPartName=rotorCsys.getPresentationName();
        char sepChar='_';
        char specialChar='n';
        char nextChar = 'b';
        int firstIndx=0;
        int secondIndx=0;
        for (int i = 0;i<tmpPartName.length();i++){
            char char1=tmpPartName.charAt(i);
            char char2=tmpPartName.charAt(i+1);    
            if(char1==sepChar && char2==specialChar){
                firstIndx=i+1;
                secondIndx=firstIndx;
            }
            if(char1==sepChar && char2==nextChar){
                secondIndx=i;
                break;
            }
        }

        return Integer.parseInt(tmpPartName.substring(firstIndx+1,secondIndx));//removes _
    }
    private double getBladeRadiusFromCsys(){
        String tmpPartName=rotorCsys.getPresentationName();
        char sepChar='_';
        char specialChar='b';
        char nextChar = 'w';
        int firstIndx=0;
        int secondIndx=0;
        for (int i = 0;i<tmpPartName.length();i++){
            char char1=tmpPartName.charAt(i);
            char char2=tmpPartName.charAt(i+1);    
            if(char1==sepChar && char2==specialChar){
                firstIndx=i+1;
                secondIndx=firstIndx;
            }
            if(char1==sepChar && char2==nextChar){
                secondIndx=i;
                break;
            }
        }

        return Double.parseDouble(tmpPartName.substring(firstIndx+1,secondIndx));//removes _
    }
    private double getOmegaFromCsys(){
      /* getOmegaFromCsys 
           assumes that the Coordinate System name is of the form:
         RTR_RotorName_
      */
        String rotorCSysName=rotorCsys.getPresentationName();
        int indxOfSecondSeparator=rotorCSysName.length();
        char sepChar='_';
        char specialChar='w';
        int firstIndx=0;
        for (int i = 0;i<rotorCSysName.length();i++){
            char char1=rotorCSysName.charAt(i);
            char char2=rotorCSysName.charAt(i+1);
            if(char1==sepChar && char2==specialChar){
                firstIndx=i+1;
                break;
            }
        }
        return Double.parseDouble(rotorCSysName.substring(firstIndx+1,indxOfSecondSeparator));//removes _
    }
    private void updateRotorCoordinateSystemSpeeds(double newVal){
        String tmpPartName=rotorCsys.getPresentationName();
        char sepChar='_';
        char specialChar='w';
        int firstIndx=0;
        for (int i = 0;i<tmpPartName.length();i++){
            char char1=tmpPartName.charAt(i);
            char char2=tmpPartName.charAt(i+1);    
            if(char1==sepChar && char2==specialChar){
                firstIndx=i+1;
                break;
            }
        }
        String tmpCsysName = tmpPartName.substring(0,firstIndx+1);//removes _
        String newCsysName = tmpCsysName+Double.toString(newVal);
        //
        cadCsys.setPresentationName(newCsysName);
        rotorCsys.setPresentationName(newCsysName);
        rotorCsysName = rotorCsys.getPresentationName();
        motionRotorCsys.setPresentationName(newCsysName);

    }
    private void getZNormalInterfacePlane(){
        //get plane from derived part tool and assign to plane itself
        PlaneSection tmpPlaneZ = DerivedPartTool.singlePlane(simu, Collections.singleton(parentRegion), "Rotor Z Vectors", rotorCsys, zeroOrigin, zVector);
        ArrayList<Boundary> slidingBounds = new ArrayList();
            for(Boundary tmpB:parentRegion.getBoundaryManager().getBoundaries()){
                if(tmpB.getPresentationName().contains(globalNames.getSlidingStr())){
                    slidingBounds.add(tmpB);
                }
            }
        tmpPlaneZ.getInputParts().setObjects(slidingBounds);
        tmpPlaneZ.setCoordinateSystem(rotorCsys);
        zNormalInterfacePlane = tmpPlaneZ;
    }
    //
    //
    
    
}
