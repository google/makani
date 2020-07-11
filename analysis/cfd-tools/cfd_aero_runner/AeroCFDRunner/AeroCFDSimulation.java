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
import star.vis.*;

public class AeroCFDSimulation {
    //Simulation
    Simulation simu;
    SimTool SimTool;
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
    
    // Geometry
    Collection<GeometryPart> allGeomParts;
    ArrayList<String> airfoilStrList;

    // Geometric configuration information
    double lengthScale;
    
    // Objects
    Domain mainDomain;
    ArrayList<Domain> allDomainObjects = new ArrayList();
    ArrayList<AerodynamicSurface> allAerodynamicSurfaces =
            new ArrayList();
    ArrayList<DeflectableControlSurface> allDeflectableControlSurfaces=
            new ArrayList();
    
    // Meshing
    double baseSize;
    double alpha_0;
    double beta_0;

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
    String wtDomainName;
    ArrayList<Region> allWTRegs = new ArrayList();
    
    //Solver Stuff
    private SolverDriver simDriver;
    
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

    //Statistic Sampling
    int statSamples=500;

    public AeroCFDSimulation(Simulation simu,CoordinateSystem bodyCsys){
        // Simulation controls
        this.simu=simu;
        this.simDriver = new SolverDriver();
        this.proxyRep=SimTool.getSimProxy(simu,"Proxy");

        // Naming conventions
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
        this.inletCsys=SimTool.getNestedCoordinate(bodyCsys,
                globalNames.getInletCsysName());
        
        // Rotate inlet to wind axis
        Units units_0 = 
      ((Units) simu.getUnitsManager().getObject("m"));
        this.inletCsys.getOrigin().setCoordinate(units_0, units_0, units_0
                ,new DoubleVector( new double[] {0.,0.,0.}));
        this.inletCsys.setBasis0(new DoubleVector(new double[] {-1.0,0.0,0.0}));
        this.inletCsys.setBasis1(new DoubleVector(new double[] {0.0,-1.0,0.0}));
    }
    
    //=======================
    // PUBLIC METHODS
    //=======================
    public void setCharacteristicLengthScale(double newValue){
        this.lengthScale=newValue;
    }
    
    public void setMeshingBaseSize(double newValue){
        // Meshing values
        baseSize = newValue;
    }

    //REFERENCE VALUES
    public void setLengthScale(double newVal){
        lengthScale=newVal;
    }
    public void updateAllReferenceValues(Simulation simu, 
                                         double refRho,double refVel,
                                         double refTemp,double refMu,
                                         double  refMa,double  refRe,
                                         double refArea,double[] refMomR){
        // Update the reference values.
        setRefVel(  refVel);
        setRefTemp(refTemp);
        setRefMu(    refMu);
        setRefMach(  refMa);
        setRefRe(    refRe);
        setRefArea(refArea);

        // Set pressure coefficient
        SimTool.setPressureCoefficientFF(simu,refRho,0.0,refVel);

        // Skin friction coefficient
        SimTool.setSkinFrictionCoefficientFF(simu,refRho,refVel);

        //Angle of attack
        // controlled first by rotate assembly
        //  then overwritten by main overset domain if it exists
        String assemblyOpName="Rotate Assembly";
        alpha_0 = MeshOpTool.getRotationAngle(simu,assemblyOpName);

        //SET DEFLECTABLE CONTROL SURFACE REFERENCE VALUES
        for(DeflectableControlSurface tmpSurf:allDeflectableControlSurfaces){
            tmpSurf.setReferenceValues(refRho, refVel, refArea, refMomR);
        }
        
        //SET AERODYNAMIC SURFACE REFERENCE VALUES
        for(AerodynamicSurface aeroSurf:allAerodynamicSurfaces){
            aeroSurf.setReferenceValues(refRho, refVel, refArea, refMomR);
        }
    }

    //MAIN CODE OPTIONS
    public void runPreProc(){
        simu.println("FC MSG: Running preprocessing.");
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
        ArrayList<GeometryPart> tmpList = new ArrayList(); 
        //
        // VOLUME CONTROLS
        //
        ArrayList<GeometryPart> volumeControlParts=getVolumeControlParts();
        CartesianCoordinateSystem assmbCoord=makeCADZeroBodyCSys();
        
        //
        // OVERSET DOMAINS
        //   TODO: Implement - refer back to WT Analysis Class for tips.

        //
        // COLLECT ALL RAW GEOMETRY
        //
        // NOTE: Remember, all CAD geometry is dummy - purely here to make
        //       the final parts that comprise the actual object
        //
        // ALL MAIN CONFIG CAD BODIES
        // If there are multiple CAD Bodies, we will combine them
        // If there is only 1 CAD bodies that has any of these things,
        //   we choose that one
        tmpList.clear();
        tmpList.addAll(allGeomParts);
        ArrayList<String> mainConfigCADBodySurfaceIDs=new ArrayList();
        // Full configuration surfaces
        mainConfigCADBodySurfaceIDs.add(globalNames.getMainWingID());
        mainConfigCADBodySurfaceIDs.add(globalNames.getPylonID());
        mainConfigCADBodySurfaceIDs.add(globalNames.getFuselageID());
        ArrayList<GeometryPart> mainConfigCADBodies=getRawGeometryParts(tmpList,
                mainConfigCADBodySurfaceIDs);
        GeometryPart bigConfigBody;
        if(mainConfigCADBodies.size()>1){ //make a MeshOperationPart
            bigConfigBody=MeshOpTool.uniteParts(simu,"Part Config Body", true,
                    false, mainConfigCADBodies);
        }else{ //only one body matches any of the criteria
            bigConfigBody=mainConfigCADBodies.get(0);
        }

        // GEOMETRY: CAD FLAPS
        tmpList.clear();
        tmpList.addAll(allGeomParts);
        // flap CAD bodies
        ArrayList<String> flapSurfaceIDs=new ArrayList();
        flapSurfaceIDs.add(globalNames.getFlapsID());
        ArrayList<GeometryPart> flapCADparts=getRawGeometryParts(tmpList,
                flapSurfaceIDs);

        // GEOMETRY: ELEVATORS
        tmpList.clear();
        tmpList.addAll(allGeomParts);

        // Flap CAD bodies
        ArrayList<String> elevatorSurfaceIDs=new ArrayList();
        elevatorSurfaceIDs.add(globalNames.getElevatorID());
        ArrayList<GeometryPart> elevatorCADparts=getRawGeometryParts(tmpList,
                elevatorSurfaceIDs);

        // GEOMETRY: RUDDERS
        tmpList.clear();
        tmpList.addAll(allGeomParts);

        // Flap CAD bodies
        ArrayList<String> rudderSurfaceIDs=new ArrayList();
        rudderSurfaceIDs.add(globalNames.getRudderID());
        ArrayList<GeometryPart> rudderCADparts=getRawGeometryParts(tmpList,
                rudderSurfaceIDs);

        // SURFACE REMESHING
        //
        // Note: This section happens before assignment of individual surface
        //       control settings for efficiency purposes in rotation but does
        //       not assing any custom surface control settings until the obect
        //       is instantiated. 
        //
        // Note: We *do not* add STL parts for remeshing. The assumption is that
        //       STL parts have been remeshed elsewhere and are ready to go.
        //
        // Note: Use concurrent meshing for parallel surface meshing
        //
        // LARGE Configuration BODY
        String configSMOpName="SM CAD Config";
        AutoMeshOperation cadConfigSurfaceOp=MeshOpTool
                .surfMeshOp(
                        simu,configSMOpName,baseSize, 100.0,10.0,36.,1.3,false);
        cadConfigSurfaceOp.getInputGeometryObjects().setObjects(bigConfigBody);
        // FOR ALL FLAPS, ELEVATORS, RUDDERS
        // per-part concurrent meshing
        String controlsSMOpName="SM All Deflectable Controls";
        AutoMeshOperation controlsSurfaceOp=MeshOpTool
                .surfMeshOp(
                        simu,controlsSMOpName,baseSize,
                        100.0,10.0,54.,1.1,false);
        controlsSurfaceOp.setMeshPartByPart(true);
        controlsSurfaceOp.getMesherParallelModeOption()
                .setSelected(MesherParallelModeOption.Type.CONCURRENT);
        tmpList.clear();
        tmpList.addAll(flapCADparts);
        tmpList.addAll(elevatorCADparts);
        tmpList.addAll(rudderCADparts);
        controlsSurfaceOp.getInputGeometryObjects().setObjects(tmpList);
        
        //LOCAL ROTATIONS FOR ALL PART-BASED FLAPS, ELEVATORS, RUDDERS
        // Note: This section happens after the creation of all individual
        //       part remeshing because if you are using part remeshing with
        //       discrete intersections, you need to mesh the part first for
        //       efficiency purposes.
        // allow GeometryPart based flaps to have their own rotation operation
        int partCounter = 0;
        tmpList.clear();
        tmpList.addAll(flapCADparts);
        tmpList.addAll(elevatorCADparts);
        tmpList.addAll(rudderCADparts);
        if(tmpList.size()>0){
            ArrayList<CartesianCoordinateSystem> flapCoordSystems=
                    getPartNamedCoordinateSystems(labCsys,tmpList);
            for(GeometryPart tmpPart:tmpList){
                CartesianCoordinateSystem tmpCsys=
                        flapCoordSystems.get(partCounter);
                createGeometryPartRotateCADAngle(tmpPart, tmpCsys,xVector);
                partCounter+=1;
            }
        }

        //
        // CREATE FINAL SOLID CFD-READY CONFIG BODY
        tmpList.clear();
        tmpList.add(bigConfigBody);
        tmpList.addAll(flapCADparts);
        tmpList.addAll(elevatorCADparts);
        tmpList.addAll(rudderCADparts);
        GeometryPart cfdConfigBody=MeshOpTool.uniteParts(
                simu,"CFD Config Body",false, true, tmpList);

        // PERFORM GEOMETRIC ROTATIONS
        //
        simu.println("FC MSG: Setting Beta_o/Alpha_o rotations");
        // Set geometry coordinate system Beta_o value
        // Note: Beta_o gets aligned w/ bodyCsys then rotated to proper beta
        //       angle before the geometry is rotated since the Z-axis rotation
        //       can use either a rotated Beta_o or a non-rotated Beta_o.
        //       Alpha_o then rotates about the Beta_o csys since it is not used
        //       for anything further
        CartesianCoordinateSystem beta0_Csys=
                SimTool.getLabBasedCoordinate(simu,"Beta_o");
        SimTool.modifyLocalCoordinate(
                simu,beta0_Csys,xVector, yVector, zeroVector);
        // allow the mesher to modify at a later time, default action is null
        tmpList.clear();
        tmpList.add(cfdConfigBody);
        MeshOpTool.rotationTransform(
                simu,"CFD Config Beta_0",tmpList,beta0_Csys,zVector);
        MeshOpTool.rotationTransform(
                simu,"CFD Config Alpha_0",tmpList,beta0_Csys,yVector);

        //  CREATE FINAL DOMAIN
        //
        // GET DOMAIN PART AND CREATE SURFACE MESH OP
        simu.println("FC MSG: Looking for raw Domain GeometryPart.");
        GeometryPart domGeomPart=getRawDomainGeometryPart();
        AutoMeshOperation domSurfaceOp=MeshOpTool
                .surfMeshOp(
                        simu,"SM Domain Part",baseSize,
                        1600.0,10.0,32.,1.3,false);
        domSurfaceOp.getInputGeometryObjects().setQuery(null);
        domSurfaceOp.getInputGeometryObjects().setObjects(domGeomPart);
        // Create final CFD domain
        //  Note: This is what we will actually build all the objects from.
        //        (Except overset whenever it gets implemented.)
        //
        tmpList.clear();
        tmpList.add(domGeomPart);
        tmpList.add(cfdConfigBody);
        //
        SubtractPartsOperation subPartOp = 
                MeshOpTool.subtractOp(simu,"Main CFD Domain", "CFD Domain",
                domGeomPart, tmpList);
        GeometryPart cfdDomainPart 
                = MeshOpTool.getSubtractOpPart(simu,subPartOp);
        
        // Volume Mesh Op
        CFD_TrimmerModel wtVolumeMeshOp = new CFD_TrimmerModel(simu,
                "VM CFD Domain",labCsys, baseSize);
        wtVolumeMeshOp.addPart(cfdDomainPart);

        // INSTANTIATE REMAINING OBJECTS
        // PRIMARY (BACKGROUND DOMAIN)
        createPrimaryDomain(cfdDomainPart);
        mainDomain.setDomainSurfMeshOp(domSurfaceOp);
        mainDomain.setDomainVolumeMeshOp(wtVolumeMeshOp.getVMOp());

        // DEFLECTABLE CONTROL SURFACES
        instantiateDeflectableControlSurfaceObjects();
        applyDCSStandardMeshSettings(allDeflectableControlSurfaces);
        if(false){
            simu.println(
                    "-------------------------------------------------------");
            simu.println(
                    "FC MSG: DeflectableControlSurface object information:");
            for(AerodynamicSurface tmpObj:allDeflectableControlSurfaces){
                simu.println("-------------------");
                simu.println(tmpObj.getName());
                simu.println("-------------------");
                simu.println("List of CAD Surfaces: ");
                simu.println("  "+tmpObj.getPartSurfaces());
                simu.println("Mesh Op Info: ");
                simu.println("  "+tmpObj.getSurfMeshCustomControl().
                        getManager().getMeshOperation().getPresentationName());
                simu.println("  "+tmpObj.getVolMeshCustomControl().
                        getManager().getMeshOperation().getPresentationName());
            }
            simu.println(
                    "-------------------------------------------------------");
        }
        
        
        // INSTANTIATE MAIN WING AERO SURFACE OBJECTS
        // Note: Must do this afterward to allow deflectable control surfaces
        //       to be removed from the overall surface lists
        instantiateAerodynamicSurfaceObjects();
        // Set up best practice mesh settings
        applyAeroSurfaceMeshSettings(
                getAeroSurfaceList(globalNames.getMainWingID()),
                100.0,3.125,54.0,24,lengthScale*0.05,lengthScale*1e-6);
        applyAeroSurfaceMeshSettings(
                getAeroSurfaceList(globalNames.getPylonID()),
                100.0,6.25,36.0,24,lengthScale*0.05,lengthScale*1e-5);
        applyAeroSurfaceMeshSettings(
                getAeroSurfaceList(globalNames.getEmpennageID()),
                100.0,3.125,36.0,24,lengthScale*0.05,lengthScale*1e-5);
        if(false){
            simu.println(
                    "-------------------------------------------------------");
            simu.println("FC MSG: AerodynamicSurface object information:");
            for(AerodynamicSurface tmpAS:allAerodynamicSurfaces){
                simu.println("-------------------");
                simu.println(tmpAS.getName());
                simu.println("-------------------");
                simu.println("List of CAD Surfaces: ");
                simu.println("  "+tmpAS.getPartSurfaces());
                simu.println("Mesh Settings for: "+tmpAS.getName());
                simu.println("  SM Op: "+tmpAS.getSurfMeshCustomControl().
                        getManager().getMeshOperation().getPresentationName());
                simu.println("  VM Op: "+tmpAS.getVolMeshCustomControl().
                        getManager().getMeshOperation().getPresentationName());
            }
            simu.println("-------------------------------------------------------");
        }
        //
        // ORGANIZE REMAINING NON-OBJECT PART SURFACES INTO BOUNDARIES
        // Put remaining surfaces into appropriate boundaries
        for(Domain tmpDomain:allDomainObjects){
            Region tmpRegion=tmpDomain.getRegion();
            Collection<PartSurface> allRemainingSurfaces = 
                    tmpDomain.getDomainPart().getPartSurfaces();
            ArrayList<PartSurface> allUsedSurfaces = getAllInUsePartSurfaces();
            allRemainingSurfaces.removeAll(allUsedSurfaces);

            ArrayList<PartSurface> preFixRemainingSurfaces = new ArrayList();
            for(String tmpPre:globalNames.getAllFullConfigurationPreFixes()){
                for(PartSurface tmpSurf:allRemainingSurfaces){
                    String tmpSurfName=tmpSurf.getPresentationName();
                    if(tmpSurfName.startsWith(tmpPre)
                            || tmpSurfName.contains("."+tmpPre)){
                        preFixRemainingSurfaces.add(tmpSurf);
                    }
                }
                addSurfaceToRegionGroupBoundary(
                        tmpRegion,tmpPre,preFixRemainingSurfaces);
            }
        }
        
        // REGIONS CLEANUP
        // Clean up the Default boundary (this should be empty anyways)
        simu.println(" ");
        simu.println("WT MSG: Regions all set up. Removing Default boundary.");
        //cleanupDefaultBoundary(allWTRegs);
        simu.println(" ");

    }

    //GEOMETRY
    private GeometryPart getRawDomainGeometryPart(){
        /* Method to determine which part contains the domain */
        ArrayList<String> domainIdent = new ArrayList();
        domainIdent.add(inStr);    // inlet
        domainIdent.add(fsStr);    // freestream
        return MeshOpTool.findGeometryPart(simu,allGeomParts,domainIdent);
    }
    private ArrayList<PartSurface> getPartSurfaces(
            GeometryPart tmpPart,String stringID){
        ArrayList<PartSurface> retArr = new ArrayList();
        for(PartSurface tmpSurf:tmpPart.getPartSurfaces()){
                String surfName=tmpSurf.getPresentationName();
                if(surfName.startsWith(stringID)
                        ||surfName.contains("."+stringID)){
                    retArr.add(tmpSurf);
                }
            }
        return retArr;
    }
    private GeometryPart getRawGeometryPart(ArrayList<GeometryPart> geomParts,
            ArrayList<String> stringIdentifiers){
        return MeshOpTool.findGeometryPart(simu,geomParts, stringIdentifiers);
    }
    private ArrayList<GeometryPart> getRawGeometryParts(
            ArrayList<GeometryPart> geomParts,
            ArrayList<String> stringIdentifiers){
        return MeshOpTool.findGeometryParts(simu,geomParts, stringIdentifiers);
    }    
    private Collection<GeometryPart> removeMeshOpParts(
            Collection<GeometryPart> theseGeomParts){
        ArrayList<MeshOperationPart> meshOpParts=new ArrayList();
        for(GeometryPart tmpPart:theseGeomParts){ //remove mesh operation parts
            if(tmpPart instanceof MeshOperationPart){
                meshOpParts.add((MeshOperationPart) tmpPart);
            }
        }
        theseGeomParts.removeAll(meshOpParts);
        return theseGeomParts;
    }
    
    //REGIONS
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
    private void addSurfaceToRegionGroupBoundary(Region tmpReg, String bndyID,
            ArrayList<PartSurface> geomSurfs){
        Boundary tmpBndy = setUpBoundary(tmpReg,globalNames.getBndyName(bndyID));
        //add necessary surfaces to the boundary
        Collection<PartSurface> tmpColl = 
                tmpBndy.getPartSurfaceGroup().getObjects();
        tmpColl.addAll(geomSurfs);
        tmpBndy.getPartSurfaceGroup().setObjects(tmpColl);
        //Set BC Type
        tmpBndy.setBoundaryType(WallBoundary.class);
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
        // must duplicate Body to Body_Rotate, otherwise meshOp will go out of
        // date on rotation of main coordinate system and PBM pipeline will show
        // not-up-to-date
        CartesianCoordinateSystem assmbCoord
                =SimTool.getLabBasedCoordinate(
                        simu,"CAD Zero "+bodyCsys.getPresentationName());
        assmbCoord.copyProperties(bodyCsys);
        //opposite direction of body, same y vector
        double oldxValue=bodyCsys.getBasis0().toDoubleArray()[0];
        double oldyValue=bodyCsys.getBasis1().toDoubleArray()[1];
        double newXMultiplier=1.0;
        if(oldxValue<0.0) newXMultiplier=-1.0;
        assmbCoord.setBasis0(
                new DoubleVector(new double[] {newXMultiplier,0.0,0.0}));
        double newYMultiplier=1.0;
        if(oldyValue<0.0) newYMultiplier=-1.0;
        assmbCoord.setBasis1(
                new DoubleVector(new double[] {0.0,newYMultiplier,0.0}));
        
        return assmbCoord;
    }
    
    
    // MESHING
    private ArrayList<GeometryPart> getVolumeControlParts(){
        /* Figures out what Objects are actually in the windtunnel */
        /* Method to determine which part contains the domain */
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
    
    private void createGeometryPartRotateCADAngle(GeometryPart tmpPart,
            CartesianCoordinateSystem tmpCsys,double[] rotVector){
        //  Airfoils: 
        //    allow all tmpPart to rotate on the turntable
        //
        String partName = tmpPart.getPresentationName();
        simu.println("FC MSG: Operation set for: "+tmpPart);
        MeshOpTool.rotationTransform(
                simu,"Rotate "+partName,tmpPart,tmpCsys,rotVector);
    }
    
    
    private void createPrimaryDomain(GeometryPart wtPart){
        mainDomain = new Domain(simu,wtPart);
        simu.println("WT MSG: Setting up windtunnel region.");
        mainDomain.setUpRegion(simu);
        simu.println("WT MSG: Setting up windtunnel boundaries.");
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
            simu.println("FC MSG: Instantiating primary Domain...");
            //Figure out which geometry creates the main region
            Collection<Region> allRegions=getRegions();
            GeometryPart mainWTPart=null;
            try{
                Region mainWTRegion = getPrimaryRegion(allRegions);
                mainWTPart = getRegionPart(mainWTRegion);
            }catch(NeoException e){
                simu.println("WT MSG: "
                        +"Found pre-existing regions, no WT region detected.");
            }
            //Create Domain objet
            mainDomain = new Domain(simu,mainWTPart);
            wtDomainName = mainDomain.getName(); // let FC runner know you are here
            
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
        simu.println("FC MSG: Domain "+mainDomain.getName()+" instantiated.");

    }
    
    
    private ArrayList<AerodynamicSurface> getAeroSurfaceList(String preFixID){
        ArrayList<AerodynamicSurface> retList=new ArrayList();
        for(AerodynamicSurface tmpAS:allAerodynamicSurfaces){
            if(tmpAS.getPrefix().startsWith(preFixID)
                    || tmpAS.getPrefix().equals(preFixID)){
                retList.add(tmpAS);
            }
        }
        return retList;
    }
    
    private void instantiateAerodynamicSurfaceObjects(){
        int i_start=0;
        int n_avail=0;

        if(allAerodynamicSurfaces.isEmpty()){
            //MAIN WING COMPONENTS
            instantiateAerodynamicSurfaceObject(
                    globalNames.getMainWingID(),"Main Wing");
            
            //PYLONS
            simu.println("FC MSG: Looking for Pylons with prefix "
                    +globalNames.getPylonID());
            for(int i=1;i<=9;i++){
                simu.println("");
                instantiateAerodynamicSurfaceObject(
                        globalNames.getPylonID()+i,"Pylon "+i);
            }

            //H Tails
            simu.println("FC MSG: Looking for H Tails with prefix "
                    +globalNames.getEmpennageID());
            i_start=1;
            n_avail=3;
            for(int i=i_start;i<=(i_start+n_avail);i++){
                simu.println("");
                instantiateAerodynamicSurfaceObject(
                        globalNames.getEmpennageID()+i,"H Tail "+(i-i_start+1));
            }

            //V Tails
            simu.println("FC MSG: Looking for V Tails with prefix "
                    +globalNames.getEmpennageID());
            i_start=7;
            n_avail=2;
            for(int i=i_start;i<=(i_start+n_avail);i++){
                simu.println("");
                instantiateAerodynamicSurfaceObject(
                        globalNames.getEmpennageID()+i,"V Tail "+(i-i_start+1));
            }

            //Display all AerodynamicSurface created
            simu.println("");
            for(AerodynamicSurface tmp:allAerodynamicSurfaces){
                simu.println("FC MSG: AS "+tmp.getName()+" instantiated.");
            }
        }

    }
    
    private ArrayList<PartSurface> getAllInUsePartSurfaces(){
        ArrayList<PartSurface> inUseSurfaceList=new ArrayList();
        for(AerodynamicSurface tmpAS:allAerodynamicSurfaces){
            inUseSurfaceList.addAll(tmpAS.getPartSurfaces());
        }
        for(DeflectableControlSurface tmpDCS:allDeflectableControlSurfaces){
            inUseSurfaceList.addAll(tmpDCS.getPartSurfaces());
        }
        return inUseSurfaceList;
    }
    
    
    private void instantiateAerodynamicSurfaceObject(
            String preFixID,String aeroSurfaceName){
        /* Used to instantiate Aerodynamic Surface from a simulation
           Can be extracted from the following domains:
            Primary Domain
        */
        double [] momentAxis = {lengthScale,lengthScale,lengthScale};
        
        // CANNOT PULL FROM THESE SURFACES
        // Note: this might do better as a check-if-the part has it list
        ArrayList<PartSurface> cannotUseSurfaceList=getAllInUsePartSurfaces();

        simu.println("FC MSG: Instantiating Aerodynamic Surface w/ preFixID "
                +preFixID);

        // Set up airfoil boundaries (if they exist)
        for(Domain tmpDomain:allDomainObjects){
            simu.println("FC MSG: There are "+allDomainObjects.size()+
                    " domains to search for Aerodynamic Surfaces.");
            simu.println("FC MSG: Looking in Domain "+tmpDomain.getName());
            GeometryPart domainPart=tmpDomain.getDomainPart();

            // PRIMARY IDEAS
            //   b/w Aerodynamic Surfaces
            //
            // Note: are assumed to be part of the CFD Config Body Part at
            //       this time. Therefore, they have no other CAD body
            //       except what is involved in the Unite operation between
            //       the Config Body and the Deflectable Control Surfaces
            //
            simu.println("FC MSG: Checking whether domain is MeshOp Part.");
            if(domainPart instanceof MeshOperationPart){
                simu.println("FC MSG: Domain is MeshOp Part.");
                // get the operation that created the domain Part
                MeshOperation domainOp=
                        ((MeshOperationPart) domainPart).getOperation();
                Collection<GeometryPart> domainOpParts=
                        domainOp.getInputGeometryObjects().getLeafParts();

                //
                // MAIN DOMAIN HANDLING
                if (tmpDomain==mainDomain){
                    simu.println("FC MSG: This domain is the mainDomain.");
                    domainOpParts.remove(getRawDomainGeometryPart());
                    //domainOpParts.remove(tmpDomain.getDomainPart());
                    //need to go one more meshUnitOp 'up' to get 
                    //  the final domain part
                    GeometryPart objectSurfaceBody=tmpDomain.getDomainPart();
                    if(domainOpParts.size()<2){
                        GeometryPart solidConfigBody=
                                (GeometryPart) domainOpParts.toArray()[0];
                        if(solidConfigBody instanceof MeshOperationPart){
                            MeshOperation solidConfigBodyOp=
                                    ((MeshOperationPart) solidConfigBody).getOperation();
                            Collection<GeometryPart> solidConfigBodyOpParts=
                                    solidConfigBodyOp.getInputGeometryObjects()
                                            .getLeafParts();
                            ArrayList<GeometryPart> tmpList = new ArrayList();
                            tmpList.addAll(solidConfigBodyOpParts);
                            ArrayList<String> mainConfigCADBodySurfaceIDs=
                                    new ArrayList();
                            // Config bodies
                            // note .getHardwareID is not here because hardware can be anywhere
                            mainConfigCADBodySurfaceIDs.
                                    add(globalNames.getFuselageID());
                            GeometryPart mainConfigCADBody=
                                    getRawGeometryPart(tmpList,
                                            mainConfigCADBodySurfaceIDs);
                        }
                    }
                    //Mesh Ops
                    String controlsSMOpName="SM CAD Config";
                    AutoMeshOperation domSurfMeshOp=MeshOpTool.
                            getAutoMeshOp(simu,controlsSMOpName);
                    AutoMeshOperation domVolMeshOp=tmpDomain.
                            getDomainVolumeMeshOp();

                    // At this stage we should now know
                    //   1) The name of primary CAD body
                    //   2) Potential custom controls and settings

                    // Look through the Part Surface list to see if there
                    // are any matching known prefixes in the surface list
                    // Make list of known deflectable control surface prefixes

                    //Create AeroDynamic Surface Object
                    ArrayList<PartSurface> tmpList=new ArrayList();
                    //we must pull surfaces from the mainConfigCADBody, not the CAD Part!
                    for(PartSurface tmpSurf:objectSurfaceBody.
                            getPartSurfaces()){
                        String tmpSurfName=tmpSurf.getPresentationName();
                        if(tmpSurfName.contains("."+preFixID)){
                            tmpList.add(tmpSurf);
                        }
                    }
                    //
                    tmpList.removeAll(cannotUseSurfaceList);
                    //
                    if(tmpList.size()>0){
                        //Instantiate the object!
                        AerodynamicSurface newAS = 
                            new AerodynamicSurface(simu,
                                tmpList,preFixID.length());
                        newAS.setPrefix(preFixID);
                        newAS.setName(aeroSurfaceName);
                        newAS.setRegion(tmpDomain.getRegion());
                        newAS.addToGroupBoundary(preFixID.substring(0,1));
                       
                        allAerodynamicSurfaces.add(newAS);

                        //No geometry transformations for aero surfaces

                        //Physics
                        newAS.setReferenceValues(
                                refRho,refVel,refArea,momentAxis);

                        //Meshing
                        String contDesc="UNKNOWN";
                        if(preFixID.startsWith(globalNames.getMainWingID())||
                           preFixID.contains("."+globalNames.getMainWingID())){
                            contDesc="000 Main Wing";
                        }else if(preFixID.startsWith(globalNames.getPylonID())||
                           preFixID.contains("."+globalNames.getPylonID())){
                            contDesc="000 Pylon";
                        }else if(preFixID.startsWith(
                                globalNames.getEmpennageID())||
                           preFixID.contains("."+globalNames.getEmpennageID())){
                           contDesc="000 Empennage";
                        }
                        else if(preFixID.startsWith(
                                globalNames.getFuselageID())
                                || preFixID.
                                        contains("."
                                                +globalNames.getFuselageID())){
                           contDesc="000 Fuselage";
                        }
                        String cntrlTitleStr=preFixID.substring(0,1)+contDesc;
                        
                        //Let airfoil know about its surface mesh controls
                        SurfaceCustomMeshControl surfSurfControl = 
                                MeshOpTool.surfControl(
                                        domSurfMeshOp, cntrlTitleStr);
                        newAS.setSurfMeshCustomControl(surfSurfControl);
                        //MeshOpTool.surfFilter(surfSurfControl,nDigitsToFilter);
                        SurfaceCustomMeshControl surfSurfControl_TE = 
                                MeshOpTool.surfControl(
                                        domSurfMeshOp, cntrlTitleStr+" TE");
                        newAS.setSurfMeshCustomTEControl(surfSurfControl_TE);
                        //MeshOpTool.surfFilter(surfSurfControl_TE,nDigitsToFilter);
                        //Let airfoil know about its volume mesh controls
                        SurfaceCustomMeshControl volSurfControl=
                                MeshOpTool.surfControl(
                                        domVolMeshOp, cntrlTitleStr);
                        newAS.setVolMeshCustomControl(volSurfControl);
                        //MeshOpTool.surfFilter(volSurfControl,nDigitsToFilter);
                        SurfaceCustomMeshControl volSurfControlTE=
                                MeshOpTool.surfControl(
                                        domVolMeshOp, cntrlTitleStr+" TE");
                        newAS.setVolMeshCustomTEControl(volSurfControlTE);
                    }
                }
            }
        }

    }
    
    private void instantiateDeflectableControlSurfaceObjects(){
        /* Used to instantiate Deflectable Control Surface from a simulation
           Can be extracted from the following domains:
            Primary Domain

        */
        double [] momentAxis = {lengthScale,lengthScale,lengthScale};
        if(allDeflectableControlSurfaces.isEmpty()){
            simu.println("FC MSG: Instantiating DCS Objects...");

            // Set up airfoil boundaries (if they exist)
            for(Domain tmpDomain:allDomainObjects){
                simu.println("FC MSG: There are "+allDomainObjects.size()+
                        " domains to search for Deflectable Control Surfaces.");
                simu.println("FC MSG: Looking in Domain "+tmpDomain.getName());
                GeometryPart domainPart=tmpDomain.getDomainPart();

                // PRIMARY POINT OF DIFFERENCE
                //   b/w WT Airfoils & DeflectableControlSurfaces
                //
                // Note: get associated automated surface and volume mesh ops
                //       that the domain is associated with
                //       For full configurations, the volume mesh operation
                //       of the entire domain is the volume mesh operation
                //       However, its surface mesh comes from the controls
                //       mesh operation.
                //

                simu.println("FC MSG: Checking whether domain is MeshOp Part.");
                if(domainPart instanceof MeshOperationPart){
                    simu.println("FC MSG: Domain is MeshOp Part.");
                    // get the operation that created the domain Part
                    MeshOperation domainOp=
                            ((MeshOperationPart) domainPart).getOperation();
                    Collection<GeometryPart> domainOpParts=
                            domainOp.getInputGeometryObjects().getLeafParts();
                    ArrayList<GeometryPart> cadBasedDCSParts=new ArrayList();

                    //
                    // MAIN DOMAIN HANDLING
                    if (tmpDomain==mainDomain){
                        simu.println("FC MSG: This domain is the mainDomain.");
                        simu.println("FC MSG: This domain is made from "
                                +domainOpParts.size()+" CAD parts.");
                        
                        domainOpParts.remove(getRawDomainGeometryPart());
                        //domainOpParts.remove(tmpDomain.getDomainPart());
                        //need to go one more meshUnitOp 'up' to get 
                        //  the final domain part
                        GeometryPart objectSurfaceBody=
                                tmpDomain.getDomainPart();
                        if(domainOpParts.size()<2){
                            simu.println("FC MSG: Extracting CFD Config Unite Operation Parts.");
                            GeometryPart solidConfigBody=
                                    (GeometryPart) domainOpParts.toArray()[0];
                            if(solidConfigBody instanceof MeshOperationPart){
                                MeshOperation solidConfigBodyOp=
                                        ((MeshOperationPart) solidConfigBody)
                                                .getOperation();
                                Collection<GeometryPart> solidConfigBodyOpParts=
                                        solidConfigBodyOp.
                                                getInputGeometryObjects().
                                                getLeafParts();
                                simu.println("FC MSG: These GeometryParts are in CFD Config Unite Op.");
                                for(GeometryPart tmpPart:solidConfigBodyOpParts){
                                    simu.println(tmpPart.getPresentationName());
                                }
                                ArrayList<GeometryPart> tmpList = new ArrayList();
                                tmpList.addAll(solidConfigBodyOpParts);
                                ArrayList<String> mainConfigCADBodySurfaceIDs=new ArrayList();
                                // config bodies
                                //mainConfigCADBodySurfaceIDs.add(globalNames.getWingID());
                                // note .getHardwareID is not here because hardware can be anywhere
                                //mainConfigCADBodySurfaceIDs.add(globalNames.getPylonID());
                                mainConfigCADBodySurfaceIDs.add(globalNames.getFuselageID());
                                GeometryPart mainConfigCADBody=getRawGeometryPart(tmpList,
                                    mainConfigCADBodySurfaceIDs);
                                simu.println("FC MSG: Main Config CAD Body name is "
                                        +mainConfigCADBody.getPresentationName());
                                //this list contains *any* CAD-based deflectable flap bodies
                                solidConfigBodyOpParts.remove(mainConfigCADBody);
                                cadBasedDCSParts.addAll(solidConfigBodyOpParts);
                                simu.println("FC MSG: Detected deflectable control surface bodies");
                                simu.println(cadBasedDCSParts);
                            }
                        }
                        simu.println("FC MSG: Mesh Operations are in use.");
                        //Volume Mesh Op
                        AutoMeshOperation domVolMeshOp=tmpDomain.getDomainVolumeMeshOp();

                        // Any deflectable control surface will be meshed in this
                        // operation when the part is part of the main domain
                        String controlsSMOpName="SM All Deflectable Controls";
                        AutoMeshOperation domSurfMeshOp=MeshOpTool.getAutoMeshOp(simu,controlsSMOpName);
                        simu.println("FC MSG: Surface Mesh Op: "+domSurfMeshOp.getPresentationName());
                        simu.println("FC MSG: Volume  Mesh Op: "+domVolMeshOp.getPresentationName());
                        
                        // At this stage we should now know
                        //   1) The name of the CAD airfoil 
                        //   2) everything about the CAD geometry manipulations,
                        //      i.e. rotation operation & the angle at
                        //      which that rotation operation is set!
                        //   3) The custom controls and settings
                        // Look through the Part Surface list to see if there are any
                        //   airfoil prefixes in this domain's part list

                        // Make list of known deflectable control surface prefixes
                        ArrayList<String> dcsNames = new ArrayList();
                        dcsNames.add(globalNames.getFlapsID());
                        dcsNames.add(globalNames.getElevatorID());
                        dcsNames.add(globalNames.getRudderID());
                        for(GeometryPart tmpPart:cadBasedDCSParts){
                            String cadCSysName=tmpPart.getPresentationName();
                            String surfPrefix="";
                            for(String tmpPrefix:dcsNames){
                                for(PartSurface tmpSurf:tmpPart.getPartSurfaces()){
                                    String tmpSurfName=tmpSurf.getPresentationName();
                                    if(tmpSurfName.contains("."+tmpPrefix)||
                                        tmpSurfName.startsWith(tmpPrefix)){
                                        surfPrefix=tmpPrefix;
                                    }
                                }
                            }
                            
                            //Create Deflectable Control Surface Object
                            CartesianCoordinateSystem cadCSys = SimTool.getLabBasedCoordinate(simu,cadCSysName); 
                            ArrayList<PartSurface> tmpList=new ArrayList();
                            //we must pull surfaces from the mainConfigCADBody, not the CAD Part!
                            for(PartSurface tmpSurf:objectSurfaceBody.getPartSurfaces()){
                                String tmpSurfName=tmpSurf.getPresentationName();
                                for(PartSurface cadSurf:tmpPart.getPartSurfaces()){
                                    String cadSurfName=cadSurf.getPresentationName();
                                    if(tmpSurfName.contains("."+cadSurfName)){
                                        tmpList.add(tmpSurf);
                                    }
                                }
                            }

                            //Instantiate the object!
                            DeflectableControlSurface newDCS = 
                                new DeflectableControlSurface(simu,
                                    tmpList,surfPrefix.length(),cadCSys);
                            newDCS.setPrefix(surfPrefix);
                            newDCS.setName(cadCSysName);
                            newDCS.setRegion(tmpDomain.getRegion());
                            newDCS.addToGroupBoundary(surfPrefix);
                            allDeflectableControlSurfaces.add(newDCS);

                            //Let DCS know about its geometry transformations
                            String dcsName=newDCS.getName();
                            //Physics
                            newDCS.setReferenceValues(refRho,refVel,refArea,momentAxis);
                            //Geometry Manipulation
                            if(tmpDomain instanceof OversetDomain){
                                newDCS.setDeflectionAngle(0.0);
                            }else{
                                newDCS.setDeflectionAngle(MeshOpTool.getRotationAngle(simu,"Rotate "+dcsName));
                            }
                            //if deflection angle is non-zero, we must re-rotate our local body coords
                            if(Math.abs(newDCS.getDeflectionAngle())>1e-5){
                                CartesianCoordinateSystem tmpCSys = newDCS.getRotationCsys();
                                tmpCSys.setBasis0(new DoubleVector( new double[] {1.0,0.0,0.0}));
                                tmpCSys.setBasis1(new DoubleVector( new double[] {0.0,1.0,0.0}));
                                SimTool.rotateCoordinateSystem(simu,newDCS.getDeflectionAngle(), zVector, tmpCSys, tmpCSys);
                            }
                            //Meshing
                            String contDesc="UNKNOWN";
//                            int nDigitsToFilter=4;
                            if(surfPrefix.startsWith(globalNames.getFlapsID())||
                               surfPrefix.contains("."+globalNames.getFlapsID())){
                                contDesc="000 Flaps";
//                                nDigitsToFilter=1;
                            }else if(surfPrefix.startsWith(globalNames.getElevatorID())||
                               surfPrefix.contains("."+globalNames.getElevatorID())){
                                contDesc="00 Elevator";
//                                nDigitsToFilter=2;
                            }else if(surfPrefix.startsWith(globalNames.getRudderID())||
                               surfPrefix.contains("."+globalNames.getRudderID())){
                                contDesc="00 Rudder";
//                                nDigitsToFilter=2;
                            }
//                            simu.println("This DCS filter n: "+nDigitsToFilter);
                            String cntrlTitleStr=surfPrefix+contDesc;
                            //Let airfoil know about its surface mesh controls
                            SurfaceCustomMeshControl surfSurfControl = MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr);
                            newDCS.setSurfMeshCustomControl(surfSurfControl);
                            //MeshOpTool.surfFilter(surfSurfControl,nDigitsToFilter);
                            SurfaceCustomMeshControl surfSurfControl_TE = MeshOpTool.surfControl(domSurfMeshOp, cntrlTitleStr+" TE");
                            newDCS.setSurfMeshCustomTEControl(surfSurfControl_TE);
                            //MeshOpTool.surfFilter(surfSurfControl_TE,nDigitsToFilter);
                            //Let airfoil know about its volume mesh controls
                            SurfaceCustomMeshControl volSurfControl=MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr);
                            newDCS.setVolMeshCustomControl(volSurfControl);
                            //MeshOpTool.surfFilter(volSurfControl,nDigitsToFilter);
                            SurfaceCustomMeshControl volSurfControlTE=MeshOpTool.surfControl(domVolMeshOp, cntrlTitleStr+" TE");
                            newDCS.setVolMeshCustomTEControl(volSurfControlTE);
                            //MeshOpTool.surfFilter(volSurfControlTE,nDigitsToFilter);
                            //
                        }
                    }
                }
            }
            //Airfoils - set custom mesh settings
            for(AerodynamicSurface tmp:allDeflectableControlSurfaces){
                simu.println("FC MSG: DCS "+tmp.getName()+" instantiated.");
            }
        }
    }
    
    
    private void applyAeroSurfaceMeshSettings(
            ArrayList<AerodynamicSurface> allObjects,double targetPct,double minPct, double nPtsOnCircle,
                int nPrismLayers, double prismLayerThickness,double firstCellThickness){
        for(AerodynamicSurface tmpAS:allObjects){
            // custom Surface Mesh controls per airfoil
            //   Best practice values:
            double foilTargetPct   =  targetPct;
            double foilMinPct      =  minPct;
            double foilNPtsCircle  =  nPtsOnCircle;
            double foilTETargetPct =   foilTargetPct*0.25;
            double foilTEMinPct    =   foilTargetPct*0.125;
            int nDigitsToFilter=1;

            //SURFACE MESH SETTINS
            //main body control
            SurfaceCustomMeshControl surfSurfControl = tmpAS.getSurfMeshCustomControl();
            tmpAS.setSurfMeshCustomControl(surfSurfControl);
            MeshOpTool.surfFilter(      surfSurfControl,nDigitsToFilter);
            MeshOpTool.surfCustTargSize(surfSurfControl,"Relative",foilTargetPct);
            MeshOpTool.surfCustMinSize( surfSurfControl,"Relative",foilMinPct);
            MeshOpTool.surfNCircle(     surfSurfControl,foilNPtsCircle);

            //trailing edges
            SurfaceCustomMeshControl surfSurfControl_TE = tmpAS.getSurfMeshCustomTEControl();
            tmpAS.setSurfMeshCustomTEControl(surfSurfControl_TE);
            MeshOpTool.surfFilter_TE(  surfSurfControl_TE,nDigitsToFilter);
            MeshOpTool.surfCustMinSize(surfSurfControl_TE,"Relative",foilTEMinPct);
            MeshOpTool.surfEdgeProx(   surfSurfControl_TE,nDigitsToFilter);

            //VOLUME MESH SETTINGS
            // Defaults
            int nPrisms = nPrismLayers; //use even #s
            double prismThickness=prismLayerThickness;
            double firstCellThick=firstCellThickness;

            //Custom volume control settings
            SurfaceCustomMeshControl volSurfControl= tmpAS.getVolMeshCustomControl();
            tmpAS.setVolMeshCustomControl(volSurfControl);
            //main body control
            MeshOpTool.surfFilter(volSurfControl,nDigitsToFilter);
            MeshOpTool.surfPrismThick( volSurfControl,"Absolute",prismThickness);
            MeshOpTool.surfPrismNearWall( volSurfControl,firstCellThick);
            MeshOpTool.surfPrismOverride( volSurfControl, true);
            MeshOpTool.surfPrismNumLayers(volSurfControl,nPrisms/2);
            tmpAS.setVolMeshCustomControl(volSurfControl);
            //trailing edges
            SurfaceCustomMeshControl volSurfControl_TE = tmpAS.getVolMeshCustomTEControl();
            MeshOpTool.surfFilter_TE(  volSurfControl_TE,nDigitsToFilter);
            MeshOpTool.surfPrismThick( volSurfControl_TE,"Absolute",prismThickness*0.25);
            MeshOpTool.surfPrismNearWall( volSurfControl_TE,firstCellThick*4.0);
            MeshOpTool.surfPrismOverride( volSurfControl_TE, true);
            MeshOpTool.surfPrismNumLayers(volSurfControl_TE,nPrisms/2);
            tmpAS.setVolMeshCustomTEControl(volSurfControl_TE);
        }
    }
    
    
    private void applyDCSStandardMeshSettings(ArrayList<DeflectableControlSurface> allObjects){
        for(DeflectableControlSurface tmpDCS:allObjects){
            // custom Surface Mesh controls per airfoil
            //   Best practice values:
            double foilTargetPct   =  12.5;
            double foilMinPct      =  3.125;
            double foilNPtsCircle  =  54.0;
            double foilTETargetPct =   foilTargetPct*0.25;
            double foilTEMinPct    =   foilTargetPct*0.125;

            int nDigitsToFilter=tmpDCS.getPrefixSize();

            //SURFACE MESH SETTINS
            //main body control
            SurfaceCustomMeshControl surfSurfControl = tmpDCS.getSurfMeshCustomControl();
            tmpDCS.setSurfMeshCustomControl(surfSurfControl);
            MeshOpTool.surfFilter(      surfSurfControl,nDigitsToFilter);
            MeshOpTool.surfCustTargSize(surfSurfControl,"Relative",foilTargetPct);
            MeshOpTool.surfCustMinSize( surfSurfControl,"Relative",foilMinPct);
            MeshOpTool.surfNCircle(     surfSurfControl,foilNPtsCircle);

            //trailing edges
            SurfaceCustomMeshControl surfSurfControl_TE = tmpDCS.getSurfMeshCustomTEControl();
            tmpDCS.setSurfMeshCustomTEControl(surfSurfControl_TE);
            MeshOpTool.surfFilter_TE(  surfSurfControl_TE,nDigitsToFilter);
            MeshOpTool.surfCustMinSize(surfSurfControl_TE,"Relative",foilTEMinPct);
            MeshOpTool.surfEdgeProx(   surfSurfControl_TE,nDigitsToFilter);

            //VOLUME MESH SETTINGS
            // Defaults
            int nPrismLayers = 24; //use even #s
            double prismThickness=lengthScale*0.05;
            double firstCellThick=lengthScale*1e-5;

            //Custom volume control settings
            SurfaceCustomMeshControl volSurfControl= tmpDCS.getVolMeshCustomControl();
            tmpDCS.setVolMeshCustomControl(volSurfControl);
            //main body control
            MeshOpTool.surfFilter(volSurfControl,nDigitsToFilter);
            MeshOpTool.surfPrismThick( volSurfControl,"Absolute",prismThickness);
            MeshOpTool.surfPrismNearWall( volSurfControl,firstCellThick);
            MeshOpTool.surfPrismOverride( volSurfControl, true);
            MeshOpTool.surfPrismNumLayers(volSurfControl,nPrismLayers/2);
            tmpDCS.setVolMeshCustomControl(volSurfControl);
            //trailing edges
            SurfaceCustomMeshControl volSurfControl_TE = tmpDCS.getVolMeshCustomTEControl();
            MeshOpTool.surfFilter_TE(  volSurfControl_TE,nDigitsToFilter);
            MeshOpTool.surfPrismThick( volSurfControl_TE,"Absolute",prismThickness*0.25);
            MeshOpTool.surfPrismNearWall( volSurfControl_TE,firstCellThick*4.0);
            MeshOpTool.surfPrismOverride( volSurfControl_TE, true);
            MeshOpTool.surfPrismNumLayers(volSurfControl_TE,nPrismLayers/2);
            tmpDCS.setVolMeshCustomTEControl(volSurfControl_TE);
        }
    }

    //PHYSICS
    public void initPhysics(String physName,String physModel){
        simu.println("WT MSG: Initializing physics continuum");
//        if(!wtPhysics.doesPhysicsExist()){
            wtPhysics= new CFD_Physics(simu,physName);
            //Physics model selection
            PhysicsContinuum tmpPhys=wtPhysics.getContinuum();
            double wallDistToFS = wtPhysics.getWallDistanceToFreeStream();
            if(physModel.equals("kwSST")){
                simu.println("WT MSG: ... kwSST Model Selected.");
                isUns=false;
                CFD_Physics.set_RANS_KwSST(tmpPhys,true, true, true,false, false,wallDistToFS);
                //Physics Solver Best Practice Settings
                CFD_Physics.setKWSSTURF(simu,0.6);
                CFD_Physics.setKWSSTViscosity(simu,0.8,1.0E20);
            }else if(physModel.equals("kwSSTGRT")){
                simu.println("WT MSG: ... kwSST GRT Model Selected.");
                isUns=false;
                CFD_Physics.set_RANS_KwSST(tmpPhys,true, true, true,false, true,wallDistToFS);
                //Physics Solver Best Practice Settings
                CFD_Physics.setKWSSTURF(simu,0.6);
                CFD_Physics.setKWSSTViscosity(simu,0.8,1.0E20);
            }else if(physModel.equals("kwSSTGRTDES")){
                simu.println("WT MSG: ... kwSST GRT DES Model Selected.");
                isUns=true;
                CFD_Physics.set_DES_KwSST(tmpPhys,true, true, true,false, true,wallDistToFS);
                //set up time step information
                SolverDriver.set2ndOrderTimeDisc(simu);
            }

            //Apply to all applicable regions
            Collection<Region> allRegs=simu.getRegionManager().getObjects();
            for(Region tmpReg:allRegs){
                tmpReg.setPhysicsContinuum(wtPhysics.getContinuum());
            }         
//        }

    }
    public void setInitialConditions(){
        CFD_Physics.set_VelocityIC(wtPhysics.getContinuum(),inletCsys,refVel,lengthScale*.01,lengthScale*0.10);
        double refPress=wtPhysics.getContinuum().getReferenceValues()
                .get(ReferencePressure.class).getInternalValue(); //Pa
        CFD_Physics.set_PressureIC(wtPhysics.getContinuum(),refVel,refPress,lengthScale*0.10);
        CFD_Physics.set_TemperatureIC(wtPhysics.getContinuum(),refPress, refRho);
    }

    //REGIONS
    private GeometryPart getRegionPart(Region tmpReg){
        Collection<GeometryPart> allRegPart = tmpReg.getPartGroup().getObjects();
        return(GeometryPart) allRegPart.toArray()[0];
    }
    private Collection<Region> getRegions(){
        return simu.getRegionManager().getRegions();
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
    private void cleanupDefaultBoundary(Collection<Region> allRegs){
        for(Region tmp:allRegs){
            String regName =tmp.getPresentationName();
            try{
                Boundary defaultBndy = tmp.getBoundaryManager()
                        .getObject("Default");
                try{
                    simu.println("FC MSG: Removing Boundary \"Default\" in "+regName+".");
                    tmp.getBoundaryManager().remove(defaultBndy);
                }catch(NeoException e){
                    simu.println("FC MSG: Boundary \"Default\" has a mesh!!!");
                }
            }catch(NeoException e){
                simu.println("FC MSG: There is no Boundary \"Default\".");
            }
        }
    }

    //
    //POST PROCESSING
    public void setPostProc(){
        int nPlotSamples=(int) 1.e5;
        int itMonFreq = 1;
        int itMonStart= 1;
        //
        //DERIVED PART SETUP
        //
        simu.println("WT MSG: Generating Derived Parts.");
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
                .get(ReferencePressure.class).getInternalValue();
        SimTool.setPressureCoefficientFF(simu,refRho,0.0,refVel);

        // skin friction coefficient
        SimTool.setSkinFrictionCoefficientFF(simu,refRho,refVel);

        //
        //REPORTS & MONITORS
        //
        simu.println("WT MSG: Setting residual monitors to absolute.");
        //setResidualsToAbsolute();
        
        simu.println("WT MSG: Generating Reports & Monitors for Objects.");
        //Any Airfoils
        //REPORTS
        //
        //
        //Need some configuration Total here
        //
        //
        for(AerodynamicSurface tmpSurf:allAerodynamicSurfaces){
            tmpSurf.makeReports(simu);
            tmpSurf.makeIterationMonitors(simu,nPlotSamples,itMonFreq,itMonStart);
        }
        for(DeflectableControlSurface tmpSurf:allDeflectableControlSurfaces){
            tmpSurf.makeReports(simu);
            tmpSurf.makeIterationMonitors(simu,nPlotSamples,itMonFreq,itMonStart);
        }
        //
        //PLOTS
        //
        //
        // MONITOR Plots
        MonitorPlot tmpPlot;
        
        // Adjust residual plots
        PlotTool.adjustResidualPlot(simu,false);

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
        retPart.setMode(IsoMode.ISOVALUE_SINGLE);
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
