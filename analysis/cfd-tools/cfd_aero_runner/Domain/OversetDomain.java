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

import java.util.ArrayList;
import java.util.Collection;

import star.common.*;
import star.base.neo.*;

import GeometricObject.*;

public class OversetDomain extends Domain{
    
  //Extension of string names
  String oversetStr;

  //Extension of Overset Boundaries
  ArrayList<Boundary> oversetBoundaries;

  //Tracking Domains that the Oveset is interfaced with
  ArrayList<Domain> interfacedWithDomains=new ArrayList();

  //Track what Objects are inside this domain
  ArrayList<AerodynamicSurface> airfoilObjects=new ArrayList();

  //Coordinate system
  CartesianCoordinateSystem localCsys;

  //Overset Domain info
  double domainAngle=0.0;
  Tag primaryOSDomainTag;
    
    
  public OversetDomain(Simulation tmpSim, GeometryPart geomPart){
    super(tmpSim,geomPart);

    //Identification
    this.oversetStr = globalNames.getOversetStr();
  }

  //GEOMETRY & OBJECTS
  public void trackAirfoilObject(AerodynamicSurface tmpObj){
      airfoilObjects.add(tmpObj);
  }
  public ArrayList<AerodynamicSurface> getAirfoilObjects(){
      return airfoilObjects;
  }
  // 
  public void setOversetCoordinateSystem(CartesianCoordinateSystem tmpCsys){
    localCsys = tmpCsys;
  }
  public void setBodyCoordinateSystem(CartesianCoordinateSystem newBodyCsys){
    bodyCsys = newBodyCsys;
  }
  public void setInletCoordinateSystem(CartesianCoordinateSystem newBodyCsys){
    inletCsys = newBodyCsys;
  }

  // INTERFACE TRACKING
  
  //REGIONS
  public IndirectRegionInterface interfaceToDomain(Domain intToDomain){
    IndirectRegionInterface oversetInterface = null;
    boolean doIcreate=true;
    String myRegName = domainRegion.getPresentationName();
    String toRegName = intToDomain.getRegion().getPresentationName();
    String newInterfaceName;
    if(intToDomain instanceof OversetDomain){
        newInterfaceName = myRegName+"_to_"+toRegName;    
    }else{
        newInterfaceName = myRegName+"_to_Background";   
    }

    domainRegion.getSimulation().println(
        "OS MSG: Interfacing with Domain "+intToDomain.getName());
    //make sure that the interface doesn't already exist
    Collection<Interface> allInterfaces = 
        domainRegion.getSimulation().getInterfaceManager().getObjects();
    for(Interface tmpInterface:allInterfaces){
      //check it is overset type
      if(tmpInterface instanceof IndirectRegionInterface){
        Region tmpReg0=tmpInterface.getRegion0();
        Region tmpReg1=tmpInterface.getRegion1();

        //check if the interface already exists
        if((tmpReg0 == domainRegion && tmpReg1 == intToDomain.getRegion())||
          (tmpReg1 == domainRegion && tmpReg0 == intToDomain.getRegion())){
          interfacedWithDomains.add(intToDomain);
          //check naming convention
          if(intToDomain instanceof OversetDomain){
            //the Region0 controls the naming convention
            if(tmpReg0==domainRegion){
              tmpInterface.setPresentationName(newInterfaceName);
            }
          }else{
            tmpInterface.setPresentationName(newInterfaceName);  
          }
          doIcreate=false;
          oversetInterface = (IndirectRegionInterface) tmpInterface;
          break;
        }
      }
    }

    //create the interface
    if(doIcreate){
      oversetInterface = 
          domainRegion.getSimulation().getInterfaceManager().createIndirectRegionInterface(
              domainRegion, intToDomain.getRegion(),
              "Overset Mesh", false);
      oversetInterface.getConditions().
          get(OversetMeshInterpolationOption.class).
              setSelected(OversetMeshInterpolationOption.Type.LINEAR);
      oversetInterface.setUseAlternateHoleCutting(true);
      if(intToDomain instanceof OversetDomain){
        oversetInterface.setPresentationName(newInterfaceName);    
      }else{
        oversetInterface.setPresentationName(newInterfaceName);   
      }
      interfacedWithDomains.add(intToDomain);
    }
    //either way, need to track the interface
    return oversetInterface;
  }
  public double getDomainAngle(){
    return domainAngle;
  }
  public void zeroOutDomainCsys(){
    double oldxValue=localCsys.getBasis0().toDoubleArray()[0];
    double newXMultiplier=1.0;
    if(oldxValue<0.0) newXMultiplier=-1.0;
    double oldyValue=localCsys.getBasis1().toDoubleArray()[1];
    double newYMultiplier=1.0;
    if(oldyValue<0.0) newYMultiplier=-1.0;
    localCsys.setBasis0(new DoubleVector(new double[] {newXMultiplier,0.0,0.0}));
    localCsys.setBasis1(new DoubleVector(new double[] {0.0,newYMultiplier,0.0}));

  }
  public double setDomainAngleFromZero(double newAngleInDeg, double[] rotationVector){
    Units units_0 = 
       domainRegion.getSimulation().getUnitsManager().getPreferredUnits(new IntVector(
          new int[] {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    Units units_1 = 
       domainRegion.getSimulation().getUnitsManager().getPreferredUnits(new IntVector(
          new int[] {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // Need to counter rotate the Region's mesh.
    double oldAngleInDeg = domainAngle;
    double oldAngleInRad = oldAngleInDeg * Math.PI / 180.0;
    domainRegion.getSimulation().println("OS: Domain " + domainName
        + " was at " + oldAngleInDeg + " deg.");

    // First, counter rotate rotate the mesh if |angle| > 0 degrees
    // so that the mesh ends up back in its original CAD Zero meshed
    // orientation.
    if(Math.abs(oldAngleInDeg) > 1.e-6){
      domainRegion.getSimulation().getRepresentationManager().rotateMesh(
          new NeoObjectVector(
              new Object[] {domainRegion}), new DoubleVector(rotationVector),
              new NeoObjectVector(new Object[] {units_1, units_1, units_1}),
              oldAngleInRad, localCsys);
    }

    // Store the new domain angle from the specified argument.
    // Note, based on the formal right hand rule this rotation for positive
    // angles requires a negative rotation about its expected axis.
    domainAngle = newAngleInDeg; 
    domainRegion.getSimulation().println("OS: Domain "
        + domainName + " new angle at " + domainAngle+" deg.");
    double angleInRads = newAngleInDeg*Math.PI/180.;

    // Reset the overset Coordinate system back to zero to allow rotation again.
    // This is allowed because the mesh is counter-rotated by the old angle
    // above so we don't need the basis to track angles at this stage.
    double oldxValue = localCsys.getBasis0().toDoubleArray()[0];
    double newXMultiplier = 1.0;
    if(oldxValue<0.0) newXMultiplier = -1.0;
    double oldyValue = localCsys.getBasis1().toDoubleArray()[1];
    double newYMultiplier = 1.0;
    if(oldyValue<0.0) newYMultiplier = -1.0;
    localCsys.setBasis0(
        new DoubleVector(new double[] {newXMultiplier, 0.0, 0.0}));
    localCsys.setBasis1(
        new DoubleVector(new double[] {0.0, newYMultiplier, 0.0}));

    // Only rotate the mesh if |angle| > 0 degrees as the CFD tool cannot handle
    // a zero rotation angle. If the angle is zero just accept the reset from
    // above.
    if(Math.abs(newAngleInDeg)>1.e-6){
        domainRegion.getSimulation().getRepresentationManager()
                .rotateMesh(new NeoObjectVector(
            new Object[] {domainRegion}), new DoubleVector(rotationVector),
            new NeoObjectVector(new Object[] {units_1, units_1, units_1}),
            -angleInRads, localCsys);
        localCsys.getLocalCoordinateSystemManager()
                .rotateLocalCoordinateSystems(
            new NeoObjectVector(new Object[] {localCsys}),
            new DoubleVector(rotationVector),
            new NeoObjectVector(new Object[] {units_1, units_1, units_1}),
            -angleInRads, localCsys);
    }

    // Let the solver know which angle you came from.
    return(oldAngleInDeg);
  }

  public double determineDomainAngle(){
    //get X orientation
    double xOrientation = localCsys.getBasis0().toDoubleArray()[0];
    double xBasisYComp = localCsys.getBasis0().toDoubleArray()[1];
    double mX=1.0;

    //get X orientation
    double yOrientation=localCsys.getBasis1().toDoubleArray()[1];
    double mY=1.0;

    if(xOrientation<0&&yOrientation>0){
      if(xBasisYComp>0){
          mX=-1.0;
      }else{
          mX=1.0;
      }
    }else if(xOrientation<0&&yOrientation<0){
      if(xBasisYComp>0){
          mX=1.0;
      }else{
          mX=-1.0;
      }
    }else if(xOrientation>0&&yOrientation>0){
      if(xBasisYComp>0){
          mX=-1.0;
      }else{
          mX=1.0;
      }
    }else if(xOrientation>0&&yOrientation<0){
      if(xBasisYComp>0){
          mX=1.0;
      }else{
          mX=-1.0;
      }
    }else{
      mX=0.0;
    }

    domainAngle = mX * Math.abs( Math.asin(xBasisYComp) ) * 180.0 / Math.PI;
    domainRegion.getSimulation().println(
        "OS MSG: "+domainName+" angle detected is "+domainAngle+" degrees");

    return domainAngle;
  }

  public void setPrimaryDomainTag(Tag newTag){
    primaryOSDomainTag = newTag;
  }
  public Tag getPrimaryDomainTag(){
    return primaryOSDomainTag;
  }
}
