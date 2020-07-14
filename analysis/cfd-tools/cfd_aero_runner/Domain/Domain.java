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
package Domain;

import Naming.*;
import Tools.SimTool;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import star.energy.StaticTemperatureProfile;
import star.flow.FlowDirectionOption;
import star.common.*;
import star.base.neo.*;
import star.flow.FlowDirectionProfile;
import star.flow.VelocityMagnitudeProfile;
import star.flow.*;

import star.turbulence.*;
import star.kwturb.*;

import star.meshing.*;

public class Domain {

  //
  int verbosity = 0;
  
  //Naming convention for prefix, postfix, boundary names, etc.
  NamingConvention globalNames;

  //Identification
  String domainName;
  String inletStr;
  String switchStr;
  String outletStr;
  String freestreamStr;
  String symmetryStr;
  String wallStr;
  String oversetStr;
  String slidingStr;
  ArrayList<String> allDomStr;

  //Geometry
  GeometryPart domainPart;
  Collection<PartSurface> domainSurfs;
  Collection<String> domainSurfNames;

  //Automated Mesh Operations
  AutoMeshOperation domainSurfMeshOp;
  AutoMeshOperation domainVolumeMeshOp;

  //Region
  Region domainRegion;

  // Reference Physics Values
  double refRho;
  double refVel;
  double refTemp;
  double refMu;
  double refMa;
  
  //Boundary lists
  ArrayList<Boundary> allBoundaries;
  ArrayList<Boundary> symmBoundaries;
  ArrayList<Boundary> wallBoundaries;
  ArrayList<Boundary> inletBoundaries;
  ArrayList<Boundary> outletBoundaries;
  ArrayList<Boundary> fsBoundaries;

  // Coordinate Systems
  LabCoordinateSystem labCsys;
  CartesianCoordinateSystem bodyCsys;
  CoordinateSystem inletCsys;
  CoordinateSystem windCsys;

  String bodyCsysName ;
  String inletCsysName;
  String windCsysName ;

  Representation proxyRep;

  public Domain(Simulation tmpSim,GeometryPart geomPart){
      //Load global naming conventions
      this.globalNames = new NamingConvention();
      this.inletStr       = globalNames.getInletStr();
      this.outletStr      = globalNames.getOutletStr();
      this.freestreamStr  = globalNames.getFreeStr();
      this.symmetryStr    = globalNames.getSymmStr();
      this.switchStr      = globalNames.getSwitchStr();
      this.wallStr        = globalNames.getWallStr();
      this.oversetStr     = globalNames.getOversetStr();
      this.slidingStr     = globalNames.getSlidingStr();

      allDomStr =new ArrayList();
      allDomStr.add(inletStr);      allDomStr.add(outletStr);
      allDomStr.add(freestreamStr); allDomStr.add(switchStr);
      allDomStr.add(symmetryStr);   allDomStr.add(wallStr);
      allDomStr.add(oversetStr);    allDomStr.add(slidingStr);

      //Names, parts, places
      this.domainPart = geomPart;
      this.domainName = geomPart.getPresentationName();
      this.domainSurfs = geomPart.getPartSurfaces();

      //Region/boundary stuff
      allBoundaries    = new ArrayList();
      symmBoundaries   = new ArrayList();
      wallBoundaries   = new ArrayList();
      inletBoundaries  = new ArrayList();
      outletBoundaries = new ArrayList();
      fsBoundaries     = new ArrayList();

      int preFixLen = inletStr.length();
      removeEmptyDomainStr(domainPart, preFixLen);

      //Coordinate System Stuff
      this.bodyCsysName  = globalNames.getBodyCsysName();
      this.inletCsysName = globalNames.getInletCsysName();
      this.windCsysName  = globalNames.getWindCsysName();

      this.proxyRep = SimTool.getSimProxy(tmpSim, "Proxy");
      this.labCsys=tmpSim.getCoordinateSystemManager()
              .getLabCoordinateSystem();
      try{
        this.bodyCsys=(CartesianCoordinateSystem) labCsys
            .getCoordinateSystemManager().getCoordinateSystem(bodyCsysName);
        this.inletCsys=bodyCsys.getLocalCoordinateSystemManager().
            getObject(inletCsysName);
        try{
          this.windCsys = bodyCsys.getLocalCoordinateSystemManager().
              getObject(windCsysName);
        }catch(NeoException e){
            tmpSim.println("Domain MSG: No wind Axis detected.");
        }
      }catch(NeoException e){
        tmpSim.println("Domain MSG: No body coordinate system found");
        tmpSim.println("Domain MSG: No inlet coordinate system found");
      }
      tmpSim.println("Domain MSG: successfully created Domain from "
          + domainName);
  }

    
  //Geometry
  public String getName(){
        return domainName;
    }
  public GeometryPart getDomainPart(){
        return domainPart;
    }
    
  //Mesh Operation
  public AutoMeshOperation getDomainSurfMeshOp(){
    return domainSurfMeshOp;
  }
  public AutoMeshOperation getDomainVolumeMeshOp(){
    return domainVolumeMeshOp;
  }
  public void setDomainSurfMeshOp(AutoMeshOperation newOp){
    domainSurfMeshOp=newOp;
  }
  public void setDomainVolumeMeshOp(AutoMeshOperation newOp){
    domainVolumeMeshOp=newOp;
  }
  public void updateMeshBaseSize(double newBaseSize){
    double oldSurfBaseSize = domainSurfMeshOp.getDefaultValues()
        .get(BaseSize.class).getInternalValue();
    double oldVolumeBaseSize = domainVolumeMeshOp.getDefaultValues()
        .get(BaseSize.class).getInternalValue();

    if(Math.abs(newBaseSize-oldSurfBaseSize)>1.e-5){
        domainSurfMeshOp.getDefaultValues()
            .get(BaseSize.class).setInternalValue(newBaseSize);
    }
    if(Math.abs(newBaseSize-oldVolumeBaseSize)>1.e-5){
        domainVolumeMeshOp.getDefaultValues()
            .get(BaseSize.class).setInternalValue(newBaseSize);
    }        
  }

  //Physics
  public void setRefRho(double newVal){
      refRho = newVal;
    }
  public double getRefRho(){
      return refRho;
    }
  public void setRefVel(double newVal){
    refVel = newVal;
  }
  public double getRefVel(){
    return refVel;
  }
  public void setRefTemp(double newVal){
    refTemp = newVal;
  }
  public double getRefTemp(){
    return refTemp;
  }
  public void setRefMu(double newVal){
    refMu = newVal;
  }
  public double getRefMu(){
    return refMu;
  }
  public void setRefMa(double newVal){
    refMa = newVal;
  }
  public double getRefMa(){
    return refMa;
  }
  public boolean isFreeStreamDomain(){
    if(inletBoundaries.size() > 0 && outletBoundaries.size() > 0){
        return false;
    }else{
        return true;
    }
  }

  //Regions
  public Region getRegion(){
      return domainRegion;
  }
  public void setUpRegion(Simulation tmpSim){
      //Check if region exists
      try{
          domainRegion = tmpSim.getRegionManager().getObject(domainName);
      }catch(NeoException e){
          domainRegion = tmpSim.getRegionManager().createEmptyRegion();
          domainRegion.setPresentationName(domainName);
      }
      //Assign domain part to Region
      domainRegion.getPartGroup().setObjects(domainPart);
  }
  public void setUpRegion(Simulation tmpSim, String withThisName){
    //Check if region exists
    try{
        domainRegion = tmpSim.getRegionManager().getObject(withThisName);
    }catch(NeoException e){
        domainRegion = tmpSim.getRegionManager().createEmptyRegion();
        domainRegion.setPresentationName(withThisName);
    }
    //Assign domain part to Region
    domainRegion.getPartGroup().setObjects(domainPart);
  }
  public Region getExistingRegion(Simulation tmpSim, String withThisName){
      try{
          domainRegion = tmpSim.getRegionManager().getObject(withThisName);
      }catch(NeoException e){
          domainRegion = tmpSim.getRegionManager().createEmptyRegion();
          domainRegion.setPresentationName(withThisName);
      }
      return domainRegion;
  }

  public Boundary setUpBoundary(String bndyName){
      Boundary tmpBndy;
      try{
          tmpBndy = domainRegion.getBoundaryManager().getBoundary(bndyName);
      }catch(NeoException e){
          tmpBndy = 
              domainRegion.getBoundaryManager().createEmptyBoundary(bndyName);
      }
      if(!allBoundaries.contains(tmpBndy)) allBoundaries.add(tmpBndy);
      return tmpBndy;
  }
  public void initPartBoundaries(){
      for(String tmpBndyID:allDomStr){
          //create boundary
          Boundary tmpBndy = setUpBoundary(globalNames.getBndyName(tmpBndyID));
          String bndyName = tmpBndy.getPresentationName();
          //add necessary surfaces to the boundary
          ArrayList<PartSurface> tmpSurfList=new ArrayList();
          for(PartSurface tmpSurf:domainSurfs){
              String tmpName=tmpSurf.getPresentationName();
              if(  tmpName.startsWith(tmpBndyID)
                 ||tmpName.contains("."+tmpBndyID)){
                  tmpSurfList.add(tmpSurf);
              }
          }
          tmpBndy.getPartSurfaceGroup().setObjects(tmpSurfList);

          //Set BC Type
          if(tmpBndyID.equals(inletStr)){
              tmpBndy.setBoundaryType(InletBoundary.class);
          }else if(tmpBndyID.equals(outletStr)){
              tmpBndy.setBoundaryType(PressureBoundary.class);
          }else if(tmpBndyID.equals(switchStr)){
              tmpBndy.setBoundaryType(InletBoundary.class);
          }else if(tmpBndyID.equals(symmetryStr)){
              tmpBndy.setBoundaryType(SymmetryBoundary.class);
          }else if(tmpBndyID.equals(freestreamStr)){
              tmpBndy.setBoundaryType(FreeStreamBoundary.class);
          }else if(tmpBndyID.equals(oversetStr)){
              tmpBndy.setBoundaryType(OversetMeshBoundary.class);
          }else if(tmpBndyID.equals(slidingStr)){
              tmpBndy.setBoundaryType(SymmetryBoundary.class);
          }
      }
  }
public void getExistingDomainBoundaries(){
  //Collect Inlet Boundaries
  for(Boundary tmp:domainRegion.getBoundaryManager().getBoundaries()){
    //Inlets
    if(tmp.getBoundaryType() instanceof InletBoundary){
      if(!inletBoundaries.contains(tmp)) inletBoundaries.add(tmp);
    }
    //Outlets
    if(tmp.getBoundaryType() instanceof PressureBoundary){
      if(!outletBoundaries.contains(tmp)) outletBoundaries.add(tmp);
    }
    //Freestream
    if(tmp.getBoundaryType() instanceof FreeStreamBoundary){
      if(!fsBoundaries.contains(tmp)) fsBoundaries.add(tmp);
    }
  }
}
public void setAllInletBCs(double refVel,double refTemp,double refMa,double fsTVR,double fsTi){
  for(Boundary tmpBndy : inletBoundaries){
    vInletBC(tmpBndy, refVel, refTemp, fsTVR, fsTi);
  }
  for(Boundary tmpBndy:fsBoundaries){
    fsInletBC(tmpBndy, refMa, refTemp, fsTVR, fsTi);
  }
}
  public void setAllOutletBCs(double fsPress,double refTemp,double fsTVR,double fsTi){
      for(Boundary tmpBndy:outletBoundaries){
          pressOutBC(tmpBndy, fsPress,refTemp,fsTVR, fsTi);
      }
  }

  public Boundary getAnInletBoundary(){
      Boundary tmpBndy;
      if(isFreeStreamDomain()){
          tmpBndy=fsBoundaries.get(0);
      }else{
          tmpBndy=inletBoundaries.get(0);
      }
      return tmpBndy;
  }

  public boolean hasFSBoundaries(){
    // full object
    if(fsBoundaries.size()>0){
      return true;
    }
    // before fully completed object built
    for(PartSurface tmpSurf:domainPart.getPartSurfaces()){
      if (tmpSurf.getPresentationName().startsWith(globalNames.getFreeStr())
        ||tmpSurf.getPresentationName().contains("."+globalNames.getFreeStr())){
        return true;
      }
    }
    return false;
  }


  public void setFSBoundariesToKAndOmega(double tkeValue, double omegaValue){
    for(Boundary tmpBndy:fsBoundaries){
      changeBoundaryToKandOmega(tmpBndy, tkeValue, omegaValue);
    }
  } 

  public static void changeBoundaryToKandOmega(Boundary tmpBndy, double tkeValue,double omegaValue){
    tmpBndy.getConditions().get(KwTurbSpecOption.class).setSelected(KwTurbSpecOption.Type.K_OMEGA);
    SpecificDissipationRateProfile specificDissipationRateProfile_1 = 
      tmpBndy.getValues().get(SpecificDissipationRateProfile.class);
    specificDissipationRateProfile_1.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tkeValue);
    TurbulentKineticEnergyProfile turbulentKineticEnergyProfile_1 = 
      tmpBndy.getValues().get(TurbulentKineticEnergyProfile.class);
    turbulentKineticEnergyProfile_1.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(omegaValue);
  }

  public void switchInAndOutBCToFreeStream(double machIn, double tempIn, double tVR, double tI){
    for(Boundary tmpB:inletBoundaries){
      fsInletBC(tmpB);
      fsInletBC(tmpB,machIn,tempIn,tVR,tI);
    }
    for(Boundary tmpB:outletBoundaries){
      fsInletBC(tmpB);
      fsInletBC(tmpB,machIn,tempIn,tVR,tI);
    }
    fsBoundaries.addAll(inletBoundaries);
    fsBoundaries.addAll(outletBoundaries);
    inletBoundaries.clear();
    outletBoundaries.clear();
  }
  
  //PRIVATE METHODS
    
//BOUNDARY CONDITIONS
private void vInletBC(Boundary tmpBndy, double vMag, double tempIn, double tVR, double tI){
  
  // Set Inlet to follow components
  tmpBndy.getConditions().get(FlowDirectionOption.class)
      .setSelected(FlowDirectionOption.Type.COMPONENTS);
  FlowDirectionProfile flowDirectionProfile = 
    tmpBndy.getValues().get(FlowDirectionProfile.class);
  flowDirectionProfile.setCoordinateSystem(this.getInletCsys());

  //Set velocity at inlet
  VelocityMagnitudeProfile vMP = 
    tmpBndy.getValues().get(VelocityMagnitudeProfile.class);
  vMP.setMethod(ConstantScalarProfileMethod.class);
  vMP.getMethod(ConstantScalarProfileMethod.class)
      .getQuantity().setValue(vMag);

  //Set temperature at inlet
  StaticTemperatureProfile sTP = 
    tmpBndy.getValues().get(StaticTemperatureProfile.class);
  sTP.getMethod(ConstantScalarProfileMethod.class)
      .getQuantity().setValue(tempIn);

  // Turbulence Stuff
  TurbulenceIntensityProfile tIP = 
      tmpBndy.getValues().get(TurbulenceIntensityProfile.class);
  tIP.getMethod(ConstantScalarProfileMethod.class)
      .getQuantity().setValue(tI);
  TurbulentViscosityRatioProfile tVRP = 
      tmpBndy.getValues().get(TurbulentViscosityRatioProfile.class);
  tVRP.getMethod(ConstantScalarProfileMethod.class)
      .getQuantity().setValue(tVR);
}
  private void fsInletBC(Boundary tmpBndy, double machIn, double tempIn, double tVR, double tI){
      tmpBndy.getConditions().get(FlowDirectionOption.class).setSelected(FlowDirectionOption.Type.COMPONENTS);
      FlowDirectionProfile flowDirectionProfile = 
        tmpBndy.getValues().get(FlowDirectionProfile.class);
      flowDirectionProfile.setCoordinateSystem(inletCsys);
      //Set Mach at inlet
      MachNumberProfile machNumberProfile_0 = 
        tmpBndy.getValues().get(MachNumberProfile.class);
      machNumberProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(machIn);
      //Temperature at inlet
      StaticTemperatureProfile staticTemperatureProfile_0 = 
        tmpBndy.getValues().get(StaticTemperatureProfile.class);
      staticTemperatureProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tempIn);
      try{
        //Ti at Inlet
        TurbulenceIntensityProfile turbulenceIntensityProfile_0 = 
          tmpBndy.getValues().get(TurbulenceIntensityProfile.class);
        turbulenceIntensityProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tI);
        //TVR at Inlet
        TurbulentViscosityRatioProfile turbulentViscosityRatioProfile_0 = 
          tmpBndy.getValues().get(TurbulentViscosityRatioProfile.class);
        turbulentViscosityRatioProfile_0.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tVR);
      }catch(NeoException e){
        // TODO: Break this function into two.
        setFSBoundariesToKAndOmega(tI,tVR); //note the swap for the order of variables
      }
  }
  private static void pressOutBC(Boundary tmpBndy, double pVal, double outTemp, double tVR, double tI){
      //make sure its a pressure boundary
      tmpBndy.setBoundaryType(PressureBoundary.class);
      //set pressure at outlet
      StaticPressureProfile sPP = tmpBndy.getValues().get(StaticPressureProfile.class);
      sPP.setMethod(ConstantScalarProfileMethod.class);
      tmpBndy.getValues().get(StaticPressureProfile.class).setValue(pVal);
      //set temperature at outlet
      StaticTemperatureProfile sTP = 
        tmpBndy.getValues().get(StaticTemperatureProfile.class);
      sTP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(outTemp);
      //Turbulent intensity
      TurbulenceIntensityProfile tIP = 
          tmpBndy.getValues().get(TurbulenceIntensityProfile.class);
      tIP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tI);
      TurbulentViscosityRatioProfile tVRP = 
          tmpBndy.getValues().get(TurbulentViscosityRatioProfile.class);
      tVRP.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(tVR);
  }
  private static void setWallBC(Boundary tmpBndy){
      tmpBndy.setBoundaryType(WallBoundary.class);
  }
  private static void setSymmBC(Boundary tmpBndy){
      tmpBndy.setBoundaryType(SymmetryBoundary.class);
  }
  private static void fsInletBC(Boundary tmpBndy){
      tmpBndy.setBoundaryType(FreeStreamBoundary.class);
  }

  //Tools
  private void removeEmptyDomainStr(GeometryPart tmpPart,int preFixLen){
      //Figures out which boundaries actually exist in the domain
      ArrayList<String> retList = new ArrayList();
      for(PartSurface tmpSurf: tmpPart.getPartSurfaces()){
          String tmpName=tmpSurf.getPresentationName();
          for(String tmpStr:allDomStr){
              if(tmpName.startsWith(tmpStr)||tmpName.contains("."+tmpStr)){
                  if(!retList.contains(tmpStr)){
                      retList.add(tmpStr);
                      break;
                  }
              }
          }

      }
      allDomStr.retainAll(retList);
  }

  // BOUNDARIES
  public void cleanUpDefaultBoundaries(){
    cleanUpDefaultBoundary(Collections.singleton(domainRegion));
  }
  private void cleanUpDefaultBoundary(Collection<Region> allRegs){
    for(Region tmp:allRegs){
      String regName =tmp.getPresentationName();
      try{
        Boundary defaultBndy = tmp.getBoundaryManager()
            .getObject("Default");
        try{
          tmp.getSimulation().println("WT MSG: Removing Boundary \"Default\" in "+regName+".");
          tmp.getBoundaryManager().remove(defaultBndy);
        }catch(NeoException e){
          tmp.getSimulation().println("WT MSG: Boundary \"Default\" has a mesh!!!");
          }
      }catch(NeoException e){
          tmp.getSimulation().println("WT MSG: There is no Boundary \"Default\".");
      }
    }
  }

  
  // COORDINATE SYSTEMS
  public CartesianCoordinateSystem getBodyCsys(){
    return bodyCsys;
  }
  public CoordinateSystem getInletCsys(){
    return inletCsys;
  }
}// End of Domain.java
