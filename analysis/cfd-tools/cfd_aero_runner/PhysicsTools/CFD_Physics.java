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
package PhysicsTools;

import java.util.*;
import star.common.*;

import star.base.neo.*;
import star.coupledflow.*;
import star.energy.*;
import star.flow.*;
import star.kwturb.*;
import star.material.*;
import star.metrics.*;
import star.segregatedenergy.*;
import star.segregatedflow.*;
import star.turbulence.*;
import star.base.report.*;

import star.walldistance.*;


public class CFD_Physics {
  PhysicsContinuum cfdPhysics;
  String physName;

  // Physics constants.
  double wallDistanceToFS=0.05;

  public CFD_Physics(Simulation tmpSim, String physName){
      this.physName = physName;
      this.cfdPhysics = getPhysContinuum(tmpSim, physName);
  }

  public boolean doesPhysicsExist(){
      return !physName.isEmpty();
  }
  public void setWallDistanceToFreeStream(double newVal){
      wallDistanceToFS=newVal;
  }
  public double getWallDistanceToFreeStream(){
    return wallDistanceToFS;
  }
  public PhysicsContinuum getContinuum(){
      return cfdPhysics;
  }

  //PHYSICS CONTINUA METHODS
  public static Collection<PhysicsContinuum> getAllFluidPhysics(Simulation tmpSim){
      // Look for regions that have active fluid physics continuua
      Collection<PhysicsContinuum> retPhysics = new ArrayList();
      if(!tmpSim.getRegionManager().getRegions().isEmpty()){
          for(Region tmp:tmpSim.getRegionManager().getRegions()){
              if((tmp.getRegionType() instanceof FluidRegion)||
                      (tmp.getRegionType() instanceof PorousRegion)){
                  if(!(retPhysics.contains(tmp.getPhysicsContinuum()))){
                      retPhysics.add(tmp.getPhysicsContinuum());
                  }
              }
          }
      }
      // Look for existing fluid physics continuua in Continuums
      for(Continuum tmp:tmpSim.getContinuumManager().getObjects()){
          if(!tmp.inUse()){
               // Test to see if it already has a gas model, in which case it is
               // considered a fluid continuum.
              try{
                  SingleComponentGasModel tmpModel = 
                          tmp.getModelManager().getModel(
                                  SingleComponentGasModel.class);
                  if(!(retPhysics.contains(tmp.getPhysicsContinuum()))){
                      retPhysics.add((PhysicsContinuum) tmp);
                      tmpSim.println("Continuum "+tmp.getPresentationName()
                              +" has a single species gas model. Added.");
                  }
              }catch(NeoException e){
                  tmpSim.println("Continuum "+tmp.getPresentationName()
                          +" has no single species gas model. Skipped.");
              }
          }
      }
      // If none exist, make a fluids continua
      if(retPhysics.isEmpty()){
          PhysicsContinuum tmp = tmpSim.getContinuumManager().
                  createContinuum(PhysicsContinuum.class);
          tmp.setPresentationName("Air");
          retPhysics.add(tmp);
      }

      // If it contains a null value for some reason, remove the null
      try{
        retPhysics.remove(null);
      }catch(NeoException e){
      }
      return retPhysics;
  }
  //
  public static PhysicsContinuum set_RANS_KwSST(PhysicsContinuum tmpPhys, boolean is3D,
          boolean isSegregated,boolean isCompressible, boolean isIsoThermal,
          boolean needGRT,double wallDistToFS){
    /* Make physics continuum to use RANS kwSST, GRT optional */

    // Turn off DES if it exists
    turnOffKwSSTDES(tmpPhys);

    // Set solver settings
    turnOn2Dor3D(tmpPhys,is3D);
    turnOnSinglePhaseGas(tmpPhys);
    turnOnSolver(tmpPhys,isSegregated,isCompressible,isIsoThermal);
    turnOnUnSteady(tmpPhys,false);
    turnOnRANS(tmpPhys);
    turnOnKWSST(tmpPhys);
    if(needGRT){
        turnOnGRT(tmpPhys, wallDistToFS);
    }else{
        turnOffGRT(tmpPhys);
    }

    return tmpPhys;
  }
  public static PhysicsContinuum set_DES_KwSST(PhysicsContinuum tmpPhys,boolean is3D,
          boolean isSegregated,boolean isCompressible, boolean isIsoThermal,
          boolean needGRT,double wallDistToFS){

    /* Make physics continuum to use RANS kwSST, GRT optional */
    turnOn2Dor3D(tmpPhys,is3D);
    turnOnSinglePhaseGas(tmpPhys);
    turnOnSolver(tmpPhys,isSegregated,isCompressible,isIsoThermal);
    turnOnUnSteady(tmpPhys,true);
    turnOnKwSSTDES(tmpPhys);
    if(needGRT){
        turnOnGRT(tmpPhys,wallDistToFS);
    }else{
        turnOffGRT(tmpPhys);
    }

    return tmpPhys;
  }
  public static PhysicsContinuum set_RANS_KwSST_SGamma(PhysicsContinuum tmpPhys,boolean is3D,
          boolean isSegregated,boolean isCompressible, boolean isIsoThermal){
    // Turn off DES if it exists
    turnOffKwSSTDES(tmpPhys);
    // Set solver settings
    turnOn2Dor3D(tmpPhys,is3D);
    turnOnSinglePhaseGas(tmpPhys);
    turnOnSolver(tmpPhys,isSegregated,isCompressible,isIsoThermal);
    turnOnUnSteady(tmpPhys,false);
    turnOnRANS(tmpPhys);
    turnOnKWSST(tmpPhys);
    try{
        turnOffGRT(tmpPhys);
    }catch(NeoException e){
    }
    turnOn3EqSGamma(tmpPhys);
    return tmpPhys;
  }
  public static PhysicsContinuum set_DES_KwSST_SGamma(PhysicsContinuum tmpPhys,boolean is3D,
          boolean isSegregated,boolean isCompressible, boolean isIsoThermal){
    /* Make physics continuum to use RANS kwSST, GRT optional */
    turnOn2Dor3D(tmpPhys,is3D);
    turnOnSinglePhaseGas(tmpPhys);
    turnOnSolver(tmpPhys,isSegregated,isCompressible,isIsoThermal);
    turnOnUnSteady(tmpPhys,true);
    turnOnKwSSTDES(tmpPhys);
    turnOffGRT(tmpPhys);
    turnOn3EqSGamma(tmpPhys);
    return tmpPhys;
  } 
  //
  public static boolean isCoupledSolverUsed(PhysicsContinuum tmpPhys){
    try{
      CoupledImplicitSolver coupledImplicitSolver = 
      ((CoupledImplicitSolver) tmpPhys.getSimulation().
              getSolverManager().getSolver(CoupledImplicitSolver.class));
      return true;
    }catch(NeoException e){
      return false;
    }
  }
  public  static CoupledImplicitSolver switchToCoupledSolver(PhysicsContinuum tmpPhys, double newCFL){
    try{
      SegregatedFluidTemperatureModel segFTM = 
        tmpPhys.getModelManager().
                getModel(SegregatedFluidTemperatureModel.class);
      tmpPhys.disableModel(segFTM);
    }catch(NeoException e){

    }
    try{
      //Disable Segregated
      SegregatedFlowModel segregatedFlowModel_0 = 
        tmpPhys.getModelManager().getModel(SegregatedFlowModel.class);
      tmpPhys.disableModel(segregatedFlowModel_0);
      //Enable Coupled
      tmpPhys.enable(CoupledFlowModel.class);
      tmpPhys.enable(CoupledEnergyModel.class);
    }catch(NeoException e){

    }

    CoupledImplicitSolver coupledImplicitSolver = 
      ((CoupledImplicitSolver) tmpPhys.getSimulation().getSolverManager().
              getSolver(CoupledImplicitSolver.class));
    coupledImplicitSolver.setCFL(newCFL);
    coupledImplicitSolver.getExpertInitManager()
      .getExpertInitOption().setSelected(ExpertInitOption.Type.GRID_SEQ_METHOD);
    GridSequencingInit gridSequencingInit_0 = 
      ((GridSequencingInit) coupledImplicitSolver.
              getExpertInitManager().getInit());
    gridSequencingInit_0.setConvGSTol(0.05);

    return coupledImplicitSolver;

  }
    
    public static ExpertDriverCoupledSolver getExpertDriver(CoupledImplicitSolver tmpSolver){
      tmpSolver.getSolutionDriverManager().getExpertDriverOption().
              setSelected(ExpertDriverOption.Type.EXPERT_DRIVER);
       return ((ExpertDriverCoupledSolver) tmpSolver.
               getSolutionDriverManager().getDriver());
   }
    // turbulence URFs
    public static void setKWTurbSolverURF(Simulation tmpSim, double newValue){
      KwTurbSolver kwTurbSolver_0 = 
        ((KwTurbSolver) tmpSim.getSolverManager().getSolver(KwTurbSolver.class));
      kwTurbSolver_0.setUrf(newValue);
    }
    public static void setKWTurbViscositySolverURF(Simulation tmpSim, double newValue){
      KwTurbViscositySolver kwTurbViscositySolver_0 = 
        ((KwTurbViscositySolver) tmpSim.getSolverManager().
                getSolver(KwTurbViscositySolver.class));
      kwTurbViscositySolver_0.setViscosityUrf(newValue);
    }
    public static void setKWSSTConstitutiveToQCR(PhysicsContinuum tmpPhys){
      SstKwTurbModel sstKwTurbModel_0 = 
        tmpPhys.getModelManager().getModel(SstKwTurbModel.class);
      sstKwTurbModel_0.getKwTurbConstitutiveOption().
              setSelected(KwTurbConstitutiveOption.Type.QCR);
    }
    public static void setKWSSTa1Coef(PhysicsContinuum tmpPhys, double newA1){
      SstKwTurbModel sstKwTurbModel_0 = 
        tmpPhys.getModelManager().getModel(SstKwTurbModel.class);
      sstKwTurbModel_0.setA1(0.31);
    }
    //
    public static void setTurbulenceToKandOmega(PhysicsContinuum tmpPhys, double newTKE, double newOmega){
      tmpPhys.getInitialConditions().get(KwTurbSpecOption.class)
        .setSelected(KwTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
      tmpPhys.getInitialConditions().get(KwTurbSpecOption.class).
              setSelected(KwTurbSpecOption.Type.K_OMEGA);
      TurbulentKineticEnergyProfile turbulentKineticEnergyProfile_0 = 
        tmpPhys.getInitialConditions().get(TurbulentKineticEnergyProfile.class);
      turbulentKineticEnergyProfile_0.
              getMethod(ConstantScalarProfileMethod.class).getQuantity().
              setValue(newTKE);
      SpecificDissipationRateProfile specificDissipationRateProfile_0 = 
        tmpPhys.getInitialConditions().get(SpecificDissipationRateProfile.class);
      specificDissipationRateProfile_0.
              getMethod(ConstantScalarProfileMethod.class).getQuantity().
              setValue(newOmega);
    }
    //
    public static void setBLTurbulenceInitialization(Simulation tmpSim, boolean useBLInit){
      KwTurbSolver kwTurbSolver_0 = 
        ((KwTurbSolver) tmpSim.getSolverManager().getSolver(KwTurbSolver.class));
      kwTurbSolver_0.setTurbulenceInitialization(useBLInit);
    }
    public static void useAUSMPlus(PhysicsContinuum tmpCont){
      CoupledFlowModel coupledFlowModel_0 = 
        tmpCont.getModelManager().getModel(CoupledFlowModel.class);
      coupledFlowModel_0.getCoupledInviscidFluxOption().
              setSelected(CoupledInviscidFluxOption.Type.AUSM_SCHEME);
    }

  private static PhysicsContinuum getPhysContinuum(Simulation tmpSim, String physName){
    try{
      return ((PhysicsContinuum) tmpSim.getContinuumManager().
              getContinuum(physName));
    }catch(NeoException e){
      PhysicsContinuum tmp = tmpSim.getContinuumManager().
              createContinuum(PhysicsContinuum.class);
      tmp.setPresentationName(physName);
      return tmp;
    }
  }
    public static boolean is2D(PhysicsContinuum tmpPhys){
        try{
            tmpPhys.getModelManager().getModel(TwoDimensionalModel.class);
            return true;
        }catch(NeoException e){
            return false;            
        }
    }
    public static double getMu(PhysicsContinuum tmpPhys){
      SingleComponentGasModel sCGM = 
        tmpPhys.getModelManager().getModel(SingleComponentGasModel.class);
      Gas gas_0 = ((Gas) sCGM.getMaterial());
      ConstantMaterialPropertyMethod cMPM = 
        ((ConstantMaterialPropertyMethod) gas_0
                .getMaterialProperties()
                .getMaterialProperty(DynamicViscosityProperty.class)
                .getMethod());
      return cMPM.getQuantity().getRawValue();
    }
    public static void updateMu(PhysicsContinuum tmpPhys,double newValue){
      SingleComponentGasModel sCGM = 
        tmpPhys.getModelManager().getModel(SingleComponentGasModel.class);
      Gas gas_0 = ((Gas) sCGM.getMaterial());
      ConstantMaterialPropertyMethod cMPM = 
        ((ConstantMaterialPropertyMethod) gas_0
                .getMaterialProperties()
                .getMaterialProperty(DynamicViscosityProperty.class)
                .getMethod());
      cMPM.getQuantity().setValue(newValue);
    }
    public static double getReferencePressure(PhysicsContinuum tmpPhys){
        return tmpPhys.getReferenceValues().get(ReferencePressure.class).
                getRawValue();
    }
    
    // CONTINUUM REFERENCE VALUES
    public static void setMinimumReferenceWallDistance(PhysicsContinuum tmpPhys,
            double newVal){
        tmpPhys.getReferenceValues()
                .get(MinimumAllowableWallDistance.class).setValue(newVal);
    }
    public static void setMinimumAllowableTemperature(PhysicsContinuum tmpPhys,
            double newVal){
        tmpPhys.getReferenceValues()
                .get(MinimumAllowableTemperature.class).setValue(newVal);
    }
    public static void setMaximumAllowableTemperature(PhysicsContinuum tmpPhys,
            double newVal){
        tmpPhys.getReferenceValues()
                .get(MaximumAllowableTemperature.class).setValue(800.0);
    }
    
    //SOLVER METHODS
    public static boolean isUnsteady(Collection<PhysicsContinuum> tmpPhysColl){
      for(PhysicsContinuum tmpPhys:tmpPhysColl){
        // Test to see if it already has a steady state model.
        try{
            SteadyModel tmpModel = tmpPhys.getModelManager().
                    getModel(SteadyModel.class);
            return false;
        // No steady model detected.
        }catch(NeoException e){
        }
      }
      return true;
    }
    public static boolean isUnsteady(PhysicsContinuum tmpPhys){
        // Test to see if it already has an steady state model.
        try{
            SteadyModel tmpModel = tmpPhys.getModelManager().
                    getModel(SteadyModel.class);
            return false;
        // No unsteady model detected.
        }catch(NeoException e){
            return true;
        }
    }
    public static boolean isUnsteady(Simulation tmpSim){
        // Test to see if it already has an unsteady model.
        try{
            tmpSim.println("Checking unsteady Solver");
            Solver tmpSolver = tmpSim.getSolverManager().
                    getObject("Implicit Unsteady");
            return true;
        // No unsteady model detected.
        }catch(NeoException e){
            tmpSim.println("No unsteady solver detected");
            return false;
        }
    }
    private static void turnOn2Dor3D(PhysicsContinuum tmpPhys, boolean is3D){
        // Check for 2D or 3D Physics, otherwise make it so.
        if(is3D){
            try{ //3D
                ThreeDimensionalModel tmpModel = tmpPhys.getModelManager().
                        getModel(ThreeDimensionalModel.class);
            }catch(NeoException e){
                tmpPhys.enable(ThreeDimensionalModel.class);
            }
        }else{
            try{ //2D
                TwoDimensionalModel tmpModel = tmpPhys.getModelManager().
                        getModel(TwoDimensionalModel.class);
            }catch(NeoException e){
                tmpPhys.enable(TwoDimensionalModel.class);
            }
        }
    }
    private static void turnOnSinglePhaseGas(PhysicsContinuum tmpPhys){
        try{
            SingleComponentGasModel singleComponentGasModel_1 = 
              tmpPhys.getModelManager().getModel(SingleComponentGasModel.class);
        }catch(NeoException e){
            tmpPhys.enable(SingleComponentGasModel.class);
        }
    }
    private static void turnOnSolver(PhysicsContinuum tmpPhys,
            boolean isSegregated, boolean isCompressible, boolean isIsoThermal){
        if(isSegregated){ //SEGREGATED SOLVER
          try{// check no coupled model exists
              CoupledFlowModel coupledFlowModel_0 = 
                tmpPhys.getModelManager().getModel(CoupledFlowModel.class);
              tmpPhys.disableModel(coupledFlowModel_0);
              if(isCompressible){
                  CoupledEnergyModel tmpModel 
                          = tmpPhys.getModelManager().
                                  getModel(CoupledEnergyModel.class);
                  tmpPhys.disableModel(tmpModel);
              }
          }catch(NeoException e){
              //Unstack Coupled Solver
          }
          try{ //prexisting segregated solver
              SegregatedFlowModel tmpModel = 
                  tmpPhys.getModelManager().getModel(SegregatedFlowModel.class);
          }catch(NeoException e1){
              tmpPhys.enable(SegregatedFlowModel.class);
          }
          // THERMAL OPTIONS
          if(isCompressible){ 
            try{
                IdealGasModel tmpModel = 
                tmpPhys.getModelManager().getModel(IdealGasModel.class);
            }catch(NeoException e){
              try{ // Check to see if it is set to constant density.
                ConstantDensityModel tmpModel 
                          = tmpPhys.getModelManager()
                                  .getModel(ConstantDensityModel.class);
                tmpPhys.disableModel(tmpModel);
              }catch(NeoException ee){
              }
              tmpPhys.enable(IdealGasModel.class);
            }
            if(isIsoThermal){
                // Set IsoThermal.
            }else{ // Set ideal gas.
                try{
                    SegregatedFluidTemperatureModel tmpModel = 
                      tmpPhys.getModelManager().
                              getModel(SegregatedFluidTemperatureModel.class);
                }catch(NeoException e){
                    tmpPhys.enable(SegregatedFluidTemperatureModel.class);
                }
            }
          }else{
            Simulation tmpSim = tmpPhys.getSimulation();
            try{ // Disable ideal gas if it is on.
              IdealGasModel idealGasMode = 
                tmpPhys.getModelManager().getModel(IdealGasModel.class);
                tmpPhys.disableModel(idealGasMode);
                SegregatedFluidTemperatureModel segregatedFluidTemperatureModel_0 = 
                tmpPhys.getModelManager().
                        getModel(SegregatedFluidTemperatureModel.class);
                tmpPhys.disableModel(segregatedFluidTemperatureModel_0);
            }catch(NeoException e){
              
            }
            double constantDensityGasValue = 1.176;
            tmpPhys.enable(ConstantDensityModel.class);
            SingleComponentGasModel singleComponentGasModel_0 = 
              tmpPhys.getModelManager().
                      getModel(SingleComponentGasModel.class);
            Gas gas_0 = 
              ((Gas) singleComponentGasModel_0.getMaterial());
            ConstantMaterialPropertyMethod constantMaterialPropertyMethod_0 = 
              ((ConstantMaterialPropertyMethod) gas_0.getMaterialProperties()
                      .getMaterialProperty(ConstantDensityProperty.class).
                      getMethod());
            constantMaterialPropertyMethod_0.getQuantity().
                    setValue(constantDensityGasValue);
          }
        // COUPLED SOLVER
        }else{ 
            try{ // Check for segregated solver.
                SegregatedFlowModel segregatedFlowModel_0 = 
                    tmpPhys.getModelManager().
                            getModel(SegregatedFlowModel.class);
                tmpPhys.disableModel(segregatedFlowModel_0);
                if(isCompressible){
                    try{
                        SegregatedFluidTemperatureModel tmpModel = 
                         tmpPhys.getModelManager().
                                 getModel(SegregatedFluidTemperatureModel.class);
                        tmpPhys.disableModel(tmpModel);
                    }catch(NeoException e){
                        tmpPhys.getSimulation().
                                println("No Segregated Fluid Temp Found");
                    }
                }
            }catch(NeoException f){
            }
            try{
                CoupledFlowModel coupledFlowModel_0 = 
                  tmpPhys.getModelManager().getModel(CoupledFlowModel.class);
            }catch(NeoException f1){
                tmpPhys.enable(CoupledFlowModel.class);
                if(isCompressible){
                    tmpPhys.enable(CoupledEnergyModel.class);    
                }
                
            }
        }
    }

    private static void turnOnUnSteady(PhysicsContinuum tmpPhys, boolean isUnSteady){
        if(!isUnSteady){ // STEADY
            try{ // test to see if it already has a steady state model
                SteadyModel tmpModel = tmpPhys.getModelManager().
                        getModel(SteadyModel.class);
                tmpPhys.getSimulation().println("Model is steady state");
            }catch(NeoException e){ // No steady model detected
                try{//disable Unsteady if it exists
                    ImplicitUnsteadyModel tmpModel = tmpPhys.getModelManager().
                            getModel(ImplicitUnsteadyModel.class);
                    tmpPhys.disableModel(tmpModel);
                    tmpPhys.getSimulation().
                            println("Phys MSG: Turning OFF Transient");
                }catch(NeoException e1){ //Otherwise just turn on Steady
                }
                tmpPhys.enable(SteadyModel.class);
                tmpPhys.getSimulation().println("Phys MSG: Simulation is STEADY");
            }
        }else{ //UNSTEADY
            try{
                ImplicitUnsteadyModel tmpModel = tmpPhys.getModelManager().
                        getModel(ImplicitUnsteadyModel.class);
                tmpPhys.getSimulation().println("Model is transient");
            }catch(NeoException e){ // No unsteady model detected
                try{//disable steady if it exists
                    SteadyModel tmpModel = tmpPhys.getModelManager().
                            getModel(SteadyModel.class);
                    tmpPhys.disableModel(tmpModel);
                    tmpPhys.getSimulation().
                            println("Phys MSG: Turning OFF Steady");
                }catch(NeoException e1){ //Otherwise just turn on unteady
                    tmpPhys.enable(ImplicitUnsteadyModel.class);
                    tmpPhys.getSimulation().
                            println("Phys MSG: Simulation is TRANSIENT");
                }
            }
        }
    }
    private static void setUnsteady2ndOrder(Simulation tmpSim){
        ImplicitUnsteadySolver implicitUnsteadySolver_0 = 
          ((ImplicitUnsteadySolver) tmpSim.getSolverManager().
                  getSolver(ImplicitUnsteadySolver.class));
        implicitUnsteadySolver_0.getTimeDiscretizationOption().
                setSelected(TimeDiscretizationOption.Type.SECOND_ORDER);
    }
    private static void setTimeStepValue(Simulation tmpSim,double dT){
        ImplicitUnsteadySolver implicitUnsteadySolver_0 =
          ((ImplicitUnsteadySolver) tmpSim.getSolverManager().
                  getSolver(ImplicitUnsteadySolver.class));
        implicitUnsteadySolver_0.getTimeStep().setValue(dT);
    }
    private static void setInnerIts(Simulation tmpSim,int innerIterations){
        InnerIterationStoppingCriterion innerIterationStoppingCriterion_0 =
          ((InnerIterationStoppingCriterion) tmpSim.
                  getSolverStoppingCriterionManager().
                  getSolverStoppingCriterion("Maximum Inner Iterations"));
        innerIterationStoppingCriterion_0.
                setMaximumNumberInnerIterations(innerIterations);
    }

    //Turbulence Solver Settings
    public static void setKWSSTURF(Simulation tmpSim, double newVal){
        tmpSim.println("CFD PHYS: Setting KWSSTURF to: "+newVal);
        KwTurbSolver kwTurbSolver_0 = 
            ((KwTurbSolver) tmpSim.getSolverManager().
                    getSolver(KwTurbSolver.class));
        kwTurbSolver_0.setUrf(newVal);
    }
    public static void setKWSSTViscosity(Simulation tmpSim,
            double newURF, double newTVRLimit){
        KwTurbViscositySolver kwTurbViscositySolver= 
            ((KwTurbViscositySolver) tmpSim.getSolverManager().
                    getSolver(KwTurbViscositySolver.class));
        tmpSim.println("CFD PHYS: Setting KWSSTURF to: "+newTVRLimit);
        kwTurbViscositySolver.setMaxTvr(newTVRLimit);
        tmpSim.println("CFD PHYS: Setting KWSSTURF to: "+newURF);
        kwTurbViscositySolver.setViscosityUrf(newURF);
    }

    //TURBULENCE/RANS METHODS
    public static String getTurbulenceModelName(PhysicsContinuum tmpPhys){
        //Make sure turbulence is active
        try{
            TurbulentModel turbModel=tmpPhys.getModelManager().
                    getModel(TurbulentModel.class);
        }catch(NeoException e){
            try{
                LaminarModel lamModel=tmpPhys.getModelManager().
                        getModel(LaminarModel.class);
                return "Laminar";
            }catch(NeoException f){
                return "Inviscid";
            }
        }
        
        //Check for kw-SST
        String kwSSTStr="kw-SST";
        try{
            tmpPhys.getModelManager().getModel(SstKwTurbModel.class);
            try{ // check for GRT
                tmpPhys.getModelManager().
                        getModel(GammaReThetaTransitionModel.class);
                kwSSTStr+="GRT";
            }catch(NeoException e){
                 try{ //check for 3Eq Gamma
                     tmpPhys.getModelManager().
                             getModel(GammaTransitionModel.class);
                 }catch(NeoException f){
                     
                 }
            }
            try{
                tmpPhys.getModelManager().getModel(SstKwTurbDesModel.class);
                kwSSTStr+="DES";
            }catch(NeoException e){
            }
            return kwSSTStr;
        }catch(NeoException e){
            try{
                tmpPhys.getModelManager().getModel(KOmegaTurbulence.class);
                return "k-Omega";
            }catch(NeoException g){
                return "Unknown";
            }
        }
    }
    private static void turnOnTurbulence(PhysicsContinuum tmpPhys){
        try{ // test to see if it already has a Turbulence mode model
            TurbulentModel tmpModel = tmpPhys.getModelManager().
                    getModel(TurbulentModel.class);
        }catch(NeoException e){ // No turbulence model
            try{ // disable Laminar
                LaminarModel laminarModel_0 = 
                    tmpPhys.getModelManager().
                            getModel(LaminarModel.class);
                tmpPhys.disableModel(laminarModel_0);
            } catch(NeoException e2){
            }
            try{ // disable Inviscide
                InviscidModel inviscidModel_0 = 
                  tmpPhys.getModelManager().
                          getModel(InviscidModel.class);
                tmpPhys.disableModel(inviscidModel_0);
            } catch(NeoException e3){
            }
            tmpPhys.enable(TurbulentModel.class);
        }
        
    }
    //
    private static void turnOnRANS(PhysicsContinuum tmpPhys){
        turnOnTurbulence(tmpPhys);
        try{
            RansTurbulenceModel tmpModel = 
              tmpPhys.getModelManager().getModel(RansTurbulenceModel.class);
        }catch(NeoException e){
            tmpPhys.enable(RansTurbulenceModel.class);
        }
    }
    private static void turnOnKWSST(PhysicsContinuum tmpPhys){

        try{
            RansTurbulenceModel tmpModel = 
              tmpPhys.getModelManager().getModel(RansTurbulenceModel.class);
        }catch(NeoException e){
            tmpPhys.enable(RansTurbulenceModel.class);
        }
        try{
            KOmegaTurbulence tmpModel = 
              tmpPhys.getModelManager().getModel(KOmegaTurbulence.class);
        }catch(NeoException e){
            tmpPhys.enable(KOmegaTurbulence.class);
        }    
        try{
            SstKwTurbModel tmpModel = tmpPhys.getModelManager().
                    getModel(SstKwTurbModel.class);
        }catch(NeoException e){
            tmpPhys.enable(SstKwTurbModel.class);
        }    
        try{
            KwAllYplusWallTreatment tmpModel = tmpPhys.getModelManager().
                    getModel(KwAllYplusWallTreatment.class);
        }catch(NeoException e){
            tmpPhys.enable(KwAllYplusWallTreatment.class);
        }  
       
    }
    private static void turnOffKWSST(PhysicsContinuum tmpPhys){
        try{
            KwAllYplusWallTreatment tmpModel = tmpPhys.getModelManager().
                    getModel(KwAllYplusWallTreatment.class);
            tmpPhys.disableModel(tmpModel );
        }catch(NeoException e){
        }  
        try{
            SstKwTurbModel tmpModel = tmpPhys.getModelManager().
                    getModel(SstKwTurbModel.class);
            tmpPhys.disableModel(tmpModel );
        }catch(NeoException e){
        }    
        try{
            KOmegaTurbulence tmpModel = 
              tmpPhys.getModelManager().getModel(KOmegaTurbulence.class);
            tmpPhys.disableModel(tmpModel );
        }catch(NeoException e){
        }    
        try{
            RansTurbulenceModel tmpModel = 
              tmpPhys.getModelManager().getModel(RansTurbulenceModel.class);
            tmpPhys.disableModel(tmpModel );
        }catch(NeoException e){
        }
    }

    //DES METHODS
    private static void turnOnKwSSTDES(PhysicsContinuum tmpPhys){
        turnOnUnSteady(tmpPhys,true); //make sure unsteady is on
        
        // UNSTACK OTHER MODELS
        turnOffKWSST(tmpPhys);
        
        // TURN ON DES KW-SST
        try{ //test if DES is already turned on
            DesTurbulenceModel desTurbulenceModel_0 = 
              tmpPhys.getModelManager().getModel(DesTurbulenceModel.class);
            tmpPhys.disableModel(desTurbulenceModel_0);
            
        }catch(NeoException e){
            tmpPhys.enable(DesTurbulenceModel.class);
        }
        tmpPhys.enable(DesTurbulenceModel.class);
        tmpPhys.enable(SstKwTurbDesModel.class);
        tmpPhys.enable(KwAllYplusWallTreatment.class);
    }
    private static void turnOffKwSSTDES(PhysicsContinuum tmpPhys){
        turnOffGRT(tmpPhys);
        turnOffKWSST(tmpPhys);
        try{
            KwAllYplusWallTreatment tmpModel = 
              tmpPhys.getModelManager().getModel(KwAllYplusWallTreatment.class);
            tmpPhys.disableModel(tmpModel);
        }catch(NeoException e){
            
        }
        try{
            SstKwTurbDesModel tmpModel = 
              tmpPhys.getModelManager().getModel(SstKwTurbDesModel.class);
            tmpPhys.disableModel(tmpModel);
        }catch(NeoException e){
            
        }
        try{
            DesTurbulenceModel tmpModel = 
              tmpPhys.getModelManager().getModel(DesTurbulenceModel.class);
            tmpPhys.disableModel(tmpModel);
        }catch(NeoException e){
            
        }
        try{ //turn off DES
            DesTurbulenceModel tmpModel = 
              tmpPhys.getModelManager().getModel(DesTurbulenceModel.class);
            tmpPhys.disableModel(tmpModel);
        }catch(NeoException e){
        }
    }

    //GRT METHODS
    public static boolean isGRTinUse(Collection<PhysicsContinuum> allFluidPhysics){
        boolean inUse=false;
        for(PhysicsContinuum tmpPhys:allFluidPhysics){
            try{
              tmpPhys.getModelManager().getModel(GammaReThetaTransitionModel.class);
              inUse=true;
            }catch(NeoException e){
            }
        }
        return inUse;
    }
    private static void turnOnGRT(PhysicsContinuum tmpPhys,double newWallDist){
      try{
        GammaReThetaTransitionModel tmpModel = tmpPhys.getModelManager().
                getModel(GammaReThetaTransitionModel.class);
      }catch(NeoException e){
        tmpPhys.enable(GammaReThetaTransitionModel.class);
      }
      set_GRT_FF(tmpPhys,newWallDist);
    }
    private static void turnOffGRT(PhysicsContinuum tmpPhys){
      try{
        GammaReThetaTransitionModel tmpModel = tmpPhys.getModelManager().
                getModel(GammaReThetaTransitionModel.class);
        tmpPhys.disableModel(tmpModel);
      }catch(NeoException e){
      }
    }
    public static void set_GRT_FF(Simulation tmpSim, 
        PhysicsContinuum tmpPhys, String newGRTFFPresentationName,
        String newGRTFFName, double wallDistance){
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) tmpSim.getFieldFunctionManager()
              .getObject(newGRTFFPresentationName));
      }catch(NeoException e){
          uFF = tmpSim.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName(newGRTFFPresentationName);
      uFF.setFunctionName(newGRTFFName);
      uFF.setDefinition("$WallDistance>"+wallDistance+"?1:0");
            GammaReThetaTransitionModel gRTModel = 
      tmpPhys.getModelManager().getModel(GammaReThetaTransitionModel.class);
      gRTModel.setFreeStreamEdgeDefinitionFieldFunction(uFF);
    }
    
    private static void set_GRT_FF(PhysicsContinuum tmpPhys, double newWallDist) {
      GammaReThetaTransitionModel gRTModel = 
      tmpPhys.getModelManager().getModel(GammaReThetaTransitionModel.class);
      gRTModel.setFreeStreamEdgeDefinitionFieldFunction(
              get_GRT_FF(tmpPhys.getSimulation(),newWallDist));
    }
    private static UserFieldFunction get_GRT_FF(Simulation tmpSim, double newWallDist){
      UserFieldFunction uFF;
      try{
          uFF = ((UserFieldFunction) tmpSim.getFieldFunctionManager().
                  getObject("GammaReTheta Freestream Function"));
      }catch(NeoException e){
          uFF = tmpSim.getFieldFunctionManager().createFieldFunction();
      }
      uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
      uFF.setPresentationName("GammaReTheta Freestream Function");
      uFF.setFunctionName("grt_fs");
      uFF.setDefinition("$WallDistance>"+newWallDist+"?1:0");
      return uFF;
    }
    private static void turnOn3EqSGamma(PhysicsContinuum tmpPhys){
      try{
          GammaTransitionModel tmpModel = tmpPhys.getModelManager().
                  getModel(GammaTransitionModel.class);
      }catch(NeoException e){
          tmpPhys.enable(GammaTransitionModel.class);
      }
    }
    private static void turnOff3EqSGamma(PhysicsContinuum tmpPhys){
      try{
          GammaTransitionModel tmpModel = tmpPhys.getModelManager().
                  getModel(GammaTransitionModel.class);
          tmpPhys.disableModel(tmpModel);
      }catch(NeoException e){
      }
    }
    
    //
    // INITIAL CONDITIONS
    //
    public  static void set_PressureIC(PhysicsContinuum thisPhys,
            double airSpeed,double refPress, double refRho) {
      thisPhys.getInitialConditions()
              .get(InitialPressureProfile.class)
              .setMethod(FunctionScalarProfileMethod.class);
      thisPhys.getInitialConditions()
              .get(InitialPressureProfile.class)
              .getMethod(FunctionScalarProfileMethod.class)
              .setFieldFunction(get_PressureIC_FF(
                      thisPhys.getSimulation(),airSpeed,refPress,refRho));
    }
    public  static void set_VelocityIC(PhysicsContinuum tmpPhys,
            CoordinateSystem cSys,double airSpeed,
            double innerLayer,double outerLayer){
      tmpPhys.getInitialConditions().get(VelocityProfile.class)
              .setMethod(FunctionVectorProfileMethod.class);
      tmpPhys.getInitialConditions().get(VelocityProfile.class)
              .getMethod(FunctionVectorProfileMethod.class)
              .setFieldFunction(get_VelocityIC_FF(
                      tmpPhys.getSimulation(),airSpeed,innerLayer,outerLayer));
      tmpPhys.getInitialConditions()
              .get(VelocityProfile.class).setCoordinateSystem( cSys);
    }
    public  static void set_TemperatureIC(PhysicsContinuum thisPhys,
            double refPress, double refRho) {
        double airR=8.3144598;  // [kJ/kmol.K]  air gas constant
        double molAir=.0289645; // [kg/mol] molar gas constant of air
        thisPhys.getInitialConditions()
                .get(StaticTemperatureProfile.class)
                .setValue(refPress/refRho/(airR/molAir));
    }

    // Init - Segregated Solver
    private static UserFieldFunction get_VelocityIC_FF(Simulation tmpSim, double airSpeed,double innerLayer,double outerLayer) {
        double dLayer=outerLayer-innerLayer;
        UserFieldFunction uFF;
        try{
            uFF = ((UserFieldFunction) tmpSim.getFieldFunctionManager().
                    getObject("Velocity Initial Condition"));
        }catch(NeoException e){
            uFF = tmpSim.getFieldFunctionManager().createFieldFunction();
        }
        uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
        uFF.setPresentationName("Velocity Initial Condition");
        uFF.setFunctionName("vel_ic");
        uFF.setDefinition("["+airSpeed+",0.,0.]");
        return uFF;
    }
    private static UserFieldFunction get_PressureIC_FF(Simulation tmpSim,
            double airSpeed,double refPress,double refRho) {
        UserFieldFunction uFF;
        try{
            uFF = ((UserFieldFunction) tmpSim.getFieldFunctionManager().
                    getObject("Pressure Initial Condition"));
        }catch(NeoException e){
            uFF = tmpSim.getFieldFunctionManager().createFieldFunction();
        }
        uFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
        uFF.setPresentationName("Pressure Initial Condition");
        uFF.setFunctionName("pressure_ic");
        uFF.setDefinition("0");
        return uFF;
    }
     
    
    //MONITORS
    public static ArrayList<Monitor> getPhysicsMonitorList(Simulation tmpSim){
        ArrayList<Monitor> retList = new ArrayList();
        try{
            retList.add(tmpSim.getMonitorManager().getMonitor("Continuity"));
            retList.add(tmpSim.getMonitorManager().getMonitor("X-momentum"));
            retList.add(tmpSim.getMonitorManager().getMonitor("Y-momentum"));
            retList.add(tmpSim.getMonitorManager().getMonitor("Z-momentum"));
        }catch(NeoException e){
            
        }
        try{
            retList.add(tmpSim.getMonitorManager().getMonitor("Energy"));  
        }catch(NeoException e){
            
        }
        try{
            ResidualMonitor residualMonitor_5 = 
              ((ResidualMonitor) tmpSim.getMonitorManager().getMonitor("Tke"));
            ResidualMonitor residualMonitor_6 = 
              ((ResidualMonitor) tmpSim.getMonitorManager().getMonitor("Sdr"));
        }catch(NeoException e){
            
        }
        return retList;
    }
    
}
