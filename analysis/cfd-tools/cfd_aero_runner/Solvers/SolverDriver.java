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
package Solvers;

import java.util.*;
import star.common.*;
import star.base.neo.*;
import star.base.report.*;

import PhysicsTools.*;
import Tools.CriterionTool;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import star.coupledflow.ContinuityConvergenceAccelerator;
import star.coupledflow.ConvergenceAcceleratorOption;
import star.coupledflow.CoupledImplicitSolver;
import star.kwturb.KwTurbSolver;
import star.kwturb.KwTurbViscositySolver;
import star.segregatedenergy.SegregatedEnergySolver;
import star.segregatedflow.ContinuityInitializer;
import star.segregatedflow.PressureSolver;
import star.segregatedflow.SegregatedFlowSolver;
import star.segregatedflow.VelocitySolver;

public class SolverDriver {
  boolean useAutoTimeStep=false;
  double autoTimeStep=1e-4;

public SolverDriver(){
}
  
// Time Step Control
public void turnOnAutoTimeStep(){
  useAutoTimeStep=true;
}
public void turnOffAutoTimeStep(){
  useAutoTimeStep=false;
}
public double getAutoTimeStep(){
  return autoTimeStep;
}
public boolean useAutoTimeStep(){
  autoTimeStep=getAutoTimeStep();
  return useAutoTimeStep;
}
public static void set2ndOrderTimeDisc(Simulation tmpSim){
  ImplicitUnsteadySolver impUnsSolver = 
    ((ImplicitUnsteadySolver) tmpSim.getSolverManager()
        .getSolver(ImplicitUnsteadySolver.class));
  impUnsSolver.getTimeDiscretizationOption()
      .setSelected(TimeDiscretizationOption.Type.SECOND_ORDER);
}
public static void setTimeStep(Simulation tmpSim,double newTimeStep){
  ImplicitUnsteadySolver impUnsSolver = 
    ((ImplicitUnsteadySolver) tmpSim.getSolverManager()
        .getSolver(ImplicitUnsteadySolver.class));
  NumberFormat formattedDouble = new DecimalFormat("0.#E0");
  String truncatedTimeStep=formattedDouble.format(newTimeStep);
  impUnsSolver.setTimeStep(Double.parseDouble(truncatedTimeStep));
}

// Solver Stopping Criterion
public static void setInnerIts(Simulation tmpSim,int innerIterations){
  InnerIterationStoppingCriterion innerIterationStoppingCriterion_0 =
    ((InnerIterationStoppingCriterion)
        CriterionTool.getCriterion(tmpSim, "Maximum Inner Iterations"));
  innerIterationStoppingCriterion_0
      .setMaximumNumberInnerIterations(innerIterations);
}
public static void setFinalSolutionTime(Simulation tmpSim,double newValue){
  CriterionTool.getCriterion(tmpSim, "Maximum Physical Time").setIsUsed(true);
  ((PhysicalTimeStoppingCriterion) CriterionTool.getCriterion(
      tmpSim, "Maximum Physical Time")).setMaximumTime(newValue);
}
public static void deactivateMaxStepsCriteria(Simulation tmpSim){
  ((SolverStoppingCriterion) CriterionTool.getCriterion(
      tmpSim, "Maximum Steps")).setIsUsed(false);
}
public static void setMaxSolverSteps(Simulation tmpSim, int numMaxSteps){
  StepStoppingCriterion maxSteps;
  try{
    maxSteps=(StepStoppingCriterion) tmpSim
        .getSolverStoppingCriterionManager()
            .getObject("Maximum Steps");
  }catch(NeoException e){
      maxSteps = tmpSim.getSolverStoppingCriterionManager()
          .createSolverStoppingCriterion(
              StepStoppingCriterion.class, "Maximum Steps");
  }
  ((StepStoppingCriterion) maxSteps).setMaximumNumberSteps(numMaxSteps);
  ((StepStoppingCriterion) maxSteps).setIsUsed(true);
}

public static String getSatisfiedStoppingCriteriaNames(Simulation tmpSim){
  String retStr = "{";
  for(SolverStoppingCriterion tmpCrit : 
      tmpSim.getSolverStoppingCriterionManager().getObjects()){
    if(tmpCrit.inUse() && tmpCrit.getIsSatisfied()){
      retStr = retStr + ";" +tmpCrit.getPresentationName();
    }
  }
  retStr=retStr+"}";
  return retStr;
}  
  
// Simulation State
public static int getCurrentIterationLevel(Simulation tmpSim){
  return tmpSim.getSimulationIterator().getCurrentIteration();
}
public static int getCurrentStepLevel(Simulation tmpSim){
  return tmpSim.getSimulationIterator().getNumberOfSteps();
}
public static double getCurrentTime(Simulation tmpSim){
  return tmpSim.getSolution().getPhysicalTime();
}
public static double getTimeStep(Simulation tmpSim){
  return tmpSim.getSolverManager().getSolver(ImplicitUnsteadySolver.class)
      .getTimeStep().getRawValue();
}
    
// Simulation Drivers
public static void stepSimulation(Simulation tmpSim, PhysicsContinuum tmpPhys,
    int numSteps, boolean needAutoTS){
  /* Method to run tmpSimlation for numSteps *and* also respect additional
  stopping criteria. This should increase tmpSimlation efficiency for
  two reasons:
    1) It is possible to connect to the server and set different values while
    the sim is running to bring it down early or make adjustments.
    2) Additional stopping criteria can be used.
  
  Pseudo-code:
    If sim stopped because max number of steps achieved
      Note: User may have set other criterion that stops this run quickly after.
        1) Add numSteps inner iteration criteria.
        2) Keep running.
    Else sim stopped because of other stopping criteria
      If unsteady:
      1) Check to see if max physical time enabled & exceeded.
         If so, add number of steps to the clock and run
    Else:
      1) Disable all enabled stopping criteria for 1/10 of numSteps.
      2) Get maxSteps, add number of numSteps.
      3) Step tmpSimlation 1/10 numSteps.
      4) Turn on all other stopping criteria.
      5) Run tmpSimlation.
  */
  
  String tab2 = "  ";
  String tab4 = "    ";
  String tab6 = "      ";
  String tab8 = "        ";

  tmpSim.println(tab2 + "===============================");
  tmpSim.println(tab2 + "Simulation stepping reporter: ");
  tmpSim.println(tab2 + "===============================");
  tmpSim.println(tab4 + "JAVA driven automatic stepping: " + needAutoTS);
  if(!needAutoTS){ // Forcibly drive tmpSimlation specified number of steps.
    tmpSim.println(tab6 + "Manual override. Stepping " + numSteps + " steps.");
    tmpSim.getSimulationIterator().step(numSteps, true);
  }else{ // Automatically run.
    SolverStoppingCriterion maxSteps =
        CriterionTool.getCriterion(tmpSim, "Maximum Steps");
    
    if(maxSteps.getIsSatisfied() && maxSteps.getIsUsed()){
      tmpSim.println(tab6 + "Case was previously stopped by max steps limit.");
      tmpSim.println(tab8 + "Old step limit criterion: " 
          + ((StepStoppingCriterion) maxSteps).getMaximumNumberSteps());
      tmpSim.println(tab8 + "Increasing max steps by:  " + numSteps);
      
      int tmpNumSteps =
          ((StepStoppingCriterion) maxSteps).getMaximumNumberSteps();
      ((StepStoppingCriterion) maxSteps)
          .setMaximumNumberSteps(tmpNumSteps + numSteps);
      tmpSim.println(tab8 + "New steps allowed: "
          + ((StepStoppingCriterion) maxSteps).getMaximumNumberSteps());
      tmpSim.println(tab8 + "Running...");
      tmpSim.getSimulationIterator().run(true);

    }else{
      tmpSim.println(tab6 + "Checking for unsteady case setup.");
      tmpSim.println(tab6 + "Is this an unsteady case: "
          + CFD_Physics.isUnsteady(tmpPhys));
      
      if(CFD_Physics.isUnsteady(tmpPhys)){ // Check if case ran out of time.
        PhysicalTimeStoppingCriterion maxTime =
            ((PhysicalTimeStoppingCriterion)
                CriterionTool.getCriterion(tmpSim, "Maximum Physical Time"));
        double dT =
            ((ImplicitUnsteadySolver)
                tmpSim.getSolverManager().getSolver(
                    ImplicitUnsteadySolver.class))
                .getTimeStep().getInternalValue(); // [Seconds].
        tmpSim.println(tab6 + "Current stop time: "
            + maxTime.getMaximumTime().getInternalValue());
        tmpSim.println(tab6 + "Recommended final time: " + numSteps * dT);
        
        if( maxTime.inUse() && maxTime.getIsSatisfied()){
          tmpSim.println(tab6 + "Case was previously stopped becacuse the "
              + "max physical time limit was reached. Incrementing time limit "
              + "and continuing to run.");
          tmpSim.println(tab6 + "...added value of time: " + numSteps * dT);
          maxTime.setMaximumTime(
              maxTime.getMaximumTime().getInternalValue() + numSteps*dT);
          tmpSim.getSimulationIterator().run(true);

        }else if( maxTime.inUse() && !maxTime.getIsSatisfied()
            && (numSteps * dT) * 1.2 < maxTime.getMaximumTime()
                .getInternalValue()){
            tmpSim.println(tab6 + "Case did not previously reach the previous "
                + "max physical time limit.");
            tmpSim.println(tab6 + "The requested time limit would be less than"
                + " the exiting limit in the sim file."
                + " Switching to the new lower limit.");
            maxTime.setMaximumTime(numSteps * dT);
            tmpSim.getSimulationIterator().run(true);
        }else{
            tmpSim.println(tab6 + "Case did not reached its recommended time "
                + "limit. Ignoring timer request and completing the run.");
            tmpSim.getSimulationIterator().run(true);
        }
      }else{ // Check to see if case was stopped for some other reason.
        tmpSim.println(tab6 + "Case was previously stopped for "
            + "non-maximum step/time limit. Investigating cause:");
        // 1) Gather all Criterion, but only care about active use ones
        Collection<SolverStoppingCriterion> allCriterion =
            tmpSim.getSolverStoppingCriterionManager().getObjects();
        Collection<SolverStoppingCriterion> ofInterest = new ArrayList();
        boolean previousStopByCriterion=false;
        for(SolverStoppingCriterion tmp : allCriterion){
          if(tmp.inUse()){
            ofInterest.add(tmp);
            if(tmp.getIsSatisfied() && !previousStopByCriterion){
              tmpSim.println(tab8 + "...tmpSimlation was previously stopped "
                  + "due to alternative criterion: "
                  + tmp.getPresentationName());
              previousStopByCriterion=true;
            }
          }
        }
        if(previousStopByCriterion){
          // Whatever criterion had stopped it should be temporarily turned off.
          // This avoids an immediate satisfaction on a minor change that may
          // take time to propagate through the solution field.
          for(SolverStoppingCriterion tmp:ofInterest){ 
            tmpSim.println(tab8 
                + "...temporarily disabling all in-use stopping criterion");
            tmp.setIsUsed(false); 
          }
          tmpSim.println(tab8
              + "...stepping tmpSimlation 1/10 of prescribed steps");
          tmpSim.getSimulationIterator()
              .step( (numSteps - (numSteps % 10) ) / 10, true);
          tmpSim.println(tab8 + "...in-use stopping criterion re-enabled");
          for(SolverStoppingCriterion tmp : ofInterest){ 
            tmp.setIsUsed(true);
          }
        }else{
           // The simulation had been brought down for some other reason, i.e.
           // a manual shutdown or not crash and was stopped before any in-use
           // criterion had activated.
          tmpSim.println(tab8 + "...existing simulation was previously stopped "
              + "for a non-criterion based reason.");
          int currentStepLevel;
          if(CFD_Physics.isUnsteady(tmpPhys)){
            currentStepLevel =
                tmpSim.getSimulationIterator().getCurrentTimeLevel();
          }else{
            currentStepLevel =
                tmpSim.getSimulationIterator().getCurrentIteration();
          }
          
          tmpSim.println(tab8 + "...current tmpSimlation step level: "
              + currentStepLevel);
          int desiredStepLevel = currentStepLevel + numSteps;
          tmpSim.println(tab8 + "...desired tmpSimlation step level: "
              + desiredStepLevel);
          tmpSim.println(tab8 + "...adjusting tmpSimlation max step criterion "
              + desiredStepLevel);
          setMaxSolverSteps(tmpSim, desiredStepLevel);
        }
        tmpSim.println(tab8 + "...running tmpSimlation.");
        tmpSim.println(tab2 + "===============================");
        tmpSim.println(tab2 + " ");
        tmpSim.println(tab2 + " ");
        tmpSim.getSimulationIterator().run(true);
      }
    }
  }
}

// Special Solver Drivers
// Coupled Solver Drivers
public static void useCCA(Simulation tmpSim, double ccaURF,
        int updateFreq, double initialValue, int startRampIt, int stopRampIt){
  CoupledImplicitSolver coupledImplicitSolver_0 = 
    ((CoupledImplicitSolver) tmpSim.getSolverManager().
            getSolver(CoupledImplicitSolver.class));
  coupledImplicitSolver_0.getConvergenceAcceleratorManager().
          getConvergenceAcceleratorOption().setSelected(
                  ConvergenceAcceleratorOption.
                          Type.CONTINUITY_CONVERGENCE_ACCELERATOR);
  ContinuityConvergenceAccelerator continuityConvergenceAccelerator_0 = 
    ((ContinuityConvergenceAccelerator) coupledImplicitSolver_0.
            getConvergenceAcceleratorManager().getConvergenceAccelerator());
  continuityConvergenceAccelerator_0.setConvergenceAcceleratorUpdateFreq(updateFreq);
  continuityConvergenceAccelerator_0.setUrf(ccaURF);
  continuityConvergenceAccelerator_0.getRampCalculatorManager().
          getRampCalculatorOption().setSelected(RampCalculatorOption.
                  Type.LINEAR_RAMP);
  LinearRampCalculator linearRampCalculator_0 = 
    ((LinearRampCalculator) continuityConvergenceAccelerator_0.
            getRampCalculatorManager().getCalculator());
  linearRampCalculator_0.setStartIteration(100);
  linearRampCalculator_0.setEndIteration(stopRampIt);
  linearRampCalculator_0.setInitialRampValue(0.01);
}
  public static void coupledGSIInit(Simulation tmpSim,int maxGSLevels,
          int maxGSIts, double gsiCFL, double convTol){
      CoupledImplicitSolver coupledImpSolver;
      try{
          coupledImpSolver = 
              ((CoupledImplicitSolver) tmpSim.getSolverManager().
                      getSolver(CoupledImplicitSolver.class));
          coupledImpSolver.getExpertInitManager().getExpertInitOption().
                  setSelected(ExpertInitOption.Type.GRID_SEQ_METHOD);      
          GridSequencingInit gSI = 
            ((GridSequencingInit) coupledImpSolver.
                    getExpertInitManager().getInit());
          gSI.setGSCfl(gsiCFL);
          gSI.setConvGSTol(convTol);
          gSI.setMaxGSIterations(maxGSIts);
          gSI.setMaxGSLevels(maxGSLevels);
       }catch(NeoException e){
          tmpSim.println("Coupled Solver is not enabled. No GSI available.");
      }
  }
  public static void useLinearRampOnCoupledSolver(Simulation tmpSim,
          double initialValue, int startRampIt, int stopRampIt){
      CoupledImplicitSolver coupledImplicitSolver_0 = 
        ((CoupledImplicitSolver) tmpSim.getSolverManager().
                getSolver(CoupledImplicitSolver.class));
      coupledImplicitSolver_0.getRampCalculatorManager().
              getRampCalculatorOption().setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
      LinearRampCalculator linearRampCalculator_0 = 
        ((LinearRampCalculator) coupledImplicitSolver_0.
                getRampCalculatorManager().getCalculator());
      linearRampCalculator_0.setInitialRampValue(initialValue);
      linearRampCalculator_0.setStartIteration(startRampIt);
      linearRampCalculator_0.setEndIteration(stopRampIt);
  }
  
  //Linear ramps for segregated solver
  public static void setSolverContinuityInit(Simulation tmpSim,
          boolean setEnabled,int numIts){
    SegregatedFlowSolver segregatedFlowSolver_0 = 
      ((SegregatedFlowSolver) tmpSim.getSolverManager().
              getSolver(SegregatedFlowSolver.class));
    segregatedFlowSolver_0.setContinuityInitialization(setEnabled);
    if(setEnabled){
        ContinuityInitializer continuityInitializer_0 = 
          segregatedFlowSolver_0.getContinuityInitializer();
        continuityInitializer_0.setIterations(numIts);
    }
  }
  
  public static void useLinearRampOnSegregatedSolver(Simulation tmpSim,
          int stopRampIt){
      SegregatedFlowSolver segregatedFlowSolver = 
        ((SegregatedFlowSolver) tmpSim.getSolverManager()
                .getSolver(SegregatedFlowSolver.class));
      //velocity
      VelocitySolver velocitySolver = 
        segregatedFlowSolver.getVelocitySolver();
      velocitySolver.getRampCalculatorManager()
              .getRampCalculatorOption()
              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
      LinearRampCalculator velLinearRampCalculator = 
        ((LinearRampCalculator) velocitySolver.getRampCalculatorManager()
                .getCalculator());
      velLinearRampCalculator.setEndIteration(stopRampIt);

      //pressure
      PressureSolver pressureSolver = 
        segregatedFlowSolver.getPressureSolver();
      pressureSolver.getRampCalculatorManager().getRampCalculatorOption()
              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);

      LinearRampCalculator pressLinearRampCalculator = 
        ((LinearRampCalculator) pressureSolver.getRampCalculatorManager()
                .getCalculator());
      pressLinearRampCalculator.setEndIteration(stopRampIt);

  }
  public  static void useLinearRampOnSegPressureSolver(Simulation tmpSim,
          int startRampIt, int stopRampIt){
      SegregatedFlowSolver segregatedFlowSolver = 
        ((SegregatedFlowSolver) tmpSim.getSolverManager()
                .getSolver(SegregatedFlowSolver.class));
      //pressure
      PressureSolver pressureSolver = 
        segregatedFlowSolver.getPressureSolver();
      pressureSolver.getRampCalculatorManager().getRampCalculatorOption()
              .setSelected(RampCalculatorOption.Type.LINEAR_RAMP);
      LinearRampCalculator pressLinearRampCalculator = 
        ((LinearRampCalculator) pressureSolver.getRampCalculatorManager()
                .getCalculator());
      pressLinearRampCalculator.setInitialRampValue(0.1);
      pressLinearRampCalculator.setStartIteration(startRampIt);
      pressLinearRampCalculator.setEndIteration(stopRampIt);
  }

  // Init - Turbulence
  public static void setKWBoundaryLayerInit(Simulation tmpSim,
          boolean needBLInit){
      KwTurbSolver kwTurbSolver_0 = 
        ((KwTurbSolver) tmpSim.getSolverManager().
                getSolver(KwTurbSolver.class));
      kwTurbSolver_0.setTurbulenceInitialization(needBLInit);
  }
  public static void setMaxTVR(Simulation tmpSim, double maxVal,
          double tmpURF){
    KwTurbViscositySolver kwTurbViscositySolver_0 = 
      ((KwTurbViscositySolver) tmpSim.getSolverManager().
              getSolver(KwTurbViscositySolver.class));
    kwTurbViscositySolver_0.setMaxTvr(maxVal);
    kwTurbViscositySolver_0.setViscosityUrf(tmpURF);
  }
  public static void setkwSSTURF(Simulation tmpSim, double newVal){
          KwTurbSolver kwTurbSolver_0 = 
    ((KwTurbSolver) tmpSim.getSolverManager().getSolver(KwTurbSolver.class));
  kwTurbSolver_0.setUrf(newVal);
  }
  
  // Energy Driver
  public static  void setEnergyURF(Simulation tmpSim, double newVal){
      SegregatedEnergySolver segregatedEnergySolver_0 = 
          ((SegregatedEnergySolver) tmpSim.getSolverManager().
                  getSolver(SegregatedEnergySolver.class));
      segregatedEnergySolver_0.setFluidUrf(newVal);
  }

}
