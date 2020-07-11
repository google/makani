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
package MeshTools;

import java.util.*;

import star.common.*;
import star.base.neo.*;

import star.meshing.*;
import star.prismmesher.*;
import star.trimmer.*;

public class CFD_TrimmerModel {
    Simulation simu;

    double baseSize;
    
    AutoMeshOperation pBMO;
    
    
    public CFD_TrimmerModel(Simulation simu,String opName,
            CoordinateSystem cartCsys, double baseSize){
        
        this.simu=simu;

        try{ 
            this.pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
        }catch(NeoException e){
            this.pBMO = simu.get(MeshOperationManager.class).createAutoMeshOperation(new StringVector(new String[] {"star.trimmer.TrimmerAutoMesher", "star.prismmesher.PrismAutoMesher"}), new NeoObjectVector(new Object[] {}));
            this.pBMO.setPresentationName(opName);
        }

        //Prism Default Settings
        double gapFillPct = 40.0;
        double minPrismThickPct = 0.01;
        double layerChopPct = 0.0;
        double nearCoreAR = 0.75;
        int nPrisms = 16;
        double firstCellThickness=2.5E-5;
        double prismAbsThickness=0.05;
        // Trimmer Default Settings
        double trimToPrismR = 2.0;
        double biggestCellSizePct = 6400.0;
        int trimGR = 8;
        
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
        maxCellSize.setRelativeSizeValue(biggestCellSizePct);
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
        prismThickness_0.setAbsoluteSize(prismAbsThickness, MeshOpTool.getPrefUVec(simu));
    }
    
    public AutoMeshOperation getVMOp(){
        return pBMO;
    }
    public void addPart(GeometryPart tmpPart){
        pBMO.getInputGeometryObjects().add(tmpPart);
    }
    
    public void addParts(ArrayList<GeometryPart> tmpParts){
      pBMO.getInputGeometryObjects().addObjects(tmpParts);
    }

    public SurfaceCustomMeshControl surfControl(String controlName){
        return MeshOpTool.surfControl(pBMO, controlName);
    }
    
    public VolumeCustomMeshControl getVolumeControl(String controlName){
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
    
    public static void custVCIsoSize(VolumeCustomMeshControl volControl, String defRelAbs, double newVal){
        // Enable Surface Remeshing Volume Control
        VolumeControlTrimmerSizeOption sizeOption = 
          volControl.getCustomConditions().get(VolumeControlTrimmerSizeOption.class);
        sizeOption.setVolumeControlBaseSizeOption(true);
        sizeOption.setTrimmerAnisotropicSizeOption(false);
        if(defRelAbs.equals("Absolute")){
            VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
            vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
            vCS.setAbsoluteSizeValue(newVal,MeshOpTool.getPrefUVec(volControl.getSimulation()));
        }else if(defRelAbs.equals("Relative")){
            VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
            vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
            vCS.setRelativeSizeValue(newVal);
        }
        volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
     }
    

}
