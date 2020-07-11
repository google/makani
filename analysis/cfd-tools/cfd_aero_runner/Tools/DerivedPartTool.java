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
package Tools;

import java.util.*;


import star.base.neo.*;
import star.common.*;

import star.vis.*;

public class DerivedPartTool {
  static double eps=1.e-6;
  
  // Planes
  public static ArbitrarySection getPartLinkedArbitrarySection(
          Simulation tmpSim, GeometryPart inputPart){

    ArbitrarySection arbitrarySection;
    String sectionName = inputPart.getPresentationName();
    
    Units units_0 = SimTool.getUnitsVec(tmpSim);
      tmpSim.getUnitsManager().getPreferredUnits(
        new IntVector(new int[] {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    try{
      arbitrarySection = (ArbitrarySection) tmpSim.getPartManager().
              getObject(sectionName);
    }catch(NeoException e){
      arbitrarySection = 
      (ArbitrarySection) tmpSim.getPartManager()
        .createArbitraryImplicitPart(
          new NeoObjectVector(new Object[] {}),
          new NeoObjectVector(inputPart.getPartSurfaces().toArray()),
          "", units_0, true);
      arbitrarySection.setPresentationName(inputPart.getPresentationName());
    }
    return arbitrarySection;
  }
  
  public static PlaneSection singlePlane(Simulation tmpSim,
          Collection<Region> myRegs,
          String planeName, CoordinateSystem myCsys,
          double[] newOrigin, double[] newNormal){
    PlaneSection myPlane;
    try{
      myPlane=(PlaneSection) tmpSim.getPartManager().getObject(planeName);
    }catch(NeoException e){
      myPlane=
          (PlaneSection) tmpSim.getPartManager().createImplicitPart(
                  new NeoObjectVector(new Object[] {}),
                  new DoubleVector(new double[] {0.0, 0.0, 1.0}),
                  new DoubleVector(new double[] {0.0, 0.0, 0.0}), 0, 1,
                  new DoubleVector(new double[] {0.0}));
    }
    myPlane.setCoordinateSystem(myCsys);

    Coordinate myOrientation = 
      myPlane.getOrientationCoordinate();
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    myOrientation.setCoordinate(units_0, units_0, units_0,
            new DoubleVector(newNormal));
    Coordinate myOrigin = myPlane.getOriginCoordinate();
    myOrigin.setCoordinate(units_0, units_0, units_0,
            new DoubleVector(newOrigin));
    myPlane.setPresentationName(planeName);
    myPlane.getInputParts().setObjects(myRegs);
    return myPlane;
  }
  
  
  public static PointPart makePointPart(Simulation tmpSim, String tmpName,
          CoordinateSystem tmpCsys, double[] xyzLoc){
    // Pitot part
    PointPart pointPart;
    try{
        pointPart = (PointPart) tmpSim.getPartManager().getObject(tmpName);
    }catch(NeoException e){
        pointPart = 
        tmpSim.getPartManager().createPointPart(
                new NeoObjectVector(new Object[] {}),
                new DoubleVector(new double[] {0.,0.,0.}));
        pointPart.setPresentationName(tmpName);
    }

    // set
    Coordinate coordinate_0 = pointPart.getPointCoordinate();
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    coordinate_0.setCoordinate(units_0, units_0, units_0,
            new DoubleVector(new double[] {xyzLoc[0], xyzLoc[1], xyzLoc[2]}));

    //assign base Csys
    pointPart.setCoordinateSystem(tmpCsys);

    return pointPart;
  }
  
  // Cells
  public static ThresholdPart highValueCells(Simulation tmpSim,
          Collection<Region> bodyParts,
    String partName, String functionName, double highValue){
      ThresholdPart myPart;
      try{
          myPart = (ThresholdPart) tmpSim.getPartManager().getObject(partName);
      }catch(NeoException e){
          Units units_0 = 
            ((Units) tmpSim.getUnitsManager().getObject("m"));
          NullFieldFunction nullFieldFunction_0 = 
            ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                    getFunction("NullFieldFunction"));
          myPart = 
            (ThresholdPart) tmpSim.getPartManager().
                    createThresholdPart(new NeoObjectVector(new Object[] {}),
                            new DoubleVector(new double[] {0.0, 1.0}), units_0,
                            nullFieldFunction_0, 0);
          myPart.setPresentationName(partName);
      }
      myPart.getInputParts().setQuery(null);
      myPart.getInputParts().setObjects(bodyParts);
      myPart.setFieldFunction(tmpSim.getFieldFunctionManager().
              getFunction(functionName));
      myPart.setMode(ThresholdMode.ABOVE_TAG);
      myPart.getRangeQuantities().setArray(new DoubleVector(
              new double[] {highValue, highValue}));
      return myPart;
  }
  public static ThresholdPart lowValueCells(Simulation tmpSim,
          Collection<Region> bodyParts,
    String partName, String functionName, double lowValue){
      ThresholdPart myPart;
      try{
          myPart = (ThresholdPart) tmpSim.getPartManager().getObject(partName);
      }catch(NeoException e){
          Units units_0 = 
            ((Units) tmpSim.getUnitsManager().getObject("m"));
          NullFieldFunction nullFieldFunction_0 = 
            ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                    getFunction("NullFieldFunction"));
          myPart = 
            (ThresholdPart) tmpSim.getPartManager().createThresholdPart(
                    new NeoObjectVector(new Object[] {}), new DoubleVector(
                            new double[] {0.0, 1.0}), units_0,
                            nullFieldFunction_0, 0);
          myPart.setPresentationName(partName);
      }
      myPart.getInputParts().setQuery(null);
      myPart.getInputParts().setObjects(bodyParts);
      myPart.setFieldFunction(tmpSim.getFieldFunctionManager().
              getFunction(functionName));
      myPart.setMode(ThresholdMode.BELOW_TAG);
      myPart.getRangeQuantities().setArray(new DoubleVector(
              new double[] {lowValue, lowValue}));
      return myPart;
  }
  
  // Surfaces
  public static IsoPart getSingleIsoSurfPart(Simulation tmpSim, String isoName,
          Collection<Region> myRegs, FieldFunction tmpFF,double newValue){
    IsoPart retPart;
    try{
      retPart = (IsoPart) tmpSim.getPartManager().getObject(isoName);
    }catch(NeoException e){
      NullFieldFunction nullFF = 
        ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                getFunction("NullFieldFunction"));
      retPart = 
        tmpSim.getPartManager().createIsoPart(new NeoObjectVector(
                new Object[] {}), nullFF);
    }
    retPart.setPresentationName(isoName);
    retPart.setMode(IsoMode.ISOVALUE_SINGLE);
    SingleIsoValue singleIsoValue_0 = retPart.getSingleIsoValue();
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
  
  // Line Probes
  public static LinePart getLineProbe(Simulation tmpSim,
      Collection<Region> myRegs, CoordinateSystem probeCsys, double lengthScale,
      String probeName, int nPts){
    LinePart lineProbe;
    try{
        lineProbe=(LinePart) tmpSim.getPartManager().getObject(probeName);
    }catch(NeoException e){
        lineProbe=tmpSim.getPartManager().createLinePart(
                new NeoObjectVector(new Object[] {}),
                new DoubleVector(new double[] {0.0, 0.0, 0.0}),
                new DoubleVector(new double[] {1.0, 0.0, 0.0}), 20);
        lineProbe.setPresentationName(probeName);
    }
    lineProbe.setCoordinateSystem( probeCsys);
    lineProbe.setResolution(nPts);
    lineProbe.getInputParts().setObjects(myRegs);
    Coordinate coordinate_0 = 
       lineProbe.getPoint2Coordinate();
    Units units_0 = 
      ((Units) tmpSim.getUnitsManager().getObject("m"));
    coordinate_0.setCoordinate(units_0, units_0, units_0,
            new DoubleVector(new double[] {lengthScale*10.0, 0.0, 0.0}));
    return lineProbe;
}
  
  // Streamlines
  public static StreamPart getStreamlinePart(Simulation tmpSim, String partName){
    StreamPart retPart;
    try{
      retPart = (StreamPart) tmpSim.getPartManager().getObject(partName);
    }catch(NeoException e){
      NullFieldFunction nullFF = 
        ((NullFieldFunction) tmpSim.getFieldFunctionManager().
                getFunction("NullFieldFunction"));
      retPart = 
        tmpSim.getPartManager().createStreamPart(
            new NeoObjectVector(new Object[] {}), new DoubleVector(
                    new double[] {0.0, 0.0, 0.0}), nullFF, 0.0, 20, 0);
      retPart.setPresentationName(partName);
    }
    return retPart;
  }
  public static void setStreamlineLineSeed(StreamPart thisSP, Units coordUnit,
          double[] startOrigin, double[] endOrigin,
          int lineSeedResolution){
    thisSP.setMode(SeedMode.LINE);
    // set resolution
    LineSeed lineSeed = thisSP.getLineSeed();
    lineSeed.setResolution(lineSeedResolution);
    // Set start and end points
    Coordinate lineStart = lineSeed.getPoint1Coordinate();
    Coordinate lineEnd = lineSeed.getPoint2Coordinate();
    lineStart.setCoordinate(coordUnit, coordUnit, coordUnit,
            new DoubleVector(new double[] {3.0, -13.0, 0.0}));
    lineEnd.setCoordinate(coordUnit, coordUnit, coordUnit,
            new DoubleVector(new double[] {3.0, 13.0, 0.0}));
  }
  public static void setStreamlinePartSeed(StreamPart thisSP, Units coordUnit,
          ArrayList<NamedObject> seedParts){
    
    thisSP.setMode(SeedMode.PART);
    SourceSeed partSeed = thisSP.getSourceSeed();
    partSeed.setSeedSources(seedParts);
    partSeed.setNGridPoints(new IntVector(new int[] {5, 5}));
    
  }
  // Resampled Volume Parts
  public static ResampledVolumePart getResampledVolumePart(Simulation tmpSim,
          String partName){
    ResampledVolumePart retPart;
    try{
      retPart = (ResampledVolumePart) tmpSim.getPartManager().getObject(partName);
    }catch(NeoException e){
      Units meterUnits = 
        ((Units) tmpSim.getUnitsManager().getObject("m"));

      Units radianUnits = 
        ((Units) tmpSim.getUnitsManager().getObject("radian"));
      retPart = 
      tmpSim.getPartManager().createResampledVolumePart(
          new NeoObjectVector(new Object[] {}), 1.0, meterUnits, 0.001,
              meterUnits, meterUnits, meterUnits, 
              new DoubleVector(new double[] {0.0, 0.0, 0.0}), meterUnits,
              meterUnits, meterUnits, 
              new DoubleVector(new double[] {1.0, 1.0, 1.0}), meterUnits,
              meterUnits, meterUnits, 
              new DoubleVector(new double[] {1.0, 0.0, 0.0}), radianUnits, 0.0);
      retPart.setPresentationName(partName);
    }
    return retPart;
  }
  
  public static ResampledVolumePart setResampledVolumeCorners(
          ResampledVolumePart thisPart,double[] corner1, double[] corner2){
    Units meterUnits = 
      ((Units) thisPart.getSimulation().getUnitsManager().getObject("m"));
    double c1_x = corner1[0];
    double c1_y = corner1[1];
    double c1_z = corner1[2];
    double c2_x = corner2[0];
    double c2_y = corner2[1];
    double c2_z = corner2[2];
    
    double origin_x = 0.5 * (c1_x + c2_x);
    double origin_y = 0.5 * (c1_y + c2_y);
    double origin_z = 0.5 * (c1_z + c2_z);
    
    double length_x = Math.abs(c1_x - c2_x);
    double length_y = Math.abs(c1_y - c2_y);
    double length_z = Math.abs(c1_z - c2_z);
    
    thisPart.getOriginCoordinate().setCoordinate(meterUnits, meterUnits, meterUnits, 
            new DoubleVector(new double[] {origin_x, origin_y, origin_z}));
    thisPart.getSizeCoordinate().setCoordinate(meterUnits, meterUnits, meterUnits,
            new DoubleVector(new double[] {length_x, length_y, length_z}));

    return thisPart;
  }

}
