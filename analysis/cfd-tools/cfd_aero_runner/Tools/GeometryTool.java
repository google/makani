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

import star.common.*;
import star.base.neo.*;
import star.meshing.*;

/**
 *
 * This class makes it easier to perform or query GeometryPart operations.
 */
public class GeometryTool {

private static SimpleSpherePart makeSphere(Simulation tmpSim, 
    String sphereName, CoordinateSystem cartCsys, double radius){
  /* Makes a CAD based simple Sphere GeometryPart. */
  SimpleSpherePart sSP=null;
  try{
    sSP=(SimpleSpherePart) tmpSim.get(SimulationPartManager.class)
        .getPart(sphereName);
  }catch(NeoException e){
    MeshPartFactory mPF = tmpSim.get(MeshPartFactory.class);
    sSP = mPF.createNewSpherePart(tmpSim.get(SimulationPartManager.class));
    sSP.setPresentationName(sphereName);
  }
  sSP.setDoNotRetessellate(true);
  sSP.setCoordinateSystem(cartCsys);
  Coordinate coordinate_0 = 
    sSP.getOrigin();
  coordinate_0.setCoordinateSystem(cartCsys);
  coordinate_0.setCoordinate(SimTool.getUnitsVec(tmpSim),
      SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
      new DoubleVector(new double[] {0.0, 0.0, 0.0})
      );
  sSP.getRadius().setUnits(SimTool.getUnitsVec(tmpSim));
  sSP.getRadius().setValue(radius);
  sSP.getTessellationDensityOption()
      .setSelected(TessellationDensityOption.Type.MEDIUM);
  sSP.rebuildSimpleShapePart();
  sSP.setDoNotRetessellate(false);
  return sSP;
}

public static SimpleBlockPart makeBlock(Simulation tmpSim, String blockName,
  CoordinateSystem cartCsys, double[] corner1, double[] corner2){
  /* Makes a CAD based simple Block GeometryPart. */
SimpleBlockPart sBP;
try{
  sBP = (SimpleBlockPart) tmpSim.get(SimulationPartManager.class)
      .getPart(blockName);
}catch(NeoException e){
  MeshPartFactory mPF = tmpSim.get(MeshPartFactory.class);
  sBP = mPF.createNewBlockPart(tmpSim.get(SimulationPartManager.class));
  sBP.setPresentationName(blockName);
} 
sBP.setDoNotRetessellate(true);
sBP.setCoordinateSystem(cartCsys);
Coordinate coordinate_0 = 
  sBP.getCorner1();
coordinate_0.setCoordinateSystem(cartCsys);
coordinate_0.setCoordinate(
    SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
    SimTool.getUnitsVec(tmpSim), new DoubleVector(corner1)
    );
Coordinate coordinate_1 = 
  sBP.getCorner2();
coordinate_1.setCoordinateSystem(cartCsys);
coordinate_1.setCoordinate(SimTool.getUnitsVec(tmpSim),
    SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
    new DoubleVector(corner2)
    );
sBP.rebuildSimpleShapePart();
sBP.setDoNotRetessellate(false);
return sBP;
}    

public static SimpleCylinderPart getSimpleCylinder(Simulation tmpSim,
    CoordinateSystem baseCsys, double z1,double z2, double radius){
    /* Makes a CAD based simple Cylinder GeometryPart. */
  MeshPartFactory mPF = tmpSim.get(MeshPartFactory.class);
  SimpleCylinderPart simpleCylinderPart = 
    mPF.createNewCylinderPart(tmpSim.get(SimulationPartManager.class));
  simpleCylinderPart.setDoNotRetessellate(true);

  simpleCylinderPart.setCoordinateSystem(baseCsys);
  //First Coordinate
  Coordinate coordinate_0 = 
    simpleCylinderPart.getStartCoordinate();
  coordinate_0.setCoordinateSystem(baseCsys);
  //side 1
  coordinate_0.setCoordinate(SimTool.getUnitsVec(tmpSim),
      SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
      new DoubleVector(new double[] {0.0, 0.0, z1})
      );
  //side 2
  Coordinate coordinate_1 = 
    simpleCylinderPart.getEndCoordinate();
  coordinate_1.setCoordinateSystem(baseCsys);
  coordinate_1.setCoordinate(SimTool.getUnitsVec(tmpSim),
      SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
      new DoubleVector(new double[] {0.0, 0.0, z2})
      );
  simpleCylinderPart.getRadius().setUnits(SimTool.getUnitsVec(tmpSim));
  simpleCylinderPart.getRadius().setValue(radius);
  simpleCylinderPart.getTessellationDensityOption()
      .setSelected(TessellationDensityOption.Type.MEDIUM);
  simpleCylinderPart.rebuildSimpleShapePart();
  return simpleCylinderPart;
}

public static SimpleConePart getSimpleCone(Simulation tmpSim, 
    CoordinateSystem baseCsys, double z1,double z2, 
    double radius1, double radius2){
    /* Makes a CAD based simple Cone GeometryPart. */
    MeshPartFactory mPF = tmpSim.get(MeshPartFactory.class);
    SimpleConePart simpleConePart = 
      mPF.createNewConePart(tmpSim.get(SimulationPartManager.class));
    simpleConePart.setDoNotRetessellate(true);
    simpleConePart.setCoordinateSystem(baseCsys);
    Coordinate coordinate_2 = simpleConePart.getStartCoordinate();
    coordinate_2.setCoordinateSystem(baseCsys);
    coordinate_2.setCoordinate(SimTool.getUnitsVec(tmpSim),
        SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
        new DoubleVector(new double[] {0.0, 0.0, z1})
        );
    simpleConePart.getStartRadius().setUnits(SimTool.getUnitsVec(tmpSim));
    simpleConePart.getStartRadius().setValue(radius1);

    Coordinate coordinate_3 = simpleConePart.getEndCoordinate();
    coordinate_3.setCoordinateSystem(baseCsys);
    coordinate_3.setCoordinate(SimTool.getUnitsVec(tmpSim),
        SimTool.getUnitsVec(tmpSim), SimTool.getUnitsVec(tmpSim),
        new DoubleVector(new double[] {0.0, 0.0, z2})
        );
    simpleConePart.getEndRadius().setUnits(SimTool.getUnitsVec(tmpSim));
    simpleConePart.getEndRadius().setValue(radius2);
    simpleConePart.getTessellationDensityOption()
        .setSelected(TessellationDensityOption.Type.MEDIUM);
    simpleConePart.rebuildSimpleShapePart();
    simpleConePart.setDoNotRetessellate(false);
    return simpleConePart;
}

public static CadPart getDonut(Simulation tmpSim, CoordinateSystem baseCsys,
    double rmax, double rmin, double zmin, double zmax){
    /* Makes a CAD based cylindrical donut shaped GeometryPart.
       It does this by creating two simply Cylinders, then subtracting them
       from each other. Note that this is a Leaf-level (dumb) CAD geometry.
       Once it is created, it cannot be parametrically modified. */
  SimpleCylinderPart outterRing = 
      getSimpleCylinder(tmpSim, baseCsys, zmin, zmax, rmax);
  SimpleCylinderPart innerRing = 
      getSimpleCylinder(tmpSim, baseCsys, zmin, zmax, rmin);
  ArrayList<MeshPart> rings = new ArrayList();
  rings.add(outterRing);
  rings.add(innerRing);
  CadPart retPart = makeDummyCADSubtractPart(tmpSim,rings,outterRing);
  tmpSim.get(SimulationPartManager.class)
      .removeParts(new NeoObjectVector(
          new Object[] {outterRing, innerRing})
          );
  return retPart;
}

public static CadPart makeDummyCADSubtractPart(Simulation tmpSim,
    Collection<MeshPart> allParts,
    MeshPart fromThisPart){
  MeshActionManager meshActionManager_0 = 
    tmpSim.get(MeshActionManager.class);
  CadPart retPart = 
    (CadPart) meshActionManager_0.
            subtractParts(allParts, fromThisPart,"CAD");
  return retPart;
}

public static void deletePart(Simulation tmpSim, String partName){
  try{
  GeometryPart thisPart = tmpSim.getGeometryPartManager().getObject(partName);
    tmpSim.get(SimulationPartManager.class)
            .removeParts(new NeoObjectVector(
                    new Object[] {thisPart}));
  }catch(NeoException e){
    tmpSim.println("GeometryTool: Tried to find and delete GeometryPart "
        + partName + " but was not found.");
  }
}

private static Collection<GeometryPart> removeMeshOpParts(
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

public static ArrayList<PartSurface> filterPartSurfacesByString(
    GeometryPart tmpPart, String stringID){
  ArrayList<PartSurface> retArr = new ArrayList();
  for(PartSurface tmpSurf : tmpPart.getPartSurfaces()){
    String surfName = tmpSurf.getPresentationName();
    if(surfName.startsWith(stringID) || surfName.contains("."+stringID)){
      retArr.add(tmpSurf);
    }
  }
  return retArr;
}

}// End Class GeometryTool 
