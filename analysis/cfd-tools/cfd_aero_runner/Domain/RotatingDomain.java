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

import java.util.ArrayList;
import java.util.Collection;

import star.common.*;

import GeometricObject.*;

public class RotatingDomain extends Domain{
    
    //Extension of string names
    String rotatingStr;
    
    //Tracking Domains that the Rotor is interfaced with
    ArrayList<Domain> interfacedWithDomains=new ArrayList();

    //Track what Objects are inside this domain
    Rotor domainRotor;
    
    //Coordinate system
    CartesianCoordinateSystem localCsys;

    //Rotor clocking
    double clockingAngle=0.0; // [deg]
    
    public RotatingDomain(Simulation tmpSim,GeometryPart geomPart){
        super(tmpSim,geomPart);
        //Identification
       this.rotatingStr=globalNames.getSlidingStr();
    }
    
    //GEOMETRY & OBJECTS
    public void assignRotor(Rotor tmpObj){
        domainRotor=tmpObj;
    }
    public Rotor getRotor(){
        return domainRotor;
    }
    
    // 
    public void setCoordinateSystem(CartesianCoordinateSystem tmpCsys){
        localCsys = tmpCsys;
    }

    //REGIONS
    public void slidingInterfaceToDomain(Domain intToDomain){
        domainRegion.getSimulation().println("Rotor MSG: Interfacing to "+intToDomain.getName());
        BoundaryInterface slidingInterface=null;
        
        boolean doIcreateInterface=true;
        
        String myRegName = domainRegion.getPresentationName();
        String toRegName = intToDomain.getRegion().getPresentationName();
        String newInterfaceName;
        if(intToDomain instanceof OversetDomain){
            newInterfaceName = myRegName+"_to_"+toRegName;
        }else{
            newInterfaceName = myRegName+"_to_Background";   
        }
        domainRegion.getSimulation().println("Rotor MSG: Interface name "+newInterfaceName);
        //make sure that the interface doesn't already exist
        Collection<Interface> allInterfaces=domainRegion.getSimulation().getInterfaceManager().getObjects();
        for(Interface tmpInterface:allInterfaces){
            //check whether it has an existing in-place type
            if(tmpInterface instanceof BoundaryInterface){
                Region tmpReg0=tmpInterface.getRegion0();
                Region tmpReg1=tmpInterface.getRegion1();
                
                //check if the interface already exists
                if((tmpReg0==domainRegion&&tmpReg1==intToDomain.getRegion())||
                  (tmpReg1==domainRegion&&tmpReg0==intToDomain.getRegion())){
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
                    doIcreateInterface=false;
                    break;
                }
            }
        }
        //
        //create the Sliding interface
        if(doIcreateInterface){
            String bndyName=globalNames.getBndyName(globalNames.getSlidingStr());
            domainRegion.getSimulation().println("Rtr MSG: Existing Interface Not Detected, Creating.");
            Boundary intToDomainBndy = intToDomain.getRegion()
                    .getBoundaryManager().getBoundary(bndyName);
            Boundary thisDomainBndy = domainRegion
                    .getBoundaryManager().getBoundary(bndyName);
            
            slidingInterface = 
              domainRegion.getSimulation().getInterfaceManager().createBoundaryInterface(
                      intToDomainBndy, thisDomainBndy,
                      "Sliding Interface");
            slidingInterface.setPresentationName(newInterfaceName);   
            interfacedWithDomains.add(intToDomain);
        }
        
    }


    public double getDomainAngle(){
        return clockingAngle;
    }
    

    
    
    
   
}
