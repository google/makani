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
package Tools;

import java.util.*;
import star.common.*;
import star.base.neo.*;

public class TagTool {

public static UserTag getTag(Simulation tmpSim, String tagName){
  UserTag retTag;
  try{
    retTag = (UserTag) tmpSim.get(TagManager.class).getObject(tagName);
  }catch(NeoException e){
    retTag = tmpSim.get(TagManager.class).createNewUserTag(tagName);
    StarImage starImage_0 = retTag.getCustomizableIcon();
    starImage_0.setStarImagePixelData(
        new IntVector(
           new int[] {16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 142639232, 1753186175, -947879808,
              -142573440, -8355712, -746553216, 1887469696, 209748096,
              0, 0, 0, 0, 0, 0, 0, 545292416, -679444352, -8101021,
              -7785677, -7594980, -7594980, -7785677, -8101535,
              -612335488, 545292416, 0, 0, 0, 0, 0, 142639232,
              -679444352, -7974832, -7595237, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7976631, -612335488,
              209748096, 0, 0, 0, 0, 1753186175, -8101021, -7595237,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7595751, -7595751, -8101535, 1887469696, 0, 0, 0, 0,
              -880770944, -7785677, -7595751, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7785677, -746553216, 0, 0, 0, 0, -75464576, -7594980,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7594980, -8355712, 0, 0,
              0, 0, -75464576, -7594980, -7595751, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7594980, -8355712, 0, 0, 0, 0, -880770944, -7785677,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7785677, -746553216,
              0, 0, 0, 0, 1753186175, -8101021, -7595237, -7595751,
              -7595751, -7595751, -7595751, -7595751, -7595751,
              -7595751, -8101535, 1887469696, 0, 0, 0, 0, 142639232,
              -679444352, -7974832, -7595751, -7595751, -7595751,
              -7595751, -7595751, -7595751, -7976631, -612335488,
              209748096, 0, 0, 0, 0, 0, 545292416, -679444352,
              -8101021, -7785677, -7594980, -7594980, -7785677,
              -8101535, -612335488, 545292416, 0, 0, 0, 0, 0, 0, 0,
              142639232, 1753186175, -880770944, -75464576, -8355712,
              -679444352, 1954578560, 209748096, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  }

  return retTag;
}

public static Collection<Tag> getSimulationTags(Simulation tmpSim){
  return tmpSim.get(TagManager.class).getObjects();
}

public static ArrayList<Tag> getSimulationTagsByPrefix(Simulation tmpSim,
    String preFix){
  ArrayList<Tag> retTags = new ArrayList();
  for(Tag tmpTag : getSimulationTags(tmpSim)){
    if(tmpTag.getPresentationName().startsWith(preFix)){
      retTags.add(tmpTag);
    }
  }
  return retTags;
}
  
public static Collection<Tag> getObjectTags(NamedObject thisObject){
  return thisObject.getTagsInput().getAllSelectedInstances();
}

public static ArrayList<Tag> getObjectTagsByPrefix(NamedObject thisObject,
    String preFix){
  ArrayList<Tag> retTags = new ArrayList();
  for(Tag tmpTag : getObjectTags(thisObject)){
    if(tmpTag.getPresentationName().startsWith(preFix)){
      retTags.add(tmpTag);
    }
  }
  return retTags;
}

public static void removeTag(Simulation tmpSim, NamedObject thisObject,
    Collection<Tag> theseTags){
  tmpSim.getTagManager().removeTags(tmpSim, theseTags);
}
  
  public static void addTags(Simulation tmpSim, NamedObject toObject,
      Collection<Tag> theseTags){
    tmpSim.getTagManager().addTags(toObject, theseTags);
  }
  
  public static void addTag(Simulation tmpSim, NamedObject toObject,
      Tag thisTag){
    tmpSim.getTagManager().addTags(toObject, Collections.singleton(thisTag));
  }
  
  public static void addTag(Simulation tmpSim,
      Collection<NamedObject> toObjects, Tag thisTag){
    for(NamedObject toObject:toObjects){
      addTag(tmpSim, toObject, thisTag);
    }
  }
  
  public static ArrayList<NamedObject> filterObjectsByTag(Simulation tmpSim,
      Collection<NamedObject> theseObjects, Tag thisTag){
    ArrayList<NamedObject> retNamedObjects = new ArrayList();
    for(NamedObject tmpNamedObject:theseObjects){
      Collection<Tag> tagsApplied = 
          tmpNamedObject.getTagsInput().getSelectedInstances();
      if(tagsApplied.contains(thisTag)){
        retNamedObjects.add(tmpNamedObject);
      }
    }
    return retNamedObjects;
  }

}// END CLASS TAGTOOL
