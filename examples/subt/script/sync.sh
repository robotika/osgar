#!/bin/bash

cp osgar/examples/subt/src/subt_example_node.cc src/subt/subt_example/src/subt_example_node.cc
cp osgar/examples/subt/src/CMakeLists.txt src/subt/subt_example/CMakeLists.txt
cp osgar/examples/subt/launch/x2lr_team.launch src/subt/subt_example/launch/x2lr_team.launch
cp osgar/examples/subt/config/robot_config.yaml src/subt/subt_example/config/robot_config.yaml
sed -i 's/name="gui" value="true"/name="gui" value="false"/' src/subt/subt_gazebo/launch/competition.launch

patch -d src/subt --strip 1 --forward <<END
diff -r 27079e732f94 subt_gazebo/src/GameLogicPlugin.cc
--- a/subt_gazebo/src/GameLogicPlugin.cc        Tue Feb 19 20:55:29 2019 +0000
+++ b/subt_gazebo/src/GameLogicPlugin.cc        Fri Mar 22 14:10:29 2019 +0100
@@ -305,6 +305,9 @@
     if (potentialArtifacts.empty())
       this->artifacts.erase(_type);
   }
+  gzmsg << "objname:" << std::get<0>(minDistance) << std::endl;
+  gzmsg << "objPosition:" << std::get<1>(minDistance) << std::endl;
+  gzmsg << "distance:" << std::get<2>(minDistance) << std::endl;

   gzmsg << "  [Total]: " << score << std::endl;
   this->Log() << "modified_score " << score << std::endl;
END

catkin_make install

