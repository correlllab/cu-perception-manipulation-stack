FILE(REMOVE_RECURSE
  "CMakeFiles/perception_gencfg"
  "devel/include/perception/perception_paramConfig.h"
  "devel/share/perception/docs/perception_paramConfig.dox"
  "devel/share/perception/docs/perception_paramConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/perception/cfg/perception_paramConfig.py"
  "devel/share/perception/docs/perception_paramConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/perception_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
