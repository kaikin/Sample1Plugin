
option(BUILD_SAMPLE1_SAMPLE "Building a sample plugin \"Sample1Plugin\"" OFF)

if(BUILD_SAMPLE1_SAMPLE)
  set(target CnoidSample1Plugin)
  add_cnoid_plugin(${target} SHARED Sample1Plugin.cpp SampleView.cpp SampleView.h database.h)
  target_link_libraries(${target} CnoidBodyPlugin)
  apply_common_setting_for_plugin(${target})
endif()

install_sample_source(Sample1Plugin.cpp SampleView.cpp SampleView.h database.h)
