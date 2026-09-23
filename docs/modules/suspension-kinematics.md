---
module: M08
source:
  - core/include/vdsim/suspension.hpp
  - core/src/suspension_lookup.cpp
  - core/include/vdsim/steering_kinematics.hpp
  - core/src/dw_native_kinematics.cpp
  - core/src/fivelink_native_kinematics.cpp
  - core/src/mp_native_kinematics.cpp
  - core/src/ta_native_kinematics.cpp
  - tools/kinematics/
theory: [theory/13_multibody_outlook.md, theory/14_hardpoint_kinematics.md]
tests: []
---
