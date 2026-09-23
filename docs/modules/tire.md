---
module: M03
source:
  - core/include/vdsim/magic_formula.hpp
  - core/src/tire_model.cpp
  - core/src/magic_formula_tire.cpp
  - core/src/pacejka_mf96.cpp
  - core/src/linear_tire.cpp
  - core/include/vdsim/lugre_tire.hpp
  - core/include/vdsim/belt_tire.hpp
  - core/include/vdsim/tire_contact.hpp
  - python/tir_to_yaml.py
theory: [theory/03_tire_pacejka_mf96.md, theory/19_lugre_dynamic_tire.md, theory/21_belt_transient.md, theory/25_tire_contact_interface.md]
tests: []
---
