include(GoogleTest)

add_subdirectory(unit)
add_subdirectory(integration)
# validation/ 은 W11+ 에서 활성화

# Documentation currency gate (07_p0_demo.md 6.5(3)): the parser/comparator that
# CI uses to reject a stale docs/VALIDATION.md is itself unit-tested here, so a
# broken gate fails the suite instead of silently passing every document.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME validation_currency
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_validation_currency.py)
    set_tests_properties(validation_currency PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
