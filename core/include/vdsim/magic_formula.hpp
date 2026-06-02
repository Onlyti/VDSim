// Full Magic Formula (Pacejka 2002 / MF-Tyre 5.2 coefficient set) tire model.
//
// Reads a standard `.tir` property file (section-aware key = value) into a
// coefficient map and evaluates pure + combined slip Fx/Fy and aligning Mz.
// Turn-slip, inflation-pressure and thermal effects are out of scope (this
// coefficient set is isothermal and has no such blocks).
//
// NOTE: tire coefficient *values* loaded from a `.tir` may be confidential.
// Keep `.tir` files and any derived coefficient dumps out of the repository
// (see `.gitignore`).  Only the evaluation equations live in source.
#pragma once

#include <cctype>
#include <memory>
#include <string>
#include <unordered_map>

#include "vdsim/interfaces.hpp"

namespace vdsim {

// Parsed Magic Formula coefficient set.  Stored as a flat name->value map so
// the parser is agnostic to which optional coefficients a given .tir provides;
// the evaluator pulls each coefficient by name with an explicit default.
struct MFCoeffs {
    std::unordered_map<std::string, double> p;

    // Case-insensitive lookup with default (missing coefficient -> default).
    // Keys are stored uppercased by parse_tir; uppercase the query to match.
    static std::string up(const char* key) {
        std::string s(key);
        for (auto& c : s) c = static_cast<char>(std::toupper(c));
        return s;
    }
    double g(const char* key, double def) const {
        auto it = p.find(up(key));
        return (it != p.end()) ? it->second : def;
    }
    bool has(const char* key) const { return p.find(up(key)) != p.end(); }
};

// Parse a .tir file into MFCoeffs.  Throws std::runtime_error if the file
// cannot be opened.  Unknown sections/keys are ignored; comments after '$'
// (and '!') and bracketed [SECTION] headers are skipped.
MFCoeffs parse_tir(const std::string& tir_path);

// Construct an ITireModel from a coefficient set.
std::unique_ptr<ITireModel> create_magic_formula_tire(const MFCoeffs& coeffs);

// Convenience: parse a .tir and build the model in one call.
std::unique_ptr<ITireModel> create_magic_formula_tire_from_tir(const std::string& tir_path);

}  // namespace vdsim
