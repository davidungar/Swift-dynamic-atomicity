//===--- DiagnosticsIRGen.h - Diagnostic Definitions ------------*- C++ -*-===//
//
// This source file is part of the Swift.org open source project
//
// Copyright (c) 2014 - 2017 Apple Inc. and the Swift project authors
// Licensed under Apache License v2.0 with Runtime Library Exception
//
// See https://swift.org/LICENSE.txt for license information
// See https://swift.org/CONTRIBUTORS.txt for the list of Swift project authors
//
//===----------------------------------------------------------------------===//
//
/// \file
/// \brief This file defines diagnostics for IR generation.
//
//===----------------------------------------------------------------------===//

#ifndef SWIFT_DIAGNOSTICSIRGEN_H
#define SWIFT_DIAGNOSTICSIRGEN_H

#include "swift/AST/DiagnosticsCommon.h"

// TODO: (dmu) turn into a run-time command flag
# define GENERATE_DYNAMIC_ATOMICITY_DIAGNOSTICS_DMU_ 0

namespace swift {
  namespace diag {
  // Declare common diagnostics objects with their appropriate types.
#define DIAG(KIND,ID,Options,Text,Signature) \
  extern detail::DiagWithArguments<void Signature>::type ID;
#include "DiagnosticsIRGen.def"
  }
}

#endif
