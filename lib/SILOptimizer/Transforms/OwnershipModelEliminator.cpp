//===--- OwnershipModelEliminator.cpp - Eliminate SILOwnership Instr. -----===//
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
///
///  \file
///
///  This file contains a small pass that lowers SIL ownership instructions to
///  their constituent operations. This will enable us to separate
///  implementation
///  of Semantic ARC in SIL and SILGen from ensuring that all of the optimizer
///  passes respect Semantic ARC. This is done by running this pass right after
///  SILGen and as the pass pipeline is updated, moving this pass further and
///  further back in the pipeline.
///
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "sil-ownership-model-eliminator"
#include "swift/SILOptimizer/PassManager/Transforms.h"
#include "swift/SIL/SILBuilder.h"
#include "swift/SIL/SILFunction.h"
#include "swift/SIL/SILVisitor.h"

using namespace swift;

//===----------------------------------------------------------------------===//
//                               Implementation
//===----------------------------------------------------------------------===//

namespace {

struct OwnershipModelEliminatorVisitor
    : SILInstructionVisitor<OwnershipModelEliminatorVisitor, bool> {
  SILBuilder &B;

  OwnershipModelEliminatorVisitor(SILBuilder &B) : B(B) {}
  void beforeVisit(ValueBase *V) {
    auto *I = cast<SILInstruction>(V);
    B.setInsertionPoint(I);
    B.setCurrentDebugScope(I->getDebugScope());
  }

  bool visitValueBase(ValueBase *V) { return false; }
  bool visitLoadInst(LoadInst *LI);
  bool visitStoreInst(StoreInst *SI);
  bool visitRefCountStoreBarrierInst(RefCountStoreBarrierInst *RCSBI);
  bool visitStoreBorrowInst(StoreBorrowInst *SI);
  bool visitCopyValueInst(CopyValueInst *CVI);
  bool visitCopyUnownedValueInst(CopyUnownedValueInst *CVI);
  bool visitDestroyValueInst(DestroyValueInst *DVI);
  bool visitLoadBorrowInst(LoadBorrowInst *LBI);
  bool visitBeginBorrowInst(BeginBorrowInst *BBI) {
    BBI->replaceAllUsesWith(BBI->getOperand());
    BBI->eraseFromParent();
    return true;
  }
  bool visitEndBorrowInst(EndBorrowInst *EBI) {
    EBI->eraseFromParent();
    return true;
  }
  bool visitUnmanagedRetainValueInst(UnmanagedRetainValueInst *URVI);
  bool visitUnmanagedReleaseValueInst(UnmanagedReleaseValueInst *URVI);
  bool visitUnmanagedAutoreleaseValueInst(UnmanagedAutoreleaseValueInst *UAVI);
};

} // end anonymous namespace

bool OwnershipModelEliminatorVisitor::visitLoadInst(LoadInst *LI) {
  auto Qualifier = LI->getOwnershipQualifier();

  // If the qualifier is unqualified, there is nothing further to do
  // here. Just return.
  if (Qualifier == LoadOwnershipQualifier::Unqualified)
    return false;

  SILValue Result = B.emitLoadValueOperation(LI->getLoc(), LI->getOperand(),
                                             LI->getOwnershipQualifier());

  // Then remove the qualified load and use the unqualified load as the def of
  // all of LI's uses.
  LI->replaceAllUsesWith(Result);
  LI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitStoreInst(StoreInst *SI) {
  auto Qualifier = SI->getOwnershipQualifier();

  // If the qualifier is unqualified, there is nothing further to do
  // here. Just return.
  if (Qualifier == StoreOwnershipQualifier::Unqualified)
    return false;

  B.emitStoreValueOperation(SI->getLoc(), SI->getSrc(), SI->getDest(),
                            SI->getOwnershipQualifier());

  // Then remove the qualified store.
  SI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitRefCountStoreBarrierInst(RefCountStoreBarrierInst *RCSBI) { // dmu
  abort(); // TODO: (dmu urgent) what to do here?
}

bool OwnershipModelEliminatorVisitor::visitStoreBorrowInst(
    StoreBorrowInst *SI) {
  B.emitStoreValueOperation(SI->getLoc(), SI->getSrc(), SI->getDest(),
                            StoreOwnershipQualifier::Init);

  // Then remove the qualified store.
  SI->eraseFromParent();
  return true;
}

bool
OwnershipModelEliminatorVisitor::visitLoadBorrowInst(LoadBorrowInst *LBI) {
  // Break down the load borrow into an unqualified load.
  auto *UnqualifiedLoad = B.createLoad(LBI->getLoc(), LBI->getOperand(),
                                       LoadOwnershipQualifier::Unqualified);

  // Then remove the qualified load and use the unqualified load as the def of
  // all of LI's uses.
  LBI->replaceAllUsesWith(UnqualifiedLoad);
  LBI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitCopyValueInst(CopyValueInst *CVI) {
  // Now that we have set the unqualified ownership flag, destroy value
  // operation will delegate to the appropriate strong_release, etc.
  B.emitCopyValueOperation(CVI->getLoc(), CVI->getOperand());
  CVI->replaceAllUsesWith(CVI->getOperand());
  CVI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitCopyUnownedValueInst(
    CopyUnownedValueInst *CVI) {
  B.createStrongRetainUnowned(CVI->getLoc(), CVI->getOperand(),
                              Atomicity::Atomic);
  // Users of copy_value_unowned expect an owned value. So we need to convert
  // our unowned value to a ref.
  auto *UTRI =
      B.createUnownedToRef(CVI->getLoc(), CVI->getOperand(), CVI->getType());
  CVI->replaceAllUsesWith(UTRI);
  CVI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitUnmanagedRetainValueInst(
    UnmanagedRetainValueInst *URVI) {
  // Now that we have set the unqualified ownership flag, destroy value
  // operation will delegate to the appropriate strong_release, etc.
  B.emitCopyValueOperation(URVI->getLoc(), URVI->getOperand());
  URVI->replaceAllUsesWith(URVI->getOperand());
  URVI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitUnmanagedReleaseValueInst(
    UnmanagedReleaseValueInst *URVI) {
  // Now that we have set the unqualified ownership flag, destroy value
  // operation will delegate to the appropriate strong_release, etc.
  B.emitDestroyValueOperation(URVI->getLoc(), URVI->getOperand());
  URVI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitUnmanagedAutoreleaseValueInst(
    UnmanagedAutoreleaseValueInst *UAVI) {
  // Now that we have set the unqualified ownership flag, destroy value
  // operation will delegate to the appropriate strong_release, etc.
  B.createAutoreleaseValue(UAVI->getLoc(), UAVI->getOperand(),
                           Atomicity::Atomic);
  UAVI->eraseFromParent();
  return true;
}

bool OwnershipModelEliminatorVisitor::visitDestroyValueInst(DestroyValueInst *DVI) {
  // Now that we have set the unqualified ownership flag, destroy value
  // operation will delegate to the appropriate strong_release, etc.
  B.emitDestroyValueOperation(DVI->getLoc(), DVI->getOperand());
  DVI->eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
//                           Top Level Entry Point
//===----------------------------------------------------------------------===//

namespace {

struct OwnershipModelEliminator : SILFunctionTransform {
  void run() override {
    SILFunction *F = getFunction();

    // Set F to have unqualified ownership.
    F->setUnqualifiedOwnership();

    bool MadeChange = false;
    SILBuilder B(*F);
    OwnershipModelEliminatorVisitor Visitor(B);

    for (auto &BB : *F) {
      for (auto II = BB.begin(), IE = BB.end(); II != IE;) {
        // Since we are going to be potentially removing instructions, we need
        // to make sure to grab out instruction and increment first.
        SILInstruction *I = &*II;
        ++II;

        MadeChange |= Visitor.visit(I);
      }
    }

    if (MadeChange) {
      // If we made any changes, we just changed instructions, so invalidate
      // that analysis.
      invalidateAnalysis(SILAnalysis::InvalidationKind::Instructions);
    }
  }

  StringRef getName() override { return "Ownership Model Eliminator"; }
};

} // end anonymous namespace

SILTransform *swift::createOwnershipModelEliminator() {
  return new OwnershipModelEliminator();
}
