//===--- SILGenDecl.cpp - Implements Lowering of ASTs -> SIL for Decls ----===//
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

#include "SILGen.h"
#include "Initialization.h"
#include "RValue.h"
#include "Scope.h"
#include "SILGenDynamicCast.h"
#include "swift/SIL/FormalLinkage.h"
#include "swift/SIL/PrettyStackTrace.h"
#include "swift/SIL/SILArgument.h"
#include "swift/SIL/SILDebuggerClient.h"
#include "swift/SIL/SILType.h"
#include "swift/SIL/SILWitnessVisitor.h"
#include "swift/SIL/TypeLowering.h"
#include "swift/AST/AST.h"
#include "swift/AST/GenericEnvironment.h"
#include "swift/AST/Mangle.h"
#include "swift/AST/ASTMangler.h"
#include "swift/AST/Module.h"
#include "swift/AST/NameLookup.h"
#include "swift/AST/ProtocolConformance.h"
#include "swift/Basic/Fallthrough.h"
#include "llvm/ADT/SmallString.h"
#include <iterator>
using namespace swift;
using namespace Mangle;
using namespace Lowering;

void Initialization::_anchor() {}
void SILDebuggerClient::anchor() {}

namespace {
  /// A "null" initialization that indicates that any value being initialized
  /// into this initialization should be discarded. This represents AnyPatterns
  /// (that is, 'var (_)') that bind to values without storing them.
  class BlackHoleInitialization : public Initialization {
  public:
    BlackHoleInitialization() {}

    SILValue getAddressOrNull() const override { return SILValue(); }

    bool canSplitIntoTupleElements() const override {
      return true;
    }
    
    MutableArrayRef<InitializationPtr>
    splitIntoTupleElements(SILGenFunction &gen, SILLocation loc,
                           CanType type,
                           SmallVectorImpl<InitializationPtr> &buf) override {
      // "Destructure" an ignored binding into multiple ignored bindings.
      for (auto fieldType : cast<TupleType>(type)->getElementTypes()) {
        (void) fieldType;
        buf.push_back(InitializationPtr(new BlackHoleInitialization()));
      }
      return buf;
    }

    void copyOrInitValueInto(SILGenFunction &gen, SILLocation loc,
                             ManagedValue value, bool isInit) override {
      /// This just ignores the provided value.
    }

    void finishUninitialized(SILGenFunction &gen) override {
      // do nothing
    }
  };
} // end anonymous namespace

void TupleInitialization::copyOrInitValueInto(SILGenFunction &SGF,
                                              SILLocation loc,
                                              ManagedValue valueMV,
                                              bool isInit) {
  // A scalar value is being copied into the tuple, break it into elements
  // and assign/init each element in turn.
  SILValue value = valueMV.forward(SGF);
  auto sourceType = cast<TupleType>(valueMV.getSwiftType());
  auto sourceSILType = value->getType();
  for (unsigned i = 0, e = sourceType->getNumElements(); i != e; ++i) {
    SILType fieldTy = sourceSILType.getTupleElementType(i);
    auto &fieldTL = SGF.getTypeLowering(fieldTy);
        
    SILValue member;
    if (value->getType().isAddress()) {
      member = SGF.B.createTupleElementAddr(loc, value, i, fieldTy);
      if (!fieldTL.isAddressOnly())
        member =
            fieldTL.emitLoad(SGF.B, loc, member, LoadOwnershipQualifier::Take);
    } else {
      member = SGF.B.createTupleExtract(loc, value, i, fieldTy);
    }
        
    auto elt = SGF.emitManagedRValueWithCleanup(member, fieldTL);
        
    SubInitializations[i]->copyOrInitValueInto(SGF, loc, elt, isInit);
    SubInitializations[i]->finishInitialization(SGF);
  }
}

void TupleInitialization::finishUninitialized(SILGenFunction &gen) {
  for (auto &subInit : SubInitializations) {
    subInit->finishUninitialized(gen);
  }
}

namespace {
  class CleanupClosureConstant : public Cleanup {
    SILValue closure;
  public:
    CleanupClosureConstant(SILValue closure) : closure(closure) {}
    void emit(SILGenFunction &gen, CleanupLocation l) override {
      gen.B.emitDestroyValueOperation(l, closure);
    }
    void dump() const override {
#ifndef NDEBUG
      llvm::errs() << "CleanupClosureConstant\n"
                   << "State:" << getState() << "\n"
                   << "closure:" << closure << "\n";
#endif
    }
  };
} // end anonymous namespace

SubstitutionList SILGenFunction::getForwardingSubstitutions() {
  return F.getForwardingSubstitutions();
}

void SILGenFunction::visitFuncDecl(FuncDecl *fd) {
  // Generate the local function body.
  SGM.emitFunction(fd);
}

MutableArrayRef<InitializationPtr>
SingleBufferInitialization::
splitIntoTupleElements(SILGenFunction &gen, SILLocation loc, CanType type,
                       SmallVectorImpl<InitializationPtr> &buf) {
  assert(SplitCleanups.empty() && "getting sub-initializations twice?");
  return splitSingleBufferIntoTupleElements(gen, loc, type, getAddress(), buf,
                                            SplitCleanups);
}

MutableArrayRef<InitializationPtr>
SingleBufferInitialization::
splitSingleBufferIntoTupleElements(SILGenFunction &gen, SILLocation loc,
                                   CanType type, SILValue baseAddr,
                                   SmallVectorImpl<InitializationPtr> &buf,
                     TinyPtrVector<CleanupHandle::AsPointer> &splitCleanups) {
  // Destructure the buffer into per-element buffers.
  for (auto i : indices(cast<TupleType>(type)->getElementTypes())) {
    // Project the element.
    SILValue eltAddr = gen.B.createTupleElementAddr(loc, baseAddr, i);

    // Create an initialization to initialize the element.
    auto &eltTL = gen.getTypeLowering(eltAddr->getType());
    auto eltInit = gen.useBufferAsTemporary(eltAddr, eltTL);

    // Remember the element cleanup.
    auto eltCleanup = eltInit->getInitializedCleanup();
    if (eltCleanup.isValid())
      splitCleanups.push_back(eltCleanup);

    buf.emplace_back(eltInit.release());
  }

  return buf;
}

void SingleBufferInitialization::
copyOrInitValueIntoSingleBuffer(SILGenFunction &gen, SILLocation loc,
                                ManagedValue value, bool isInit,
                                SILValue destAddr) {
  if (!isInit) {
    assert(value.getValue() != destAddr && "copying in place?!");
    value.copyInto(gen, destAddr, loc);
    return;
  }
  
  // If we didn't evaluate into the initialization buffer, do so now.
  if (value.getValue() != destAddr) {
    value.forwardInto(gen, loc, destAddr);
  } else {
    // If we did evaluate into the initialization buffer, disable the
    // cleanup.
    value.forwardCleanup(gen);
  }
}

void SingleBufferInitialization::finishInitialization(SILGenFunction &gen) {
  // Forward all of the split element cleanups, assuming we made any.
  for (CleanupHandle eltCleanup : SplitCleanups)
    gen.Cleanups.forwardCleanup(eltCleanup);
}

void KnownAddressInitialization::anchor() const {
}

void TemporaryInitialization::finishInitialization(SILGenFunction &gen) {
  SingleBufferInitialization::finishInitialization(gen);
  if (Cleanup.isValid())
    gen.Cleanups.setCleanupState(Cleanup, CleanupState::Active);
}

namespace {
class EndBorrowCleanup : public Cleanup {
  SILValue original;
  SILValue borrowed;

public:
  EndBorrowCleanup(SILValue original, SILValue borrowed)
      : original(original), borrowed(borrowed) {}

  void emit(SILGenFunction &gen, CleanupLocation l) override {
    gen.B.createEndBorrow(l, borrowed, original);
  }

  void dump() const override {
#ifndef NDEBUG
    llvm::errs() << "EndBorrowCleanup "
                 << "State:" << getState() << "\n"
                 << "original:" << original << "\n"
                 << "borrowed:" << borrowed << "\n";
#endif
  }
};
} // end anonymous namespace

namespace {
class ReleaseValueCleanup : public Cleanup {
  SILValue v;
public:
  ReleaseValueCleanup(SILValue v) : v(v) {}

  void emit(SILGenFunction &gen, CleanupLocation l) override {
    if (v->getType().isAddress())
      gen.B.createDestroyAddr(l, v);
    else
      gen.B.emitDestroyValueOperation(l, v);
  }

  void dump() const override {
#ifndef NDEBUG
    llvm::errs() << "ReleaseValueCleanup\n"
                 << "State:" << getState() << "\n"
                 << "Value:" << v << "\n";
#endif
  }
};
} // end anonymous namespace

namespace {
/// Cleanup to destroy an initialized variable.
class DeallocStackCleanup : public Cleanup {
  SILValue Addr;
public:
  DeallocStackCleanup(SILValue addr) : Addr(addr) {}

  void emit(SILGenFunction &gen, CleanupLocation l) override {
    gen.B.createDeallocStack(l, Addr);
  }

  void dump() const override {
#ifndef NDEBUG
    llvm::errs() << "DeallocStackCleanup\n"
                 << "State:" << getState() << "\n"
                 << "Addr:" << Addr << "\n";
#endif
  }
};
} // end anonymous namespace

namespace {
/// Cleanup to destroy an initialized 'var' variable.
class DestroyLocalVariable : public Cleanup {
  VarDecl *Var;
public:
  DestroyLocalVariable(VarDecl *var) : Var(var) {}

  void emit(SILGenFunction &gen, CleanupLocation l) override {
    gen.destroyLocalVariable(l, Var);
  }

  void dump() const override {
#ifndef NDEBUG
    llvm::errs() << "DestroyLocalVariable\n"
                 << "State:" << getState() << "\n";
    // TODO: Make sure we dump var.
    llvm::errs() << "\n";
#endif
  }
};
} // end anonymous namespace

namespace {
/// Cleanup to destroy an uninitialized local variable.
class DeallocateUninitializedLocalVariable : public Cleanup {
  VarDecl *Var;
public:
  DeallocateUninitializedLocalVariable(VarDecl *var) : Var(var) {}

  void emit(SILGenFunction &gen, CleanupLocation l) override {
    gen.deallocateUninitializedLocalVariable(l, Var);
  }

  void dump() const override {
#ifndef NDEBUG
    llvm::errs() << "DeallocateUninitializedLocalVariable\n"
                 << "State:" << getState() << "\n";
    // TODO: Make sure we dump var.
    llvm::errs() << "\n";
#endif
  }
};
} // end anonymous namespace

namespace {
/// An initialization of a local 'var'.
class LocalVariableInitialization : public SingleBufferInitialization {
  /// The local variable decl being initialized.
  VarDecl *decl;
  SILGenFunction &SGF;

  /// The cleanup we pushed to deallocate the local variable before it
  /// gets initialized.
  CleanupHandle DeallocCleanup;

  /// The cleanup we pushed to destroy and deallocate the local variable.
  CleanupHandle ReleaseCleanup;

  bool DidFinish = false;
public:
  /// Sets up an initialization for the allocated box. This pushes a
  /// CleanupUninitializedBox cleanup that will be replaced when
  /// initialization is completed.
  LocalVariableInitialization(VarDecl *decl, bool NeedsMarkUninit,
                              unsigned ArgNo, SILGenFunction &SGF)
      : decl(decl), SGF(SGF) {
    assert(decl->getDeclContext()->isLocalContext() &&
           "can't emit a local var for a non-local var decl");
    assert(decl->hasStorage() && "can't emit storage for a computed variable");
    assert(!SGF.VarLocs.count(decl) && "Already have an entry for this decl?");

    auto boxType = SGF.SGM.Types
      .getContextBoxTypeForCapture(decl,
                     SGF.getLoweredType(decl->getType()).getSwiftRValueType(),
                     SGF.F.getGenericEnvironment(),
                     /*mutable*/ true);

    // The variable may have its lifetime extended by a closure, heap-allocate
    // it using a box.
    AllocBoxInst *allocBox =
        SGF.B.createAllocBox(decl, boxType, {decl->isLet(), ArgNo});
    SILValue addr = SGF.B.createProjectBox(decl, allocBox, 0);

    // Mark the memory as uninitialized, so DI will track it for us.
    if (NeedsMarkUninit)
      addr = SGF.B.createMarkUninitializedVar(decl, addr);

    /// Remember that this is the memory location that we're emitting the
    /// decl to.
    SGF.VarLocs[decl] = SILGenFunction::VarLoc::get(addr, allocBox);

    // Push a cleanup to destroy the local variable.  This has to be
    // inactive until the variable is initialized.
    SGF.Cleanups.pushCleanupInState<DestroyLocalVariable>(CleanupState::Dormant,
                                                          decl);
    ReleaseCleanup = SGF.Cleanups.getTopCleanup();

    // Push a cleanup to deallocate the local variable.
    SGF.Cleanups.pushCleanup<DeallocateUninitializedLocalVariable>(decl);
    DeallocCleanup = SGF.Cleanups.getTopCleanup();
  }

  ~LocalVariableInitialization() override {
    assert(DidFinish && "did not call VarInit::finishInitialization!");
  }

  SILValue getAddressOrNull() const override {
    assert(SGF.VarLocs.count(decl) && "did not emit var?!");
    return SGF.VarLocs[decl].value;
  }

  void finishUninitialized(SILGenFunction &gen) override {
    LocalVariableInitialization::finishInitialization(gen);
  }

  void finishInitialization(SILGenFunction &SGF) override {
    SingleBufferInitialization::finishInitialization(SGF);
    assert(!DidFinish &&
           "called LocalVariableInitialization::finishInitialization twice!");
    SGF.Cleanups.setCleanupState(DeallocCleanup, CleanupState::Dead);
    SGF.Cleanups.setCleanupState(ReleaseCleanup, CleanupState::Active);
    DidFinish = true;
  }
};
} // end anonymous namespace

namespace {
/// Initialize a writeback buffer that receives the value of a 'let'
/// declaration.
class LetValueInitialization : public Initialization {
  /// The VarDecl for the let decl.
  VarDecl *vd;

  /// The address of the buffer used for the binding, if this is an address-only
  /// let.
  SILValue address;

  /// The cleanup we pushed to destroy the local variable.
  CleanupHandle DestroyCleanup;

  /// Cleanups we introduced when splitting.
  TinyPtrVector<CleanupHandle::AsPointer> SplitCleanups;

  bool DidFinish = false;

public:
  LetValueInitialization(VarDecl *vd, SILGenFunction &gen) : vd(vd)
  {
    auto &lowering = gen.getTypeLowering(vd->getType());
    
    // Decide whether we need a temporary stack buffer to evaluate this 'let'.
    // There are three cases we need to handle here: parameters, initialized (or
    // bound) decls, and uninitialized ones.
    bool needsTemporaryBuffer;
    bool isUninitialized = false;

    assert(!isa<ParamDecl>(vd)
           && "should not bind function params on this path");
    if (vd->getParentPatternBinding() && !vd->getParentInitializer()) {
      // This value is uninitialized (and unbound) if it has a pattern binding
      // decl, with no initializer value.
      assert(!vd->hasNonPatternBindingInit() && "Bound values aren't uninit!");
      
      // If this is a let-value without an initializer, then we need a temporary
      // buffer.  DI will make sure it is only assigned to once.
      needsTemporaryBuffer = true;
      isUninitialized = true;
    } else {
      // If this is a let with an initializer or bound value, we only need a
      // buffer if the type is address only.
      needsTemporaryBuffer = lowering.isAddressOnly();
    }
   
    if (needsTemporaryBuffer) {
      address = gen.emitTemporaryAllocation(vd, lowering.getLoweredType());
      if (isUninitialized)
        address = gen.B.createMarkUninitializedVar(vd, address);
      DestroyCleanup = gen.enterDormantTemporaryCleanup(address, lowering);
      gen.VarLocs[vd] = SILGenFunction::VarLoc::get(address);
    } else if (!lowering.isTrivial()) {
      // Push a cleanup to destroy the let declaration.  This has to be
      // inactive until the variable is initialized: if control flow exits the
      // before the value is bound, we don't want to destroy the value.
      gen.Cleanups.pushCleanupInState<DestroyLocalVariable>(
                                                    CleanupState::Dormant, vd);
      DestroyCleanup = gen.Cleanups.getTopCleanup();
    } else {
      DestroyCleanup = CleanupHandle::invalid();
    }
  }

  ~LetValueInitialization() override {
    assert(DidFinish && "did not call LetValueInit::finishInitialization!");
  }

  bool hasAddress() const { return (bool)address; }
  
  // SingleBufferInitializations always have an address.
  SILValue getAddressForInPlaceInitialization() const override {
    // Emit into the buffer that 'let's produce for address-only values if
    // we have it.
    if (hasAddress()) return address;
    return SILValue();
  }

  /// Return true if we can get the addresses of elements with the
  /// 'getSubInitializationsForTuple' method.
  ///
  /// Let-value initializations cannot be broken into constituent pieces if a
  /// scalar value needs to be bound.  If there is an address in play, then we
  /// can initialize the address elements of the tuple though.
  bool canSplitIntoTupleElements() const override {
    return hasAddress();
  }
  
  MutableArrayRef<InitializationPtr>
  splitIntoTupleElements(SILGenFunction &gen, SILLocation loc, CanType type,
                         SmallVectorImpl<InitializationPtr> &buf) override {
    assert(SplitCleanups.empty());
    return SingleBufferInitialization
       ::splitSingleBufferIntoTupleElements(gen, loc, type, getAddress(), buf,
                                            SplitCleanups);
  }

  SILValue getAddressOrNull() const override {
    return address;
  }

  void bindValue(SILValue value, SILGenFunction &gen) {
    assert(!gen.VarLocs.count(vd) && "Already emitted this vardecl?");
    // If we're binding an address to this let value, then we can use it as an
    // address later.  This happens when binding an address only parameter to
    // an argument, for example.
    if (value->getType().isAddress())
      address = value;
    gen.VarLocs[vd] = SILGenFunction::VarLoc::get(value);

    // Emit a debug_value[_addr] instruction to record the start of this value's
    // lifetime.
    SILLocation PrologueLoc(vd);
    PrologueLoc.markAsPrologue();
    if (address)
      gen.B.createDebugValueAddr(PrologueLoc, value);
    else
      gen.B.createDebugValue(PrologueLoc, value);
  }
  
  void copyOrInitValueInto(SILGenFunction &gen, SILLocation loc,
                           ManagedValue value, bool isInit) override {
    // If this let value has an address, we can handle it just like a single
    // buffer value.
    if (hasAddress())
      return SingleBufferInitialization::
        copyOrInitValueIntoSingleBuffer(gen, loc, value, isInit, getAddress());
    
    // Otherwise, we bind the value.
    if (isInit) {
      // Disable the rvalue expression cleanup, since the let value
      // initialization has a cleanup that lives for the entire scope of the
      // let declaration.
      bindValue(value.forward(gen), gen);
    } else {
      // Disable the expression cleanup of the copy, since the let value
      // initialization has a cleanup that lives for the entire scope of the
      // let declaration.
      bindValue(value.copyUnmanaged(gen, loc).forward(gen), gen);
    }
  }

  void finishUninitialized(SILGenFunction &gen) override {
    LetValueInitialization::finishInitialization(gen);
  }

  void finishInitialization(SILGenFunction &gen) override {
    assert(!DidFinish &&
           "called LetValueInit::finishInitialization twice!");
    assert(gen.VarLocs.count(vd) && "Didn't bind a value to this let!");

    // Deactivate any cleanups we made when splitting the tuple.
    for (auto cleanup : SplitCleanups)
      gen.Cleanups.forwardCleanup(cleanup);

    // Activate the destroy cleanup.
    if (DestroyCleanup != CleanupHandle::invalid())
      gen.Cleanups.setCleanupState(DestroyCleanup, CleanupState::Active);

    DidFinish = true;
  }
};
} // end anonymous namespace


namespace {
/// Initialize a variable of reference-storage type.
class ReferenceStorageInitialization : public Initialization {
  InitializationPtr VarInit;
public:
  ReferenceStorageInitialization(InitializationPtr &&subInit)
    : VarInit(std::move(subInit)) {}

  SILValue getAddressOrNull() const override { return SILValue(); }


  void copyOrInitValueInto(SILGenFunction &gen, SILLocation loc,
                           ManagedValue value, bool isInit) override {
    // If this is not an initialization, copy the value before we translateIt,
    // translation expects a +1 value.
    if (isInit)
      value.forwardInto(gen, loc, VarInit->getAddress());
    else
      value.copyInto(gen, VarInit->getAddress(), loc);
  }

  void finishUninitialized(SILGenFunction &gen) override {
    ReferenceStorageInitialization::finishInitialization(gen);
  }
  
  void finishInitialization(SILGenFunction &gen) override {
    VarInit->finishInitialization(gen);
  }
};
} // end anonymous namespace

namespace {
/// Abstract base class for refutable pattern initializations.
class RefutablePatternInitialization : public Initialization {
  /// This is the label to jump to if the pattern fails to match.
  JumpDest failureDest;
public:
  RefutablePatternInitialization(JumpDest failureDest)
    : failureDest(failureDest) {
    assert(failureDest.isValid() &&
           "Refutable patterns can only exist in failable conditions");
  }

  JumpDest getFailureDest() const { return failureDest; }

  SILValue getAddressOrNull() const override { return SILValue(); }

  void copyOrInitValueInto(SILGenFunction &gen, SILLocation loc,
                           ManagedValue value, bool isInit) override = 0;

  void bindVariable(SILLocation loc, VarDecl *var, ManagedValue value,
                    CanType formalValueType, SILGenFunction &SGF) {
    // Initialize the variable value.
    InitializationPtr init = SGF.emitInitializationForVarDecl(var);
    RValue(SGF, loc, formalValueType, value).forwardInto(SGF, loc, init.get());
  }

};
} // end anonymous namespace

namespace {
class ExprPatternInitialization : public RefutablePatternInitialization {
  ExprPattern *P;
public:
  ExprPatternInitialization(ExprPattern *P, JumpDest patternFailDest)
    : RefutablePatternInitialization(patternFailDest), P(P) {}

  void copyOrInitValueInto(SILGenFunction &gen, SILLocation loc,
                           ManagedValue value, bool isInit) override;
};
} // end anonymous namespace

void ExprPatternInitialization::
copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                    ManagedValue value, bool isInit) {
  assert(isInit && "Only initialization is supported for refutable patterns");

  FullExpr scope(SGF.Cleanups, CleanupLocation(P));
  bindVariable(P, P->getMatchVar(), value,
               P->getType()->getCanonicalType(), SGF);

  // Emit the match test.
  SILValue testBool;
  {
    FullExpr scope(SGF.Cleanups, CleanupLocation(P->getMatchExpr()));
    testBool = SGF.emitRValueAsSingleValue(P->getMatchExpr()).
       getUnmanagedValue();
  }

  SILBasicBlock *contBB = SGF.B.splitBlockForFallthrough();
  auto falseBB = SGF.Cleanups.emitBlockForCleanups(getFailureDest(), loc);
  SGF.B.createCondBranch(loc, testBool, contBB, falseBB);

  SGF.B.setInsertionPoint(contBB);
}

namespace {
class EnumElementPatternInitialization : public RefutablePatternInitialization {
  EnumElementDecl *ElementDecl;
  InitializationPtr subInitialization;
public:
  EnumElementPatternInitialization(EnumElementDecl *ElementDecl,
                                   InitializationPtr &&subInitialization,
                                   JumpDest patternFailDest)
    : RefutablePatternInitialization(patternFailDest), ElementDecl(ElementDecl),
      subInitialization(std::move(subInitialization)) {}
    
  void copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                           ManagedValue value, bool isInit) override {
    assert(isInit && "Only initialization is supported for refutable patterns");
    emitEnumMatch(value, ElementDecl, subInitialization.get(), getFailureDest(),
                  loc, SGF);
  }

  static void emitEnumMatch(ManagedValue value, EnumElementDecl *ElementDecl,
                            Initialization *subInit, JumpDest FailureDest,
                            SILLocation loc, SILGenFunction &SGF);
  
  void finishInitialization(SILGenFunction &SGF) override {
    if (subInitialization.get())
      subInitialization->finishInitialization(SGF);
  }
};
} // end anonymous namespace

static bool shouldDisableCleanupOnFailurePath(ManagedValue value,
                                              EnumElementDecl *elementDecl,
                                              SILGenFunction &SGF) {
  // If the enum is trivial, then there is no cleanup to disable.
  if (value.isPlusZeroRValueOrTrivial()) return false;
  
  // Check all of the members of the enum.  If any have a non-trivial payload,
  // then we can't disable the cleanup.
  for (auto elt : elementDecl->getParentEnum()->getAllElements()) {
    // Ignore the element that will be handled.
    if (elt == elementDecl) continue;
    
    // Elements without payloads are trivial.
    if (!elt->getArgumentInterfaceType()) continue;

    auto eltTy = value.getType().getEnumElementType(elt, SGF.SGM.M);
    if (!eltTy.isTrivial(SGF.SGM.M))
      return false;
  }
  return true;
}

void EnumElementPatternInitialization::
emitEnumMatch(ManagedValue value, EnumElementDecl *ElementDecl,
              Initialization *subInit, JumpDest failureDest,
              SILLocation loc, SILGenFunction &SGF) {
  
  SILBasicBlock *contBB = SGF.B.splitBlockForFallthrough();
  auto destination = std::make_pair(ElementDecl, contBB);
  
  
  // Get a destination that runs all of the cleanups needed when existing on the
  // failure path.  If the enum we're testing is non-trivial, there will be a
  // cleanup in this stack that will release its value.
  //
  // However, if the tested case is the only non-trivial case in the enum, then
  // the destruction on the failure path will be a no-op, so we can disable the
  // cleanup on that path.  This is an important micro-optimization for
  // Optional, since the .None case doesn't need to be cleaned up.
  bool ShouldDisableCleanupOnFailure =
    shouldDisableCleanupOnFailurePath(value, ElementDecl, SGF);
  
  if (ShouldDisableCleanupOnFailure)
    SGF.Cleanups.setCleanupState(value.getCleanup(), CleanupState::Dormant);
  
  auto defaultBB = SGF.Cleanups.emitBlockForCleanups(failureDest, loc);

  // Restore it if we disabled it.
  if (ShouldDisableCleanupOnFailure)
    SGF.Cleanups.setCleanupState(value.getCleanup(), CleanupState::Active);
  
  if (value.getType().isAddress())
    SGF.B.createSwitchEnumAddr(loc, value.getValue(), defaultBB, destination);
  else
    SGF.B.createSwitchEnum(loc, value.getValue(), defaultBB, destination);
  
  SGF.B.setInsertionPoint(contBB);
  
  // If the enum case has no bound value, we're done.
  if (!ElementDecl->getArgumentInterfaceType()) {
    assert(subInit == nullptr &&
           "Cannot have a subinit when there is no value to match against");
    return;
  }
  
  // Otherwise, the bound value for the enum case is available.
  SILType eltTy = value.getType().getEnumElementType(ElementDecl, SGF.SGM.M);
  auto &eltTL = SGF.getTypeLowering(eltTy);
  
  // If the case value is provided to us as a BB argument as long as the enum
  // is not address-only.
  SILValue eltValue;
  if (!value.getType().isAddress())
    eltValue = contBB->createPHIArgument(eltTy, ValueOwnershipKind::Owned);

  if (subInit == nullptr) {
    // If there is no subinitialization, then we are done matching.  Don't
    // bother projecting out the address-only element value only to ignore it.
    return;
  }
  
  if (value.getType().isAddress()) {
    // If the enum is address-only, take from the enum we have and load it if
    // the element value is loadable.
    assert((eltTL.isTrivial() || value.hasCleanup())
           && "must be able to consume value");
    eltValue = SGF.B.createUncheckedTakeEnumDataAddr(loc, value.forward(SGF),
                                                     ElementDecl, eltTy);
    // Load a loadable data value.
    if (eltTL.isLoadable())
      eltValue =
          eltTL.emitLoad(SGF.B, loc, eltValue, LoadOwnershipQualifier::Take);
  } else {
    // Otherwise, we're consuming this as a +1 value.
    value.forward(SGF);
  }
  
  // Now we have a +1 value.
  auto eltMV = SGF.emitManagedRValueWithCleanup(eltValue, eltTL);

  // If the payload is indirect, project it out of the box.
  if (ElementDecl->isIndirect() || ElementDecl->getParentEnum()->isIndirect()) {
    SILValue boxedValue = SGF.B.createProjectBox(loc, eltMV.getValue(), 0);
    auto &boxedTL = SGF.getTypeLowering(boxedValue->getType());
    // SEMANTIC ARC TODO: Revisit this when the verifier is enabled.
    if (boxedTL.isLoadable())
      boxedValue = boxedTL.emitLoad(SGF.B, loc, boxedValue,
                                    LoadOwnershipQualifier::Take);

    // We must treat the boxed value as +0 since it may be shared. Copy it if
    // nontrivial.
    // TODO: Should be able to hand it off at +0 in some cases.
    eltMV = ManagedValue::forUnmanaged(boxedValue);
    eltMV = eltMV.copyUnmanaged(SGF, loc);
  }
  
  // Reabstract to the substituted type, if needed.
  CanType substEltTy =
    value.getSwiftType()->getTypeOfMember(SGF.SGM.M.getSwiftModule(),
                                      ElementDecl,
                                      ElementDecl->getArgumentInterfaceType())
      ->getCanonicalType();

  AbstractionPattern origEltTy =
    (ElementDecl == SGF.getASTContext().getOptionalSomeDecl()
       ? AbstractionPattern(substEltTy)
       : SGF.SGM.M.Types.getAbstractionPattern(ElementDecl));
  
  eltMV = SGF.emitOrigToSubstValue(loc, eltMV, origEltTy, substEltTy);

  // Pass the +1 value down into the sub initialization.
  subInit->copyOrInitValueInto(SGF, loc, eltMV, /*is an init*/true);
}

namespace {
class IsPatternInitialization : public RefutablePatternInitialization {
  IsPattern *pattern;
  InitializationPtr subInitialization;
public:
  IsPatternInitialization(IsPattern *pattern,
                          InitializationPtr &&subInitialization,
                          JumpDest patternFailDest)
  : RefutablePatternInitialization(patternFailDest), pattern(pattern),
    subInitialization(std::move(subInitialization)) {}
    
  void copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                           ManagedValue value, bool isInit) override;
  
  void finishInitialization(SILGenFunction &SGF) override {
    if (subInitialization.get())
      subInitialization->finishInitialization(SGF);
  }
};
} // end anonymous namespace

void IsPatternInitialization::
copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                    ManagedValue value, bool isInit) {
  assert(isInit && "Only initialization is supported for refutable patterns");
  
  // Try to perform the cast to the destination type, producing an optional that
  // indicates whether we succeeded.
  auto destType = OptionalType::get(pattern->getCastTypeLoc().getType());
  
  value = emitConditionalCheckedCast(SGF, loc, value, pattern->getType(),
                                     destType, pattern->getCastKind(),
                                     SGFContext())
            .getAsSingleValue(SGF, loc);
  
  // Now that we have our result as an optional, we can use an enum projection
  // to do all the work.
  EnumElementPatternInitialization::
  emitEnumMatch(value, SGF.getASTContext().getOptionalSomeDecl(),
                subInitialization.get(), getFailureDest(), loc, SGF);
}

namespace {
class BoolPatternInitialization : public RefutablePatternInitialization {
  BoolPattern *pattern;
public:
  BoolPatternInitialization(BoolPattern *pattern,
                            JumpDest patternFailDest)
    : RefutablePatternInitialization(patternFailDest), pattern(pattern) {}

  void copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                           ManagedValue value, bool isInit) override;
};
} // end anonymous namespace

void BoolPatternInitialization::
copyOrInitValueInto(SILGenFunction &SGF, SILLocation loc,
                    ManagedValue value, bool isInit) {
  assert(isInit && "Only initialization is supported for refutable patterns");

  // Extract the i1 from the Bool struct.
  StructDecl *BoolStruct = cast<StructDecl>(SGF.getASTContext().getBoolDecl());
  auto Members = BoolStruct->lookupDirect(SGF.getASTContext().Id_value_);
  assert(Members.size() == 1 &&
         "Bool should have only one property with name '_value'");
  auto Member = dyn_cast<VarDecl>(Members[0]);
  assert(Member &&"Bool should have a property with name '_value' of type Int1");
  auto *i1Val = SGF.B.createStructExtract(loc, value.forward(SGF), Member);

  // Branch on the boolean based on whether we're testing for true or false.
  SILBasicBlock *trueBB = SGF.B.splitBlockForFallthrough();
  auto contBB = trueBB;
  auto falseBB = SGF.Cleanups.emitBlockForCleanups(getFailureDest(), loc);

  if (!pattern->getValue())
    std::swap(trueBB, falseBB);
  SGF.B.createCondBranch(loc, i1Val, trueBB, falseBB);
  SGF.B.setInsertionPoint(contBB);
}


namespace {

/// InitializationForPattern - A visitor for traversing a pattern, generating
/// SIL code to allocate the declared variables, and generating an
/// Initialization representing the needed initializations.
///
/// It is important that any Initialization created for a pattern that might
/// not have an immediate initializer implement finishUninitialized.  Note
/// that this only applies to irrefutable patterns.
struct InitializationForPattern
  : public PatternVisitor<InitializationForPattern, InitializationPtr>
{
  SILGenFunction &SGF;

  /// This is the place that should be jumped to if the pattern fails to match.
  /// This is invalid for irrefutable pattern initializations.
  JumpDest patternFailDest;

  InitializationForPattern(SILGenFunction &SGF, JumpDest patternFailDest)
    : SGF(SGF), patternFailDest(patternFailDest) {}

  // Paren, Typed, and Var patterns are noops, just look through them.
  InitializationPtr visitParenPattern(ParenPattern *P) {
    return visit(P->getSubPattern());
  }
  InitializationPtr visitTypedPattern(TypedPattern *P) {
    return visit(P->getSubPattern());
  }
  InitializationPtr visitVarPattern(VarPattern *P) {
    return visit(P->getSubPattern());
  }

  // AnyPatterns (i.e, _) don't require any storage. Any value bound here will
  // just be dropped.
  InitializationPtr visitAnyPattern(AnyPattern *P) {
    return InitializationPtr(new BlackHoleInitialization());
  }

  // Bind to a named pattern by creating a memory location and initializing it
  // with the initial value.
  InitializationPtr visitNamedPattern(NamedPattern *P) {
    if (!P->getDecl()->hasName()) {
      // Unnamed parameters don't require any storage. Any value bound here will
      // just be dropped.
      return InitializationPtr(new BlackHoleInitialization());
    }

    return SGF.emitInitializationForVarDecl(P->getDecl());
  }

  // Bind a tuple pattern by aggregating the component variables into a
  // TupleInitialization.
  InitializationPtr visitTuplePattern(TuplePattern *P) {
    TupleInitialization *init = new TupleInitialization();
    for (auto &elt : P->getElements())
      init->SubInitializations.push_back(visit(elt.getPattern()));
    return InitializationPtr(init);
  }

  InitializationPtr visitEnumElementPattern(EnumElementPattern *P) {
    InitializationPtr subInit;
    if (auto *subP = P->getSubPattern())
      subInit = visit(subP);
    auto *res = new EnumElementPatternInitialization(P->getElementDecl(),
                                                     std::move(subInit),
                                                     patternFailDest);
    return InitializationPtr(res);
  }
  InitializationPtr visitOptionalSomePattern(OptionalSomePattern *P) {
    InitializationPtr subInit = visit(P->getSubPattern());
    auto *res = new EnumElementPatternInitialization(P->getElementDecl(),
                                                     std::move(subInit),
                                                     patternFailDest);
    return InitializationPtr(res);
  }
  InitializationPtr visitIsPattern(IsPattern *P) {
    InitializationPtr subInit;
    if (auto *subP = P->getSubPattern())
      subInit = visit(subP);
    return InitializationPtr(new IsPatternInitialization(P, std::move(subInit),
                                                         patternFailDest));
  }
  InitializationPtr visitBoolPattern(BoolPattern *P) {
    return InitializationPtr(new BoolPatternInitialization(P, patternFailDest));
  }
  InitializationPtr visitExprPattern(ExprPattern *P) {
    return InitializationPtr(new ExprPatternInitialization(P, patternFailDest));
  }
};

} // end anonymous namespace

InitializationPtr SILGenFunction::emitInitializationForVarDecl(VarDecl *vd) {
  // If this is a computed variable, we don't need to do anything here.
  // We'll generate the getter and setter when we see their FuncDecls.
  if (!vd->hasStorage())
    return InitializationPtr(new BlackHoleInitialization());

  if (vd->isDebuggerVar()) {
    DebuggerClient *DebugClient = SGM.SwiftModule->getDebugClient();
    assert(DebugClient && "Debugger variables with no debugger client");
    SILDebuggerClient *SILDebugClient = DebugClient->getAsSILDebuggerClient();
    assert(SILDebugClient && "Debugger client doesn't support SIL");
    SILValue SV = SILDebugClient->emitLValueForVariable(vd, B);

    VarLocs[vd] = SILGenFunction::VarLoc::get(SV);
    return InitializationPtr(new KnownAddressInitialization(SV));
  }

  CanType varType = vd->getType()->getCanonicalType();

  assert(!isa<InOutType>(varType) && "local variables should never be inout");

  // If this is a 'let' initialization for a non-global, set up a
  // let binding, which stores the initialization value into VarLocs directly.
  if (vd->isLet() && vd->getDeclContext()->isLocalContext() &&
      !isa<ReferenceStorageType>(varType))
    return InitializationPtr(new LetValueInitialization(vd, *this));

  // If the variable has no initial value, emit a mark_uninitialized instruction
  // so that DI tracks and enforces validity of it.
  bool isUninitialized =
    vd->getParentPatternBinding() && !vd->getParentInitializer();
  
  // If this is a global variable, initialize it without allocations or
  // cleanups.
  InitializationPtr Result;
  if (!vd->getDeclContext()->isLocalContext()) {
    auto *silG = SGM.getSILGlobalVariable(vd, NotForDefinition);
    B.createAllocGlobal(vd, silG);
    SILValue addr = B.createGlobalAddr(vd, silG);
    if (isUninitialized)
      addr = B.createMarkUninitializedVar(vd, addr);

    VarLocs[vd] = SILGenFunction::VarLoc::get(addr);
    Result = InitializationPtr(new KnownAddressInitialization(addr));
  } else {
    Result = emitLocalVariableWithCleanup(vd, isUninitialized);
  }

  // If we're initializing a weak or unowned variable, this requires a change in
  // type.
  if (isa<ReferenceStorageType>(varType))
    Result = InitializationPtr(new
                           ReferenceStorageInitialization(std::move(Result)));
  return Result;
}

void SILGenFunction::emitPatternBinding(PatternBindingDecl *PBD,
                                        unsigned pbdEntry) {
  auto &entry = PBD->getPatternList()[pbdEntry];
  auto initialization = emitPatternBindingInitialization(entry.getPattern(),
                                                         JumpDest::invalid());

  // If an initial value expression was specified by the decl, emit it into
  // the initialization. Otherwise, mark it uninitialized for DI to resolve.
  if (auto *Init = entry.getInit()) {
    FullExpr Scope(Cleanups, CleanupLocation(Init));
    emitExprInto(Init, initialization.get());
  } else {
    initialization->finishUninitialized(*this);
  }
}

void SILGenFunction::visitPatternBindingDecl(PatternBindingDecl *PBD) {

  // Allocate the variables and build up an Initialization over their
  // allocated storage.
  for (unsigned i : indices(PBD->getPatternList())) {
    emitPatternBinding(PBD, i);
  }
}

void SILGenFunction::visitVarDecl(VarDecl *D) {
  // We handle emitting the variable storage when we see the pattern binding.
  // Here we just emit the behavior witness table, if any.
  
  if (D->hasBehavior())
    SGM.emitPropertyBehavior(D);
}

/// Emit a check that returns 1 if the running OS version is in
/// the specified version range and 0 otherwise. The returned SILValue
/// (which has type Builtin.Int1) represents the result of this check.
SILValue SILGenFunction::emitOSVersionRangeCheck(SILLocation loc,
                                                 const VersionRange &range) {
  // Emit constants for the checked version range.
  clang::VersionTuple Vers = range.getLowerEndpoint();
  unsigned major = Vers.getMajor();
  unsigned minor =
      (Vers.getMinor().hasValue() ? Vers.getMinor().getValue() : 0);
  unsigned subminor =
      (Vers.getSubminor().hasValue() ? Vers.getSubminor().getValue() : 0);

  SILType wordType = SILType::getBuiltinWordType(getASTContext());

  SILValue majorValue = B.createIntegerLiteral(loc, wordType, major);
  SILValue minorValue = B.createIntegerLiteral(loc, wordType, minor);
  SILValue subminorValue = B.createIntegerLiteral(loc, wordType, subminor);

  // Emit call to _stdlib_isOSVersionAtLeast(major, minor, patch)
  FuncDecl *versionQueryDecl =
      getASTContext().getIsOSVersionAtLeastDecl(nullptr);
  assert(versionQueryDecl);

  auto silDeclRef = SILDeclRef(versionQueryDecl);
  SILValue availabilityGTEFn = emitGlobalFunctionRef(
      loc, silDeclRef, getConstantInfo(silDeclRef));

  SILValue args[] = {majorValue, minorValue, subminorValue};
  return B.createApply(loc, availabilityGTEFn, args, false);
}


/// Emit the boolean test and/or pattern bindings indicated by the specified
/// stmt condition.  If the condition fails, control flow is transferred to the
/// specified JumpDest.  The insertion point is left in the block where the
/// condition has matched and any bound variables are in scope.
///
void SILGenFunction::emitStmtCondition(StmtCondition Cond,
                                       JumpDest FailDest, SILLocation loc) {

  assert(B.hasValidInsertionPoint() &&
         "emitting condition at unreachable point");
  
  for (const auto &elt : Cond) {
    SILLocation booleanTestLoc = loc;
    SILValue booleanTestValue;

    switch (elt.getKind()) {
    case StmtConditionElement::CK_PatternBinding: {
      InitializationPtr initialization =
      InitializationForPattern(*this, FailDest).visit(elt.getPattern());

      // Emit the initial value into the initialization.
      FullExpr Scope(Cleanups, CleanupLocation(elt.getInitializer()));
      emitExprInto(elt.getInitializer(), initialization.get());
      // Pattern bindings handle their own tests, we don't need a boolean test.
      continue;
    }

    case StmtConditionElement::CK_Boolean: { // Handle boolean conditions.
      auto *expr = elt.getBoolean();
      // Evaluate the condition as an i1 value (guaranteed by Sema).
      FullExpr Scope(Cleanups, CleanupLocation(expr));
      booleanTestValue = emitRValue(expr).forwardAsSingleValue(*this, expr);
      booleanTestLoc = expr;
      break;
    }
    case StmtConditionElement::CK_Availability:
      // Check the running OS version to determine whether it is in the range
      // specified by elt.
      VersionRange OSVersion = elt.getAvailability()->getAvailableRange();
      assert(!OSVersion.isEmpty());

      if (OSVersion.isAll()) {
        // If there's no check for the current platform, this condition is
        // trivially true.
        SILType i1 = SILType::getBuiltinIntegerType(1, getASTContext());
        booleanTestValue = B.createIntegerLiteral(loc, i1, true);
      } else {
        booleanTestValue = emitOSVersionRangeCheck(loc, OSVersion);
      }
      break;
    }

    // Now that we have a boolean test as a Builtin.i1, emit the branch.
    assert(booleanTestValue->getType().
           castTo<BuiltinIntegerType>()->isFixedWidth(1) &&
           "Sema forces conditions to have Builtin.i1 type");
    
    // Just branch on the condition.  On failure, we unwind any active cleanups,
    // on success we fall through to a new block.
    SILBasicBlock *ContBB = createBasicBlock();
    auto FailBB = Cleanups.emitBlockForCleanups(FailDest, loc);
    B.createCondBranch(booleanTestLoc, booleanTestValue, ContBB, FailBB);
    
    // Finally, emit the continue block and keep emitting the rest of the
    // condition.
    B.emitBlock(ContBB);
  }
}

InitializationPtr
SILGenFunction::emitPatternBindingInitialization(Pattern *P,
                                                 JumpDest failureDest) {
  return InitializationForPattern(*this, failureDest).visit(P);
}

/// Enter a cleanup to deallocate the given location.
CleanupHandle SILGenFunction::enterDeallocStackCleanup(SILValue temp) {
  assert(temp->getType().isAddress() &&  "dealloc must have an address type");
  Cleanups.pushCleanup<DeallocStackCleanup>(temp);
  return Cleanups.getTopCleanup();
}

CleanupHandle SILGenFunction::enterDestroyCleanup(SILValue valueOrAddr) {
  Cleanups.pushCleanup<ReleaseValueCleanup>(valueOrAddr);
  return Cleanups.getTopCleanup();
}

CleanupHandle SILGenFunction::enterEndBorrowCleanup(SILValue original,
                                                    SILValue borrowed) {
  Cleanups.pushCleanup<EndBorrowCleanup>(original, borrowed);
  return Cleanups.getTopCleanup();
}

namespace {
  /// A cleanup that deinitializes an opaque existential container
  /// before a value has been stored into it, or after its value was taken.
  class DeinitExistentialCleanup: public Cleanup {
    SILValue existentialAddr;
    CanType concreteFormalType;
    ExistentialRepresentation repr;
  public:
    DeinitExistentialCleanup(SILValue existentialAddr,
                             CanType concreteFormalType,
                             ExistentialRepresentation repr)
      : existentialAddr(existentialAddr),
        concreteFormalType(concreteFormalType),
        repr(repr) {}
    
    void emit(SILGenFunction &gen, CleanupLocation l) override {
      switch (repr) {
      case ExistentialRepresentation::None:
      case ExistentialRepresentation::Class:
      case ExistentialRepresentation::Metatype:
        llvm_unreachable("cannot cleanup existential");
      case ExistentialRepresentation::Opaque:
        gen.B.createDeinitExistentialAddr(l, existentialAddr);
        break;
      case ExistentialRepresentation::Boxed:
        gen.B.createDeallocExistentialBox(l, concreteFormalType,
                                          existentialAddr);
        break;
      }
    }

    void dump() const override {
#ifndef NDEBUG
      llvm::errs() << "DeinitExistentialCleanup\n"
                   << "State:" << getState() << "\n"
                   << "Value:" << existentialAddr << "\n";
#endif
    }
  };
} // end anonymous namespace

/// Enter a cleanup to emit a DeinitExistentialAddr or DeinitExistentialBox
/// of the specified value.
CleanupHandle SILGenFunction::enterDeinitExistentialCleanup(
                                               SILValue valueOrAddr,
                                               CanType concreteFormalType,
                                               ExistentialRepresentation repr) {
  Cleanups.pushCleanup<DeinitExistentialCleanup>(valueOrAddr,
                                                 concreteFormalType,
                                                 repr);
  return Cleanups.getTopCleanup();
}

void SILGenModule::emitExternalWitnessTable(ProtocolConformance *c) {
  auto root = c->getRootNormalConformance();
  // Emit the witness table right now if we used it.
  if (usedConformances.count(root)) {
    getWitnessTable(c);
    return;
  }
  // Otherwise, remember it for later.
  delayedConformances.insert({root, {lastEmittedConformance}});
  lastEmittedConformance = root;
}

void SILGenModule::emitExternalDefinition(Decl *d) {
  switch (d->getKind()) {
  case DeclKind::Func: {
    emitFunction(cast<FuncDecl>(d));
    break;
  }
  case DeclKind::Constructor: {
    auto C = cast<ConstructorDecl>(d);
    // For factories, we don't need to emit a special thunk; the normal
    // foreign-to-native thunk is sufficient.
    if (C->isFactoryInit())
      break;

    emitConstructor(C);
    break;
  }
  case DeclKind::Enum:
  case DeclKind::Struct:
  case DeclKind::Class: {
    // Emit witness tables.
    auto nom = cast<NominalTypeDecl>(d);
    for (auto c : nom->getLocalConformances(ConformanceLookupKind::All,
                                            nullptr, /*sorted=*/true)) {
      auto *proto = c->getProtocol();
      if (Lowering::TypeConverter::protocolRequiresWitnessTable(proto) &&
          isa<NormalProtocolConformance>(c) &&
          c->isComplete())
        emitExternalWitnessTable(c);
    }
    break;
  }

  case DeclKind::Protocol:
    // Nothing to do in SILGen for other external types.
    break;

  case DeclKind::Var:
    // Imported static vars are handled solely in IRGen.
    break;

  case DeclKind::IfConfig:
  case DeclKind::Extension:
  case DeclKind::PatternBinding:
  case DeclKind::EnumCase:
  case DeclKind::EnumElement:
  case DeclKind::TopLevelCode:
  case DeclKind::TypeAlias:
  case DeclKind::AssociatedType:
  case DeclKind::GenericTypeParam:
  case DeclKind::Param:
  case DeclKind::Import:
  case DeclKind::Subscript:
  case DeclKind::Destructor:
  case DeclKind::VisitorOfRefsInInstance_dmu_:
  case DeclKind::InfixOperator:
  case DeclKind::PrefixOperator:
  case DeclKind::PostfixOperator:
  case DeclKind::PrecedenceGroup:
  case DeclKind::Module:
    llvm_unreachable("Not a valid external definition for SILGen");
  }
}

/// Create a LocalVariableInitialization for the uninitialized var.
InitializationPtr
SILGenFunction::emitLocalVariableWithCleanup(VarDecl *vd, bool NeedsMarkUninit,
                                             unsigned ArgNo) {
  return InitializationPtr(
      new LocalVariableInitialization(vd, NeedsMarkUninit, ArgNo, *this));
}

/// Create an Initialization for an uninitialized temporary.
std::unique_ptr<TemporaryInitialization>
SILGenFunction::emitTemporary(SILLocation loc, const TypeLowering &tempTL) {
  SILValue addr = emitTemporaryAllocation(loc, tempTL.getLoweredType());
  return useBufferAsTemporary(addr, tempTL);
}

/// Create an Initialization for an uninitialized buffer.
std::unique_ptr<TemporaryInitialization>
SILGenFunction::useBufferAsTemporary(SILValue addr,
                                     const TypeLowering &tempTL) {
  CleanupHandle cleanup = enterDormantTemporaryCleanup(addr, tempTL);
  return std::unique_ptr<TemporaryInitialization>(
                                    new TemporaryInitialization(addr, cleanup));
}

CleanupHandle
SILGenFunction::enterDormantTemporaryCleanup(SILValue addr,
                                             const TypeLowering &tempTL) {
  if (tempTL.isTrivial())
    return CleanupHandle::invalid();

  Cleanups.pushCleanupInState<ReleaseValueCleanup>(CleanupState::Dormant, addr);
  return Cleanups.getCleanupsDepth();
}

void SILGenFunction::destroyLocalVariable(SILLocation silLoc, VarDecl *vd) {
  assert(vd->getDeclContext()->isLocalContext() &&
         "can't emit a local var for a non-local var decl");
  assert(vd->hasStorage() && "can't emit storage for a computed variable");

  assert(VarLocs.count(vd) && "var decl wasn't emitted?!");

  auto loc = VarLocs[vd];

  // For a heap variable, the box is responsible for the value. We just need
  // to give up our retain count on it.
  if (loc.box) {
    B.emitDestroyValueOperation(silLoc, loc.box);
    return;
  }

  // For 'let' bindings, we emit a release_value or destroy_addr, depending on
  // whether we have an address or not.
  SILValue Val = loc.value;
  if (!Val->getType().isAddress())
    B.emitDestroyValueOperation(silLoc, Val);
  else
    B.createDestroyAddr(silLoc, Val);
}

void SILGenFunction::deallocateUninitializedLocalVariable(SILLocation silLoc,
                                                          VarDecl *vd) {
  assert(vd->getDeclContext()->isLocalContext() &&
         "can't emit a local var for a non-local var decl");
  assert(vd->hasStorage() && "can't emit storage for a computed variable");

  assert(VarLocs.count(vd) && "var decl wasn't emitted?!");

  auto loc = VarLocs[vd];

  // Ignore let values captured without a memory location.
  if (!loc.value->getType().isAddress()) return;

  assert(loc.box && "captured var should have been given a box");
  B.createDeallocBox(silLoc, loc.box);
}

namespace {

// Is this a free function witness satisfying a static method requirement?
static IsFreeFunctionWitness_t isFreeFunctionWitness(ValueDecl *requirement,
                                                     ValueDecl *witness) {
  if (!witness->getDeclContext()->isTypeContext()) {
    assert(!requirement->isInstanceMember()
           && "free function satisfying instance method requirement?!");
    return IsFreeFunctionWitness;
  }

  return IsNotFreeFunctionWitness;
}

/// A CRTP class for emitting witness thunks for the requirements of a
/// protocol.
///
/// There are two subclasses:
///
/// - SILGenConformance: emits witness thunks for a conformance of a
///   a concrete type to a protocol
/// - SILGenDefaultWitnessTable: emits default witness thunks for
///   default implementations of protocol requirements
///
template<typename T> class SILGenWitnessTable : public SILWitnessVisitor<T> {
  T &asDerived() { return *static_cast<T*>(this); }

public:
  void addMethod(FuncDecl *fd, Witness witness) {
    return addMethod(fd, witness.getDecl(), witness);
  }

  void addConstructor(ConstructorDecl *cd, Witness witness) {
    SILDeclRef requirementRef(cd, SILDeclRef::Kind::Allocator,
                              ResilienceExpansion::Minimal);

    SILDeclRef witnessRef(witness.getDecl(), SILDeclRef::Kind::Allocator,
                          SILDeclRef::ConstructAtBestResilienceExpansion,
                          requirementRef.uncurryLevel);

    asDerived().addMethod(requirementRef, witnessRef, IsNotFreeFunctionWitness,
                          witness);
  }

  /// Subclasses must override SILWitnessVisitor::visitAbstractStorageDecl()
  /// to call addAbstractStorageDecl(), since we need the substitutions to
  /// be passed down into addMethod().
  ///
  /// FIXME: Seems that conformance->getWitness() should do this for us?
  void addAbstractStorageDecl(AbstractStorageDecl *d, Witness witness) {
    auto *witnessSD = cast<AbstractStorageDecl>(witness.getDecl());
    addMethod(d->getGetter(), witnessSD->getGetter(), witness);
    if (d->isSettable(d->getDeclContext()))
      addMethod(d->getSetter(), witnessSD->getSetter(), witness);
    if (auto materializeForSet = d->getMaterializeForSetFunc())
      addMethod(materializeForSet, witnessSD->getMaterializeForSetFunc(),
                witness);
  }

private:
  void addMethod(FuncDecl *fd, ValueDecl *witnessDecl, Witness witness) {

    // TODO: multiple resilience expansions?
    // TODO: multiple uncurry levels?
    SILDeclRef requirementRef(fd, SILDeclRef::Kind::Func,
                              ResilienceExpansion::Minimal);
    // Free function witnesses have an implicit uncurry layer imposed on them by
    // the inserted metatype argument.
    auto isFree = isFreeFunctionWitness(fd, witnessDecl);
    unsigned witnessUncurryLevel = isFree ? requirementRef.uncurryLevel - 1
                                          : requirementRef.uncurryLevel;

    SILDeclRef witnessRef(witnessDecl, SILDeclRef::Kind::Func,
                          SILDeclRef::ConstructAtBestResilienceExpansion,
                          witnessUncurryLevel);

    asDerived().addMethod(requirementRef, witnessRef, isFree, witness);
  }

};

/// Emit a witness table for a protocol conformance.
class SILGenConformance : public SILGenWitnessTable<SILGenConformance> {
  using super = SILGenWitnessTable<SILGenConformance>;

public:
  SILGenModule &SGM;
  NormalProtocolConformance *Conformance;
  std::vector<SILWitnessTable::Entry> Entries;
  SILLinkage Linkage;

  SILGenConformance(SILGenModule &SGM, NormalProtocolConformance *C)
    // We only need to emit witness tables for base NormalProtocolConformances.
    : SGM(SGM), Conformance(C->getRootNormalConformance()),
      Linkage(getLinkageForProtocolConformance(Conformance,
                                               ForDefinition))
  {
    // Not all protocols use witness tables.
    if (!Lowering::TypeConverter::protocolRequiresWitnessTable(
        Conformance->getProtocol()))
      Conformance = nullptr;
  }

  SILWitnessTable *emit() {
    // Nothing to do if this wasn't a normal conformance.
    if (!Conformance)
      return nullptr;

    auto *proto = Conformance->getProtocol();
    visitProtocolDecl(proto);

    // Serialize the witness table in two cases:
    // 1) We're serializing everything
    // 2) The type has a fixed layout in all resilience domains, and the
    //    conformance is externally visible
    IsFragile_t isFragile = IsNotFragile;
    if (SGM.makeModuleFragile)
      isFragile = IsFragile;
    if (auto nominal = Conformance->getInterfaceType()->getAnyNominal())
      if (nominal->hasFixedLayout() &&
          proto->getEffectiveAccess() >= Accessibility::Public &&
          nominal->getEffectiveAccess() >= Accessibility::Public)
        isFragile = IsFragile;

    // Check if we already have a declaration or definition for this witness
    // table.
    if (auto *wt = SGM.M.lookUpWitnessTable(Conformance, false)) {
      // If we have a definition already, just return it.
      //
      // FIXME: I am not sure if this is possible, if it is not change this to an
      // assert.
      if (wt->isDefinition())
        return wt;

      // If we have a declaration, convert the witness table to a definition.
      if (wt->isDeclaration()) {
        wt->convertToDefinition(Entries, isFragile);

        // Since we had a declaration before, its linkage should be external,
        // ensure that we have a compatible linkage for sanity. *NOTE* we are ok
        // with both being shared since we do not have a shared_external
        // linkage.
        assert(stripExternalFromLinkage(wt->getLinkage()) == Linkage &&
               "Witness table declaration has inconsistent linkage with"
               " silgen definition.");

        // And then override the linkage with the new linkage.
        wt->setLinkage(Linkage);
        return wt;
      }
    }

    // Otherwise if we have no witness table yet, create it.
    return SILWitnessTable::create(SGM.M, Linkage, isFragile,
                                   Conformance, Entries);
  }

  void addOutOfLineBaseProtocol(ProtocolDecl *baseProtocol) {
    assert(Lowering::TypeConverter::protocolRequiresWitnessTable(baseProtocol));

    auto foundBaseConformance
      = Conformance->getInheritedConformances().find(baseProtocol);
    assert(foundBaseConformance != Conformance->getInheritedConformances().end()
           && "no inherited conformance for base protocol");

    auto conformance = foundBaseConformance->second;

    Entries.push_back(SILWitnessTable::BaseProtocolWitness{
      baseProtocol,
      conformance,
    });

    // Emit the witness table for the base conformance if it is shared.
    if (getLinkageForProtocolConformance(
                                        conformance->getRootNormalConformance(),
                                        NotForDefinition)
          == SILLinkage::Shared)
      SGM.getWitnessTable(conformance->getRootNormalConformance());
  }

  void addMethod(FuncDecl *fd) {
    Witness witness = Conformance->getWitness(fd, nullptr);
    super::addMethod(fd, witness);
  }

  void addConstructor(ConstructorDecl *cd) {
    Witness witness = Conformance->getWitness(cd, nullptr);
    super::addConstructor(cd, witness);
  }

  void addMethod(SILDeclRef requirementRef,
                 SILDeclRef witnessRef,
                 IsFreeFunctionWitness_t isFree,
                 Witness witness) {
    // Emit the witness thunk and add it to the table.

    // If this is a non-present optional requirement, emit a MissingOptional.
    if (!witnessRef) {
      auto *fd = requirementRef.getDecl();
      assert(fd->getAttrs().hasAttribute<OptionalAttr>() &&
             "Non-optional protocol requirement lacks a witness?");
      Entries.push_back(SILWitnessTable::MissingOptionalWitness{ fd });
      return;
    }

    SILFunction *witnessFn =
      SGM.emitProtocolWitness(Conformance, Linkage, requirementRef, witnessRef,
                              isFree, witness);
    Entries.push_back(
                    SILWitnessTable::MethodWitness{requirementRef, witnessFn});
  }

  void addAssociatedType(AssociatedTypeDecl *td,
                         ArrayRef<ProtocolDecl *> protos) {
    // Find the substitution info for the witness type.
    const auto &witness = Conformance->getTypeWitness(td, /*resolver=*/nullptr);

    // Emit the record for the type itself.
    Entries.push_back(SILWitnessTable::AssociatedTypeWitness{td,
                                witness.getReplacement()->getCanonicalType()});

    // Emit records for the protocol requirements on the type.
    assert(protos.size() == witness.getConformances().size()
           && "number of conformances in assoc type substitution do not match "
              "number of requirements on assoc type");
    // The conformances should be all abstract or all concrete.
    assert(witness.getConformances().empty()
           || (witness.getConformances()[0].isConcrete()
                 ? std::all_of(witness.getConformances().begin(),
                               witness.getConformances().end(),
                               [&](const ProtocolConformanceRef C) -> bool {
                                 return C.isConcrete();
                               })
                 : std::all_of(witness.getConformances().begin(),
                               witness.getConformances().end(),
                               [&](const ProtocolConformanceRef C) -> bool {
                                 return C.isAbstract();
                               })));

    for (auto *protocol : protos) {
      // Only reference the witness if the protocol requires it.
      if (!Lowering::TypeConverter::protocolRequiresWitnessTable(protocol))
        continue;

      ProtocolConformanceRef conformance(protocol);
      // If the associated type requirement is satisfied by an associated type,
      // these will all be abstract conformances.
      if (witness.getConformances()[0].isConcrete()) {
        auto foundConformance = std::find_if(witness.getConformances().begin(),
                                        witness.getConformances().end(),
                                        [&](ProtocolConformanceRef c) {
                                          return c.getRequirement() == protocol;
                                        });
        assert(foundConformance != witness.getConformances().end());
        conformance = *foundConformance;
      }
      SGM.useConformance(conformance);

      Entries.push_back(SILWitnessTable::AssociatedTypeProtocolWitness{
        td, protocol, conformance
      });
    }
  }

  void visitAbstractStorageDecl(AbstractStorageDecl *d) {
    Witness witness = Conformance->getWitness(d, nullptr);
    addAbstractStorageDecl(d, witness);
  }
};

} // end anonymous namespace

static SILWitnessTable *
getWitnessTableToInsertAfter(SILGenModule &SGM,
                             NormalProtocolConformance *insertAfter) {
  while (insertAfter) {
    // If the table was emitted, emit after it.
    auto found = SGM.emittedWitnessTables.find(insertAfter);
    if (found != SGM.emittedWitnessTables.end())
      return found->second;

    // Otherwise, try inserting after the table we would transitively be
    // inserted after.
    auto foundDelayed = SGM.delayedConformances.find(insertAfter);
    if (foundDelayed != SGM.delayedConformances.end())
      insertAfter = foundDelayed->second.insertAfter;
    else
      break;
  }

  return nullptr;
}

SILWitnessTable *
SILGenModule::getWitnessTable(ProtocolConformance *conformance) {
  auto normal = conformance->getRootNormalConformance();

  // If we've already emitted this witness table, return it.
  auto found = emittedWitnessTables.find(normal);
  if (found != emittedWitnessTables.end())
    return found->second;

  SILWitnessTable *table = SILGenConformance(*this, normal).emit();
  emittedWitnessTables.insert({normal, table});

  // If we delayed emission of this witness table, move it to its rightful
  // place within the module.
  auto foundDelayed = delayedConformances.find(normal);
  if (foundDelayed != delayedConformances.end()) {
    M.witnessTables.remove(table);
    auto insertAfter = getWitnessTableToInsertAfter(*this,
                                              foundDelayed->second.insertAfter);
    if (!insertAfter) {
      M.witnessTables.push_front(table);
    } else {
      M.witnessTables.insertAfter(insertAfter->getIterator(), table);
    }
  } else {
    // We would have marked a delayed conformance as "last emitted" when it
    // was delayed.
    lastEmittedConformance = normal;
  }
  return table;
}

static bool maybeOpenCodeProtocolWitness(SILGenFunction &gen,
                                         ProtocolConformance *conformance,
                                         SILLinkage linkage,
                                         Type selfInterfaceType,
                                         Type selfType,
                                         GenericEnvironment *genericEnv,
                                         SILDeclRef requirement,
                                         SILDeclRef witness,
                                         SubstitutionList witnessSubs) {
  if (auto witnessFn = dyn_cast<FuncDecl>(witness.getDecl())) {
    if (witnessFn->getAccessorKind() == AccessorKind::IsMaterializeForSet) {
      auto reqFn = cast<FuncDecl>(requirement.getDecl());
      assert(reqFn->getAccessorKind() == AccessorKind::IsMaterializeForSet);
      return gen.maybeEmitMaterializeForSetThunk(conformance, linkage,
                                                 selfInterfaceType, selfType,
                                                 genericEnv, reqFn, witnessFn,
                                                 witnessSubs);
    }
  }

  return false;
}

SILFunction *
SILGenModule::emitProtocolWitness(ProtocolConformance *conformance,
                                  SILLinkage linkage,
                                  SILDeclRef requirement,
                                  SILDeclRef witnessRef,
                                  IsFreeFunctionWitness_t isFree,
                                  Witness witness) {
  auto requirementInfo = Types.getConstantInfo(requirement);
  unsigned witnessUncurryLevel = witnessRef.uncurryLevel;

  // If the witness is a free function, consider the self argument
  // uncurry level.
  if (isFree)
    ++witnessUncurryLevel;

  // The SIL witness thunk has the type of the AST-level witness with
  // witness substitutions applied, at the abstraction level of the
  // original protocol requirement.
  assert(requirement.uncurryLevel == witnessUncurryLevel &&
         "uncurry level of requirement and witness do not match");

  GenericEnvironment *genericEnv = nullptr;

  // Work out the lowered function type of the SIL witness thunk.
  auto reqtOrigTy
    = cast<GenericFunctionType>(requirementInfo.LoweredInterfaceType);
  CanAnyFunctionType reqtSubstTy;
  SubstitutionList witnessSubs;
  if (witness.requiresSubstitution()) {
    genericEnv = witness.getSyntheticEnvironment();
    witnessSubs = witness.getSubstitutions();

    const SubstitutionMap &reqtSubs
      = witness.getRequirementToSyntheticMap();
    auto input = reqtOrigTy->getInput().subst(reqtSubs)->getCanonicalType();
    auto result = reqtOrigTy->getResult().subst(reqtSubs)->getCanonicalType();

    if (genericEnv) {
      auto *genericSig = genericEnv->getGenericSignature();
      reqtSubstTy = cast<GenericFunctionType>(
        GenericFunctionType::get(genericSig, input, result,
                                 reqtOrigTy->getExtInfo())
          ->getCanonicalType());
    } else {
      reqtSubstTy = CanFunctionType::get(input, result,
                                         reqtOrigTy->getExtInfo());
    }
  } else {
    genericEnv = witnessRef.getDecl()->getInnermostDeclContext()
                   ->getGenericEnvironmentOfContext();

    Type concreteTy = conformance->getInterfaceType();

    // FIXME: conformance substitutions should be in terms of interface types
    auto concreteSubs = concreteTy->gatherAllSubstitutions(M.getSwiftModule(),
                                                           nullptr, nullptr);
    auto specialized = conformance;
    if (conformance->getGenericSignature()) {
      ASTContext &ctx = getASTContext();
      specialized = ctx.getSpecializedConformance(concreteTy, conformance,
                                                  concreteSubs);
    }

    auto reqtSubs = SubstitutionMap::getProtocolSubstitutions(
        conformance->getProtocol(),
        concreteTy,
        ProtocolConformanceRef(specialized));

    auto input = reqtOrigTy->getInput().subst(reqtSubs)->getCanonicalType();
    auto result = reqtOrigTy->getResult().subst(reqtSubs)->getCanonicalType();

    reqtSubstTy = CanFunctionType::get(input, result, reqtOrigTy->getExtInfo());
  }

  // Lower the witness thunk type with the requirement's abstraction level.
  auto witnessSILFnType = getNativeSILFunctionType(M,
                                                   AbstractionPattern(reqtOrigTy),
                                                   reqtSubstTy,
                                                   witnessRef);

  // Mangle the name of the witness thunk.
  std::string nameBuffer;
  {
    Mangler mangler;

    // Concrete witness thunks get a special mangling.
    if (conformance) {
      mangler.append("_TTW");
      mangler.mangleProtocolConformance(conformance);

    // Default witness thunks are mangled as if they were the protocol
    // requirement.
    } else {
      mangler.append("_T");
    }

    if (auto ctor = dyn_cast<ConstructorDecl>(requirement.getDecl())) {
      mangler.mangleConstructorEntity(ctor, /*isAllocating=*/true,
                                      requirement.uncurryLevel);
    } else {
      assert(isa<FuncDecl>(requirement.getDecl())
             && "need to handle mangling of non-Func SILDeclRefs here");
      auto requiredDecl = cast<FuncDecl>(requirement.getDecl());
      mangler.mangleEntity(requiredDecl, requirement.uncurryLevel);
    }
    std::string Old = mangler.finalize();

    NewMangling::ASTMangler NewMangler;
    std::string New = NewMangler.mangleWitnessThunk(conformance,
                                                    requirement.getDecl());

    nameBuffer = NewMangling::selectMangling(Old, New);
  }

  // If the thunked-to function is set to be always inlined, do the
  // same with the witness, on the theory that the user wants all
  // calls removed if possible, e.g. when we're able to devirtualize
  // the witness method call. Otherwise, use the default inlining
  // setting on the theory that forcing inlining off should only
  // effect the user's function, not otherwise invisible thunks.
  Inline_t InlineStrategy = InlineDefault;
  if (witnessRef.isAlwaysInline())
    InlineStrategy = AlwaysInline;

  IsFragile_t isFragile = IsNotFragile;
  if (makeModuleFragile)
    isFragile = IsFragile;
  if (witnessRef.isFragile())
    isFragile = IsFragile;

  auto *f = M.createFunction(
      linkage, nameBuffer, witnessSILFnType,
      genericEnv, SILLocation(witnessRef.getDecl()),
      IsNotBare, IsTransparent, isFragile, IsThunk,
      SILFunction::NotRelevant, InlineStrategy);

  f->setDebugScope(new (M)
                   SILDebugScope(RegularLocation(witnessRef.getDecl()), f));

  PrettyStackTraceSILFunction trace("generating protocol witness thunk", f);

  // Create the witness.
  Type selfInterfaceType;
  Type selfType;

  // If the witness is a free function, there is no Self type.
  if (!isFree) {
    if (conformance) {
      selfInterfaceType = conformance->getInterfaceType();
    } else {
      auto *proto = cast<ProtocolDecl>(requirement.getDecl()->getDeclContext());
      selfInterfaceType = proto->getSelfInterfaceType();
    }

    selfType = GenericEnvironment::mapTypeIntoContext(
        M.getSwiftModule(), genericEnv, selfInterfaceType);
  }

  SILGenFunction gen(*this, *f);

  // Open-code certain protocol witness "thunks".
  if (maybeOpenCodeProtocolWitness(gen, conformance, linkage,
                                   selfInterfaceType, selfType, genericEnv,
                                   requirement, witnessRef, witnessSubs)) {
    assert(!isFree);
    return f;
  }

  gen.emitProtocolWitness(selfType,
                          AbstractionPattern(reqtOrigTy),
                          reqtSubstTy,
                          requirement, witnessRef,
                          witnessSubs, isFree);

  return f;
}

namespace {

/// Emit a default witness table for a resilient protocol definition.
class SILGenDefaultWitnessTable
    : public SILGenWitnessTable<SILGenDefaultWitnessTable> {
  using super = SILGenWitnessTable<SILGenDefaultWitnessTable>;

public:
  SILGenModule &SGM;
  ProtocolDecl *Proto;
  SILLinkage Linkage;

  SmallVector<SILDefaultWitnessTable::Entry, 8> DefaultWitnesses;

  SILGenDefaultWitnessTable(SILGenModule &SGM, ProtocolDecl *proto,
                            SILLinkage linkage)
      : SGM(SGM), Proto(proto), Linkage(linkage) { }

  void addMissingDefault() {
    DefaultWitnesses.push_back(SILDefaultWitnessTable::Entry());
  }

  void addOutOfLineBaseProtocol(ProtocolDecl *baseProto) {
    addMissingDefault();
  }

  void addMethod(FuncDecl *fd) {
    auto witness = Proto->getDefaultWitness(fd);
    if (!witness) {
      addMissingDefault();
      return;
    }

    super::addMethod(fd, witness);
  }

  void addConstructor(ConstructorDecl *cd) {
    auto witness = Proto->getDefaultWitness(cd);
    if (!witness) {
      addMissingDefault();
      return;
    }

    super::addConstructor(cd, witness);
  }

  void addMethod(SILDeclRef requirementRef,
                 SILDeclRef witnessRef,
                 IsFreeFunctionWitness_t isFree,
                 Witness witness) {
    SILFunction *witnessFn = SGM.emitProtocolWitness(nullptr, Linkage,
                                                     requirementRef, witnessRef,
                                                     isFree, witness);
    auto entry = SILDefaultWitnessTable::Entry(requirementRef, witnessFn);
    DefaultWitnesses.push_back(entry);
  }

  void addAssociatedType(AssociatedTypeDecl *ty,
                         ArrayRef<ProtocolDecl *> protos) {
    // Add a dummy entry for the metatype itself, and then for each conformance.
    addMissingDefault();

    for (auto *protocol : protos) {
      // Only reference the witness if the protocol requires it.
      if (!Lowering::TypeConverter::protocolRequiresWitnessTable(protocol))
        continue;

      addMissingDefault();
    }
  }

  void visitAbstractStorageDecl(AbstractStorageDecl *d) {
    auto witness = Proto->getDefaultWitness(d);
    if (!witness) {
      addMissingDefault();
      if (d->isSettable(d->getDeclContext()))
        addMissingDefault();
      if (d->getMaterializeForSetFunc())
        addMissingDefault();
      return;
    }

    addAbstractStorageDecl(d, witness);
  }
};

} // end anonymous namespace

void SILGenModule::emitDefaultWitnessTable(ProtocolDecl *protocol) {
  SILLinkage linkage =
      getSILLinkage(getDeclLinkage(protocol), ForDefinition);

  SILGenDefaultWitnessTable builder(*this, protocol, linkage);
  builder.visitProtocolDecl(protocol);

  SILDefaultWitnessTable *defaultWitnesses =
      M.createDefaultWitnessTableDeclaration(protocol, linkage);
  defaultWitnesses->convertToDefinition(builder.DefaultWitnesses);
}

SILFunction *SILGenModule::
getOrCreateReabstractionThunk(GenericEnvironment *genericEnv,
                              CanSILFunctionType thunkType,
                              CanSILFunctionType fromType,
                              CanSILFunctionType toType,
                              IsFragile_t Fragile) {
  // Mangle the reabstraction thunk.
  std::string name;
  {
    Mangler mangler;

    // This is actually the SIL helper function.  For now, IR-gen
    // makes the actual thunk.
    mangler.append("_TTR");
    if (auto generics = thunkType->getGenericSignature()) {
      mangler.append(thunkType->isPseudogeneric() ? 'g' : 'G');
      mangler.setModuleContext(M.getSwiftModule());
      mangler.mangleGenericSignature(generics);
    }

    // Substitute context parameters out of the "from" and "to" types.
    auto fromInterfaceType
        = GenericEnvironment::mapTypeOutOfContext(genericEnv, fromType)
                ->getCanonicalType();
    auto toInterfaceType
        = GenericEnvironment::mapTypeOutOfContext(genericEnv, toType)
                ->getCanonicalType();

    mangler.mangleType(fromInterfaceType, /*uncurry*/ 0);
    mangler.mangleType(toInterfaceType, /*uncurry*/ 0);
    std::string Old = mangler.finalize();

    NewMangling::ASTMangler NewMangler;
    std::string New = NewMangler.mangleReabstractionThunkHelper(thunkType,
                       fromInterfaceType, toInterfaceType, M.getSwiftModule());

    name = NewMangling::selectMangling(Old, New);
  }

  auto loc = RegularLocation::getAutoGeneratedLocation();
  return M.getOrCreateSharedFunction(loc, name, thunkType, IsBare,
                                     IsTransparent, Fragile,
                                     IsReabstractionThunk);
}
