//===--- SILGenDestructor.cpp - SILGen for destructors --------------------===//
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

#include "SILGenFunction.h"
#include "RValue.h"
#include "swift/AST/AST.h"
#include "swift/SIL/TypeLowering.h"

using namespace swift;
using namespace Lowering;

void SILGenFunction::emitDestroyingDestructor(DestructorDecl *dd) {
  MagicFunctionName = DeclName(SGM.M.getASTContext().getIdentifier("deinit"));

  RegularLocation Loc(dd);
  if (dd->isImplicit())
    Loc.markAutoGenerated();

  auto cd = cast<ClassDecl>(dd->getDeclContext());
  SILValue selfValue = emitSelfDecl(dd->getImplicitSelfDecl());

  // Create a basic block to jump to for the implicit destruction behavior
  // of releasing the elements and calling the superclass destructor.
  // We won't actually emit the block until we finish with the destructor body.
  prepareEpilog(Type(), false, CleanupLocation::get(Loc));

  emitProfilerIncrement(dd->getBody());
  // Emit the destructor body.
  emitStmt(dd->getBody());

  Optional<SILValue> maybeReturnValue;
  SILLocation returnLoc(Loc);
  std::tie(maybeReturnValue, returnLoc) = emitEpilogBB(Loc);

  if (!maybeReturnValue)
    return;

  auto cleanupLoc = CleanupLocation::get(Loc);

  // If we have a superclass, invoke its destructor.
  SILValue resultSelfValue;
  SILType objectPtrTy = SILType::getNativeObjectType(F.getASTContext());
  if (cd->hasSuperclass()) {
    Type superclassTy = dd->mapTypeIntoContext(cd->getSuperclass());
    ClassDecl *superclass = superclassTy->getClassOrBoundGenericClass();
    auto superclassDtorDecl = superclass->getDestructor();
    SILDeclRef dtorConstant =
      SILDeclRef(superclassDtorDecl, SILDeclRef::Kind::Destroyer);
    SILType baseSILTy = getLoweredLoadableType(superclassTy);
    SILValue baseSelf = B.createUpcast(cleanupLoc, selfValue, baseSILTy);
    ManagedValue dtorValue;
    SILType dtorTy;
    SubstitutionList subs
      = superclassTy->gatherAllSubstitutions(SGM.M.getSwiftModule(), nullptr);
    std::tie(dtorValue, dtorTy, subs)
      = emitSiblingMethodRef(cleanupLoc, baseSelf, dtorConstant, subs);
    resultSelfValue = B.createApply(cleanupLoc, dtorValue.forward(*this),
                                    dtorTy, objectPtrTy, subs, baseSelf);
  } else {
    resultSelfValue = B.createUncheckedRefCast(cleanupLoc, selfValue,
                                                 objectPtrTy);
  }

  // Release our members.
  emitClassMemberDestruction(selfValue, cd, cleanupLoc);

  B.createReturn(returnLoc, resultSelfValue);
}

void SILGenFunction::emitDeallocatingDestructor(DestructorDecl *dd) {
  MagicFunctionName = DeclName(SGM.M.getASTContext().getIdentifier("deinit"));

  // The deallocating destructor is always auto-generated.
  RegularLocation loc(dd);
  loc.markAutoGenerated();

  // Emit the prolog.
  SILValue selfValue = emitSelfDecl(dd->getImplicitSelfDecl());

  // Form a reference to the destroying destructor.
  SILDeclRef dtorConstant(dd, SILDeclRef::Kind::Destroyer);
  auto classTy = selfValue->getType();
  ManagedValue dtorValue;
  SILType dtorTy;
  SubstitutionList subs = classTy.gatherAllSubstitutions(SGM.M);
  std::tie(dtorValue, dtorTy, subs)
    = emitSiblingMethodRef(loc, selfValue, dtorConstant, subs);

  // Call the destroying destructor.
  SILType objectPtrTy = SILType::getNativeObjectType(F.getASTContext());
  selfValue = B.createApply(loc, dtorValue.forward(*this),
                            dtorTy, objectPtrTy, subs, selfValue);

  // Deallocate the object.
  selfValue = B.createUncheckedRefCast(loc, selfValue, classTy);
  B.createDeallocRef(loc, selfValue, false);

  // Return.
  B.createReturn(loc, emitEmptyTuple(loc));
}

void SILGenFunction::emitIVarDestroyer(SILDeclRef ivarDestroyer) {
  auto cd = cast<ClassDecl>(ivarDestroyer.getDecl());
  RegularLocation loc(cd);
  loc.markAutoGenerated();

  SILValue selfValue = emitSelfDecl(cd->getDestructor()->getImplicitSelfDecl());

  auto cleanupLoc = CleanupLocation::get(loc);
  prepareEpilog(TupleType::getEmpty(getASTContext()), false, cleanupLoc);
  emitClassMemberDestruction(selfValue, cd, cleanupLoc);
  B.createReturn(loc, emitEmptyTuple(loc));
  emitEpilog(loc);
}

void SILGenFunction::emitClassMemberDestruction(SILValue selfValue,
                                                ClassDecl *cd,
                                                CleanupLocation cleanupLoc) {
  for (VarDecl *vd : cd->getStoredProperties()) {
    const TypeLowering &ti = getTypeLowering(vd->getType());
    if (!ti.isTrivial()) {
      SILValue addr = B.createRefElementAddr(cleanupLoc, selfValue, vd,
                                         ti.getLoweredType().getAddressType());
      B.createDestroyAddr(cleanupLoc, addr);
    }
  }
}

// TODO: (dmu) factor with SILGenFunction::emitDestroyingDestructor(DestructorDecl *dd)
void SILGenFunction::emitVisitRefsInInstance_dmu_(VisitRefsInInstance_dmu_Decl *dd) {
  MagicFunctionName = DeclName(SGM.M.getASTContext().getIdentifier("visitRefsInInstance_dmu_"));
  
  RegularLocation Loc(dd);
  if (dd->isImplicit())
    Loc.markAutoGenerated();

  auto cd = cast<ClassDecl>(dd->getDeclContext());
  SILValue selfValue = emitSelfDecl(dd->getImplicitSelfDecl());
  
  // Create a basic block to jump to for the implicit visitation behavior
  // of releasing the elements and calling the superclass destructor.
  // We won't actually emit the block until we finish with the destructor body.
  prepareEpilog(Type(), false, CleanupLocation::get(Loc));
  
  emitProfilerIncrement(dd->getBody());
  // Emit the destructor body.
  emitStmt(dd->getBody());
  
  Optional<SILValue> maybeReturnValue;
  SILLocation returnLoc(Loc);
  std::tie(maybeReturnValue, returnLoc) = emitEpilogBB(Loc);
  
  if (!maybeReturnValue)
    return;

  auto cleanupLoc = CleanupLocation::get(Loc);
  
  // If we have a superclass, invoke its visitor.
  // If cd is ObjC, then it won't have a visitor, but all of its fields (and its superclasses') should be
  // atomically reference-counted.
  if (cd->hasSuperclass()) {
    Type superclassTy = dd->mapTypeIntoContext(cd->getSuperclass());
    ClassDecl *superclass = superclassTy->getClassOrBoundGenericClass();
    if (!superclass->isObjC()) {
      auto superclassVisitorDecl = superclass->getVisitRefsInInstance_dmu_();
      SILDeclRef visitorConstant =
      SILDeclRef(superclassVisitorDecl, SILDeclRef::Kind::VisitRefsInInstance_dmu_);
      SILType baseSILTy = getLoweredLoadableType(superclassTy);
      SILValue baseSelf = B.createUpcast(cleanupLoc, selfValue, baseSILTy);
      ManagedValue visitorValue;
      SILType visitorTy;
      SubstitutionList subs
      = superclassTy->gatherAllSubstitutions(SGM.M.getSwiftModule(), nullptr);
      std::tie(visitorValue, visitorTy, subs)
      = emitSiblingMethodRef(cleanupLoc, baseSelf, visitorConstant, subs);
      SILType emptyType = getLoweredType(TupleType::getEmpty(SGM.M.getASTContext()));
      B.createApply(cleanupLoc, visitorValue.forward(*this),
                    visitorTy, emptyType, subs, baseSelf);
    }
  }
  emitVisitRefsInInstance_dmu_(selfValue, cd, cleanupLoc);
  
  B.createReturn(returnLoc, emitEmptyTuple(Loc));
}

// TODO: (dmu) check if we handle barrier in VisitStore, many calls to this may be redundant
void SILGenFunction::emitVisitRefsInInstance_dmu_(SILValue selfValue,
                                                ClassDecl *cd,
                                                CleanupLocation cleanupLoc) {
  // Now that SILGenFunction::emitVisitRefsInInstance_dmu_(VisitRefsInInstance_dmu_Decl *dd)
  // calls the superclass's visitor, only hit this class's properties
  for (VarDecl *vd : cd->getStoredProperties()) {
     const TypeLowering &ti = getTypeLowering(vd->getType());
    if (!ti.isTrivial()) {
      SILValue addr = B.createRefElementAddr(cleanupLoc, selfValue, vd,
                                             ti.getLoweredType().getAddressType());
      B.createVisitRefAtAddr_dmu_(cleanupLoc, addr);
    }
  }
}


void SILGenFunction::emitObjCDestructor(SILDeclRef dtor) {
  auto dd = cast<DestructorDecl>(dtor.getDecl());
  auto cd = cast<ClassDecl>(dd->getDeclContext());
  MagicFunctionName = DeclName(SGM.M.getASTContext().getIdentifier("deinit"));

  RegularLocation loc(dd);
  if (dd->isImplicit())
    loc.markAutoGenerated();

  SILValue selfValue = emitSelfDecl(dd->getImplicitSelfDecl());

  // Create a basic block to jump to for the implicit destruction behavior
  // of releasing the elements and calling the superclass destructor.
  // We won't actually emit the block until we finish with the destructor body.
  prepareEpilog(Type(), false, CleanupLocation::get(loc));

  // Emit the destructor body.
  emitStmt(dd->getBody());

  Optional<SILValue> maybeReturnValue;
  SILLocation returnLoc(loc);
  std::tie(maybeReturnValue, returnLoc) = emitEpilogBB(loc);

  if (!maybeReturnValue)
    return;

  auto cleanupLoc = CleanupLocation::get(loc);

  // Note: the ivar destroyer is responsible for destroying the
  // instance variables before the object is actually deallocated.

  // Form a reference to the superclass -dealloc.
  Type superclassTy = dd->mapTypeIntoContext(cd->getSuperclass());
  assert(superclassTy && "Emitting Objective-C -dealloc without superclass?");
  ClassDecl *superclass = superclassTy->getClassOrBoundGenericClass();
  auto superclassDtorDecl = superclass->getDestructor();
  SILDeclRef superclassDtor(superclassDtorDecl,
                            SILDeclRef::Kind::Deallocator,
                            SILDeclRef::ConstructAtBestResilienceExpansion,
                            SILDeclRef::ConstructAtNaturalUncurryLevel,
                            /*isForeign=*/true);
  auto superclassDtorType = SGM.getConstantType(superclassDtor);
  SILValue superclassDtorValue = B.createSuperMethod(
                                   cleanupLoc, selfValue, superclassDtor,
                                   superclassDtorType);

  // Call the superclass's -dealloc.
  SILType superclassSILTy = getLoweredLoadableType(superclassTy);
  SILValue superSelf = B.createUpcast(cleanupLoc, selfValue, superclassSILTy);
  SubstitutionList subs
    = superclassTy->gatherAllSubstitutions(SGM.M.getSwiftModule(), nullptr);
  auto substDtorType = superclassDtorType.castTo<SILFunctionType>()
    ->substGenericArgs(SGM.M, subs);
  SILFunctionConventions dtorConv(substDtorType, SGM.M);
  B.createApply(cleanupLoc, superclassDtorValue,
                SILType::getPrimitiveObjectType(substDtorType),
                dtorConv.getSILResultType(), subs, superSelf);

  // Return.
  B.createReturn(returnLoc, emitEmptyTuple(cleanupLoc));
}
