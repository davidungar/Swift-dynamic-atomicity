//===--- TypeCheckGeneric.cpp - Generics ----------------------------------===//
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
// This file implements support for generics.
//
//===----------------------------------------------------------------------===//
#include "TypeChecker.h"
#include "GenericTypeResolver.h"
#include "swift/AST/ArchetypeBuilder.h"
#include "swift/AST/ProtocolConformance.h"
#include "swift/AST/GenericEnvironment.h"
#include "swift/AST/Types.h"
#include "swift/Basic/Defer.h"

using namespace swift;

Type DependentGenericTypeResolver::resolveGenericTypeParamType(
                                     GenericTypeParamType *gp) {
  auto gpDecl = gp->getDecl();
  assert(gpDecl && "Missing generic parameter declaration");

  // Don't resolve generic parameters.
  return gp;
}

Type DependentGenericTypeResolver::resolveDependentMemberType(
                                     Type baseTy,
                                     DeclContext *DC,
                                     SourceRange baseRange,
                                     ComponentIdentTypeRepr *ref) {
  auto archetype = Builder.resolveArchetype(baseTy);
  assert(archetype && "Bad generic context nesting?");

  return archetype
           ->getNestedType(ref->getIdentifier(), Builder)
           ->getDependentType(GenericParams, /*allowUnresolved=*/true);
}

Type DependentGenericTypeResolver::resolveSelfAssociatedType(
       Type selfTy, AssociatedTypeDecl *assocType) {
  auto archetype = Builder.resolveArchetype(selfTy);
  assert(archetype && "Bad generic context nesting?");
  
  return archetype
           ->getNestedType(assocType, Builder)
           ->getDependentType(GenericParams, /*allowUnresolved=*/true);
}

Type DependentGenericTypeResolver::resolveTypeOfContext(DeclContext *dc) {
  return dc->getSelfInterfaceType();
}

Type DependentGenericTypeResolver::resolveTypeOfDecl(TypeDecl *decl) {
  return decl->getDeclaredInterfaceType();
}

bool DependentGenericTypeResolver::areSameType(Type type1, Type type2) {
  if (!type1->hasTypeParameter() && !type2->hasTypeParameter())
    return type1->isEqual(type2);

  auto pa1 = Builder.resolveArchetype(type1);
  auto pa2 = Builder.resolveArchetype(type2);
  if (pa1 && pa2)
    return pa1->getArchetypeAnchor(Builder) == pa2->getArchetypeAnchor(Builder);

  return type1->isEqual(type2);
}

void DependentGenericTypeResolver::recordParamType(ParamDecl *decl, Type type) {
  // Do nothing
}

Type GenericTypeToArchetypeResolver::resolveGenericTypeParamType(
                                       GenericTypeParamType *gp) {
  auto gpDecl = gp->getDecl();
  assert(gpDecl && "Missing generic parameter declaration");

  // Hack: See parseGenericParameters(). When the issue there is fixed,
  // we won't need the isInvalid() check anymore.
  if (gpDecl->isInvalid() || !GenericEnv)
    return ErrorType::get(gpDecl->getASTContext());

  return GenericEnv->mapTypeIntoContext(gp);
}

Type GenericTypeToArchetypeResolver::resolveDependentMemberType(
                                  Type baseTy,
                                  DeclContext *DC,
                                  SourceRange baseRange,
                                  ComponentIdentTypeRepr *ref) {
  llvm_unreachable("Dependent type after archetype substitution");
}

Type GenericTypeToArchetypeResolver::resolveSelfAssociatedType(
       Type selfTy, AssociatedTypeDecl *assocType) {
  llvm_unreachable("Dependent type after archetype substitution");  
}

Type GenericTypeToArchetypeResolver::resolveTypeOfContext(DeclContext *dc) {
  return GenericEnvironment::mapTypeIntoContext(
      dc->getParentModule(), GenericEnv,
      dc->getSelfInterfaceType());
}

Type GenericTypeToArchetypeResolver::resolveTypeOfDecl(TypeDecl *decl) {
  return GenericEnvironment::mapTypeIntoContext(
      decl->getDeclContext()->getParentModule(), GenericEnv,
      decl->getDeclaredInterfaceType());
}

bool GenericTypeToArchetypeResolver::areSameType(Type type1, Type type2) {
  return type1->isEqual(type2);
}

void GenericTypeToArchetypeResolver::recordParamType(ParamDecl *decl, Type type) {
  decl->setType(type);

  // When type checking a closure or subscript index, this is the only
  // resolver that runs, so make sure we also set the interface type,
  // if one was not already set.
  //
  // When type checking functions, the CompleteGenericTypeResolver sets
  // the interface type.
  if (!decl->hasInterfaceType())
    decl->setInterfaceType(GenericEnvironment::mapTypeOutOfContext(
        GenericEnv, type));
}

Type CompleteGenericTypeResolver::resolveGenericTypeParamType(
                                              GenericTypeParamType *gp) {
  assert(gp->getDecl() && "Missing generic parameter declaration");
  return gp;
}


Type CompleteGenericTypeResolver::resolveDependentMemberType(
                                    Type baseTy,
                                    DeclContext *DC,
                                    SourceRange baseRange,
                                    ComponentIdentTypeRepr *ref) {
  // Resolve the base to a potential archetype.
  auto basePA = Builder.resolveArchetype(baseTy);
  assert(basePA && "Missing potential archetype for base");

  // Retrieve the potential archetype for the nested type.
  auto nestedPA = basePA->getNestedType(ref->getIdentifier(), Builder);

  // If this potential archetype was renamed due to typo correction,
  // complain and fix it.
  if (nestedPA->wasRenamed()) {
    auto newName = nestedPA->getName();
    TC.diagnose(ref->getIdLoc(), diag::invalid_member_type_suggest,
                baseTy, ref->getIdentifier(), newName)
      .fixItReplace(ref->getIdLoc(), newName.str());
    ref->overwriteIdentifier(newName);
    nestedPA->setAlreadyDiagnosedRename();
    
    // Go get the actual nested type.
    nestedPA = basePA->getNestedType(newName, Builder);
    assert(!nestedPA->wasRenamed());
  }

  // If the nested type has been resolved to an associated type, use it.
  if (auto assocType = nestedPA->getResolvedAssociatedType()) {
    ref->setValue(assocType);
    return DependentMemberType::get(baseTy, assocType);
  }

  // If the nested type comes from a type alias, use either the alias's
  // concrete type, or resolve its components down to another dependent member.
  if (auto alias = nestedPA->getTypeAliasDecl()) {
    assert(!alias->getGenericParams() && "Generic typealias in protocol");
    ref->setValue(alias);
    return TC.substMemberTypeWithBase(DC->getParentModule(), alias, baseTy);
  }
  
  Identifier name = ref->getIdentifier();
  SourceLoc nameLoc = ref->getIdLoc();

  // Check whether the name can be found in the superclass.
  // FIXME: The archetype builder should be doing this and mapping down to a
  // concrete type.
  if (auto superclassTy = basePA->getSuperclass()) {
    if (auto lookup = TC.lookupMemberType(DC, superclassTy, name)) {
      if (lookup.isAmbiguous()) {
        TC.diagnoseAmbiguousMemberType(baseTy, baseRange, name, nameLoc,
                                       lookup);
        return ErrorType::get(TC.Context);
      }

      ref->setValue(lookup.front().first);
      // FIXME: Record (via type sugar) that this was referenced via baseTy.
      return lookup.front().second;
    }
  }

  // Complain that there is no suitable type.
  TC.diagnose(nameLoc, diag::invalid_member_type, name, baseTy)
    .highlight(baseRange);
  return ErrorType::get(TC.Context);
}

Type CompleteGenericTypeResolver::resolveSelfAssociatedType(
       Type selfTy, AssociatedTypeDecl *assocType) {
  return Builder.resolveArchetype(selfTy)
           ->getNestedType(assocType, Builder)
           ->getDependentType(GenericParams, /*allowUnresolved=*/false);
}

Type CompleteGenericTypeResolver::resolveTypeOfContext(DeclContext *dc) {
  return dc->getSelfInterfaceType();
}

Type CompleteGenericTypeResolver::resolveTypeOfDecl(TypeDecl *decl) {
  return decl->getDeclaredInterfaceType();
}

bool CompleteGenericTypeResolver::areSameType(Type type1, Type type2) {
  if (!type1->hasTypeParameter() && !type2->hasTypeParameter())
    return type1->isEqual(type2);

  auto pa1 = Builder.resolveArchetype(type1);
  auto pa2 = Builder.resolveArchetype(type2);
  if (pa1 && pa2)
    return pa1->getArchetypeAnchor(Builder) == pa2->getArchetypeAnchor(Builder);

  return type1->isEqual(type2);
}

void
CompleteGenericTypeResolver::recordParamType(ParamDecl *decl, Type type) {
  decl->setInterfaceType(type);
}

/// Check the generic parameters in the given generic parameter list (and its
/// parent generic parameter lists) according to the given resolver.
void TypeChecker::checkGenericParamList(ArchetypeBuilder *builder,
                                        GenericParamList *genericParams,
                                        GenericSignature *parentSig,
                                        GenericTypeResolver *resolver) {
  // If there is a parent context, add the generic parameters and requirements
  // from that context.
  if (builder)
    builder->addGenericSignature(parentSig);

  // If there aren't any generic parameters at this level, we're done.
  if (!genericParams)
    return;

  assert(genericParams->size() > 0 && "Parsed an empty generic parameter list?");

  // Determine where and how to perform name lookup for the generic
  // parameter lists and where clause.
  TypeResolutionOptions options;
  DeclContext *lookupDC = genericParams->begin()[0]->getDeclContext();
  if (!lookupDC->isModuleScopeContext()) {
    assert(isa<GenericTypeDecl>(lookupDC) || isa<ExtensionDecl>(lookupDC) ||
           isa<AbstractFunctionDecl>(lookupDC) &&
           "not a proper generic parameter context?");
    options = TR_GenericSignature;
  }    

  // First, add the generic parameters to the archetype builder.
  // Do this before checking the inheritance clause, since it may
  // itself be dependent on one of these parameters.
  if (builder) {
    for (auto param : *genericParams)
      builder->addGenericParameter(param);
  }

  // Now, check the inheritance clauses of each parameter.
  for (auto param : *genericParams) {
    checkInheritanceClause(param, resolver);

    if (builder) {
      builder->addGenericParameterRequirements(param);

      // Infer requirements from the inherited types.
      for (const auto &inherited : param->getInherited()) {
        builder->inferRequirements(inherited, genericParams);
      }
    }
  }

  // Visit each of the requirements, adding them to the builder.
  // Add the requirements clause to the builder, validating the types in
  // the requirements clause along the way.
  for (auto &req : genericParams->getRequirements()) {
    if (req.isInvalid())
      continue;

    switch (req.getKind()) {
    case RequirementReprKind::TypeConstraint: {
      // Validate the types.
      if (validateType(req.getSubjectLoc(), lookupDC, options, resolver)) {
        req.setInvalid();
        continue;
      }

      if (validateType(req.getConstraintLoc(), lookupDC, options,
                       resolver)) {
        req.setInvalid();
        continue;
      }

      // FIXME: Feels too early to perform this check.
      if (!req.getConstraint()->isExistentialType() &&
          !req.getConstraint()->getClassOrBoundGenericClass()) {
        diagnose(genericParams->getWhereLoc(),
                 diag::requires_conformance_nonprotocol,
                 req.getSubjectLoc(), req.getConstraintLoc());
        req.getConstraintLoc().setInvalidType(Context);
        req.setInvalid();
        continue;
      }

      break;
    }

    case RequirementReprKind::LayoutConstraint: {
      // Validate the types.
      if (validateType(req.getSubjectLoc(), lookupDC, options, resolver)) {
        req.setInvalid();
        continue;
      }

      if (req.getLayoutConstraintLoc().isNull()) {
        req.setInvalid();
        continue;
      }

      break;
    }

    case RequirementReprKind::SameType:
      if (validateType(req.getFirstTypeLoc(), lookupDC, options,
                       resolver)) {
        req.setInvalid();
        continue;
      }

      if (validateType(req.getSecondTypeLoc(), lookupDC, options,
                       resolver)) {
        req.setInvalid();
        continue;
      }
      
      break;
    }
    
    if (builder && builder->addRequirement(req))
      req.setInvalid();
  }
}

/// Check the signature of a generic function.
static bool checkGenericFuncSignature(TypeChecker &tc,
                                      ArchetypeBuilder *builder,
                                      AbstractFunctionDecl *func,
                                      GenericTypeResolver &resolver) {
  bool badType = false;

  // Check the generic parameter list.
  auto genericParams = func->getGenericParams();

  tc.checkGenericParamList(
                         builder, genericParams,
                         func->getDeclContext()->getGenericSignatureOfContext(),
                         &resolver);

  // Check the parameter patterns.
  for (auto params : func->getParameterLists()) {
    // Check the pattern.
    if (tc.typeCheckParameterList(params, func, TypeResolutionOptions(),
                                  resolver))
      badType = true;

    // Infer requirements from the pattern.
    if (builder) {
      builder->inferRequirements(params, genericParams);
    }
  }

  // If there is a declared result type, check that as well.
  if (auto fn = dyn_cast<FuncDecl>(func)) {
    if (!fn->getBodyResultTypeLoc().isNull()) {
      // Check the result type of the function.
      TypeResolutionOptions options;
      if (fn->hasDynamicSelf())
        options |= TR_DynamicSelfResult;

      if (tc.validateType(fn->getBodyResultTypeLoc(), fn, options, &resolver)) {
        badType = true;
      }

      // Infer requirements from it.
      if (builder && fn->getBodyResultTypeLoc().getTypeRepr()) {
        builder->inferRequirements(fn->getBodyResultTypeLoc(), genericParams);
      }
    }
  }

  return badType;
}

/// Determine whether the given type is \c Self, an associated type of \c Self,
/// or a concrete type.
static bool isSelfDerivedOrConcrete(Type protoSelf, Type type) {
  // Check for a concrete type.
  if (!type->hasTypeParameter())
    return true;

  // Unwrap dependent member types.
  while (auto depMem = type->getAs<DependentMemberType>()) {
    type = depMem->getBase();
  }

  if (type->is<GenericTypeParamType>())
    return type->isEqual(protoSelf);

  return false;
}

void
TypeChecker::prepareGenericParamList(GenericParamList *gp,
                                     DeclContext *dc) {
  Accessibility access;
  if (auto *fd = dyn_cast<FuncDecl>(dc))
    access = fd->getFormalAccess();
  else if (auto *nominal = dyn_cast<NominalTypeDecl>(dc))
    access = nominal->getFormalAccess();
  else
    access = Accessibility::Internal;
  access = std::max(access, Accessibility::Internal);

  unsigned depth = gp->getDepth();
  for (auto paramDecl : *gp) {
    paramDecl->setDepth(depth);
    if (!paramDecl->hasAccessibility())
      paramDecl->setAccessibility(access);
  }
}

/// Add the generic parameter types from the given list to the vector.
static void addGenericParamTypes(GenericParamList *gpList,
                                 SmallVectorImpl<GenericTypeParamType *> &params) {
  if (!gpList) return;

  for (auto gpDecl : *gpList) {
    params.push_back(
            gpDecl->getDeclaredInterfaceType()->castTo<GenericTypeParamType>());
  }
}

GenericSignature *
TypeChecker::validateGenericFuncSignature(AbstractFunctionDecl *func) {
  bool invalid = false;

  auto *gp = func->getGenericParams();
  if (gp)
    prepareGenericParamList(gp, func);

  // Collect the generic parameters.
  SmallVector<GenericTypeParamType *, 4> allGenericParams;
  if (auto parentSig = func->getDeclContext()->getGenericSignatureOfContext())
    allGenericParams.append(parentSig->getGenericParams().begin(),
                            parentSig->getGenericParams().end());
  addGenericParamTypes(gp, allGenericParams);

  // Create the archetype builder.
  ArchetypeBuilder builder = createArchetypeBuilder(func->getParentModule());

  // Type check the function declaration, treating all generic type
  // parameters as dependent, unresolved.
  DependentGenericTypeResolver dependentResolver(builder, allGenericParams);
  if (checkGenericFuncSignature(*this, &builder, func, dependentResolver))
    invalid = true;

  // Finalize the generic requirements.
  (void)builder.finalize(func->getLoc(), allGenericParams);

  // The archetype builder now has all of the requirements, although there might
  // still be errors that have not yet been diagnosed. Revert the generic
  // function signature and type-check it again, completely.
  revertGenericFuncSignature(func);
  if (gp)
    revertGenericParamList(gp);

  CompleteGenericTypeResolver completeResolver(*this, builder,
                                               allGenericParams);
  if (checkGenericFuncSignature(*this, nullptr, func, completeResolver))
    invalid = true;
  if (builder.diagnoseRemainingRenames(func->getLoc(), allGenericParams))
    invalid = true;

  // The generic function signature is complete and well-formed. Determine
  // the type of the generic function.
  auto sig = builder.getGenericSignature();

  // For a generic requirement in a protocol, make sure that the requirement
  // set didn't add any requirements to Self or its associated types.
  if (!invalid && func->getGenericParams() &&
      isa<ProtocolDecl>(func->getDeclContext())) {
    auto proto = cast<ProtocolDecl>(func->getDeclContext());
    auto protoSelf = proto->getSelfInterfaceType();
    for (auto req : sig->getRequirements()) {
      // If one of the types in the requirement is dependent on a non-Self
      // type parameter, this requirement is okay.
      if (!isSelfDerivedOrConcrete(protoSelf, req.getFirstType()) ||
          !isSelfDerivedOrConcrete(protoSelf, req.getSecondType()))
        continue;

      // The conformance of 'Self' to the protocol is okay.
      if (req.getKind() == RequirementKind::Conformance &&
          req.getSecondType()->getAs<ProtocolType>()->getDecl() == proto &&
          req.getFirstType()->is<GenericTypeParamType>())
        continue;

      diagnose(func,
               Context.LangOpts.EffectiveLanguageVersion[0] >= 4
                 ? diag::requirement_restricts_self
                 : diag::requirement_restricts_self_swift3,
               func->getDescriptiveKind(), func->getFullName(),
               req.getFirstType().getString(),
               static_cast<unsigned>(req.getKind()),
               req.getSecondType().getString());

      if (Context.LangOpts.EffectiveLanguageVersion[0] >= 4)
        invalid = true;
    }
  }

  // Debugging of the archetype builder and generic signature generation.
  if (Context.LangOpts.DebugGenericSignatures) {
    func->dumpRef(llvm::errs());
    llvm::errs() << "\n";
    builder.dump(llvm::errs());
    llvm::errs() << "Generic signature: ";
    sig->print(llvm::errs());
    llvm::errs() << "\n";
    llvm::errs() << "Canonical generic signature: ";
    sig->getCanonicalSignature()->print(llvm::errs());
    llvm::errs() << "\n";
  }

  if (invalid) {
    func->setInterfaceType(ErrorType::get(Context));
    func->setInvalid();
    // null doesn't mean error here: callers still expect the signature.
    return sig;
  }

  configureInterfaceType(func, sig);
  return sig;
}

void TypeChecker::configureInterfaceType(AbstractFunctionDecl *func,
                                         GenericSignature *sig) {
  Type funcTy;
  Type initFuncTy;

  if (auto fn = dyn_cast<FuncDecl>(func)) {
    funcTy = fn->getBodyResultTypeLoc().getType();
    if (!funcTy)
      funcTy = TupleType::getEmpty(Context);

  } else if (auto ctor = dyn_cast<ConstructorDecl>(func)) {
    auto *dc = ctor->getDeclContext();

    funcTy = dc->getSelfInterfaceType();
    if (!funcTy)
      funcTy = ErrorType::get(Context);

    // Adjust result type for failability.
    if (ctor->getFailability() != OTK_None)
      funcTy = OptionalType::get(ctor->getFailability(), funcTy);

    initFuncTy = funcTy;
  } else {
    assert(isa<DestructorDecl>(func) || isa<MakeContainedReferencesCountAtomicallyDecl>(func)); // dmu
    funcTy = TupleType::getEmpty(Context);
  }

  auto paramLists = func->getParameterLists();
  SmallVector<ParameterList*, 4> storedParamLists;

  // FIXME: Destructors don't have the '()' pattern in their signature, so
  // paste it here.
  if (isa<DestructorDecl>(func) || isa<MakeContainedReferencesCountAtomicallyDecl>(func)) { // dmu
    assert(paramLists.size() == 1 && "Only the self paramlist");
    storedParamLists.push_back(paramLists[0]);
    storedParamLists.push_back(ParameterList::createEmpty(Context));
    paramLists = storedParamLists;
  }

  bool hasSelf = func->getDeclContext()->isTypeContext();
  for (unsigned i = 0, e = paramLists.size(); i != e; ++i) {
    Type argTy;
    Type initArgTy;

    Type selfTy;
    if (i == e-1 && hasSelf) {
      selfTy = func->computeInterfaceSelfType();
      // Substitute in our own 'self' parameter.

      argTy = selfTy;
      if (initFuncTy) {
        initArgTy = func->computeInterfaceSelfType(/*isInitializingCtor=*/true);
      }
    } else {
      argTy = paramLists[e - i - 1]->getInterfaceType(Context);

      if (initFuncTy)
        initArgTy = argTy;
    }

    // 'throws' only applies to the innermost function.
    AnyFunctionType::ExtInfo info;
    if (i == 0 && func->hasThrows())
      info = info.withThrows();

    assert(!argTy->hasArchetype());
    assert(!funcTy->hasArchetype());
    if (initFuncTy)
      assert(!initFuncTy->hasArchetype());

    if (sig && i == e-1) {
      funcTy = GenericFunctionType::get(sig, argTy, funcTy, info);
      if (initFuncTy)
        initFuncTy = GenericFunctionType::get(sig, initArgTy, initFuncTy, info);
    } else {
      funcTy = FunctionType::get(argTy, funcTy, info);
      if (initFuncTy)
        initFuncTy = FunctionType::get(initArgTy, initFuncTy, info);
    }
  }

  // Record the interface type.
  func->setInterfaceType(funcTy);
  if (initFuncTy)
    cast<ConstructorDecl>(func)->setInitializerInterfaceType(initFuncTy);

  if (func->getGenericParams()) {
    // Collect all generic params referenced in parameter types,
    // return type or requirements.
    SmallPtrSet<GenericTypeParamDecl *, 4> referencedGenericParams;

    auto visitorFn = [&referencedGenericParams](Type t) {
      if (auto *paramTy = t->getAs<GenericTypeParamType>())
        referencedGenericParams.insert(paramTy->getDecl());
    };

    funcTy->castTo<AnyFunctionType>()->getInput().visit(visitorFn);
    funcTy->castTo<AnyFunctionType>()->getResult().visit(visitorFn);

    auto requirements = sig->getRequirements();
    for (auto req : requirements) {
      if (req.getKind() == RequirementKind::SameType) {
        // Same type requirements may allow for generic
        // inference, even if this generic parameter
        // is not mentioned in the function signature.
        // TODO: Make the test more precise.
        auto left = req.getFirstType();
        auto right = req.getSecondType();
        // For now consider any references inside requirements
        // as a possibility to infer the generic type.
        left.visit(visitorFn);
        right.visit(visitorFn);
      }
    }

    // Find the depth of the function's own generic parameters.
    unsigned fnGenericParamsDepth = func->getGenericParams()->getDepth();

    // Check that every generic parameter type from the signature is
    // among referencedArchetypes.
    for (auto *genParam : sig->getGenericParams()) {
      auto *paramDecl = genParam->getDecl();
      if (paramDecl->getDepth() != fnGenericParamsDepth)
        continue;
      if (!referencedGenericParams.count(paramDecl)) {
        // Produce an error that this generic parameter cannot be bound.
        diagnose(paramDecl->getLoc(), diag::unreferenced_generic_parameter,
                 paramDecl->getNameStr());
        func->setInterfaceType(ErrorType::get(Context));
        func->setInvalid();
      }
    }
  }
}

/// Visit the given generic parameter lists from the outermost to the innermost,
/// calling the visitor function for each list.
static void visitOuterToInner(
                      GenericParamList *genericParams,
                      llvm::function_ref<void(GenericParamList *)> visitor) {
  if (auto outerGenericParams = genericParams->getOuterParameters())
    visitOuterToInner(outerGenericParams, visitor);

  visitor(genericParams);
}

GenericEnvironment *TypeChecker::checkGenericEnvironment(
                      GenericParamList *genericParams,
                      DeclContext *dc,
                      GenericSignature *parentSig,
                      bool allowConcreteGenericParams,
                      llvm::function_ref<void(ArchetypeBuilder &)>
                        inferRequirements) {
  assert(genericParams && "Missing generic parameters?");
  bool recursivelyVisitGenericParams =
    genericParams->getOuterParameters() && !parentSig;

  // Collect the generic parameters.
  SmallVector<GenericTypeParamType *, 4> allGenericParams;
  if (recursivelyVisitGenericParams) {
    visitOuterToInner(genericParams,
                      [&](GenericParamList *gpList) {
      addGenericParamTypes(gpList, allGenericParams);
    });
  } else {
    if (parentSig) {
      allGenericParams.append(parentSig->getGenericParams().begin(),
                              parentSig->getGenericParams().end());
    }

    addGenericParamTypes(genericParams, allGenericParams);
  }

  // Create the archetype builder.
  ModuleDecl *module = dc->getParentModule();
  ArchetypeBuilder builder = createArchetypeBuilder(module);

  // Type check the generic parameters, treating all generic type
  // parameters as dependent, unresolved.
  DependentGenericTypeResolver dependentResolver(builder, allGenericParams);
  if (recursivelyVisitGenericParams) {
    visitOuterToInner(genericParams,
                      [&](GenericParamList *gpList) {
      checkGenericParamList(&builder, gpList, nullptr, &dependentResolver);
    });
  } else {
    checkGenericParamList(&builder, genericParams, parentSig,
                          &dependentResolver);
  }

  /// Perform any necessary requirement inference.
  inferRequirements(builder);

  // Finalize the generic requirements.
  (void)builder.finalize(genericParams->getSourceRange().Start,
                         allGenericParams,
                         allowConcreteGenericParams);

  // The archetype builder now has all of the requirements, although there might
  // still be errors that have not yet been diagnosed. Revert the signature
  // and type-check it again, completely.
  if (recursivelyVisitGenericParams) {
    visitOuterToInner(genericParams,
                      [&](GenericParamList *gpList) {
      revertGenericParamList(gpList);
    });
  } else {
    revertGenericParamList(genericParams);
  }

  CompleteGenericTypeResolver completeResolver(*this, builder,
                                               allGenericParams);
  if (recursivelyVisitGenericParams) {
    visitOuterToInner(genericParams,
                      [&](GenericParamList *gpList) {
      checkGenericParamList(nullptr, gpList, nullptr, &completeResolver);
    });
  } else {
    checkGenericParamList(nullptr, genericParams, parentSig,
                          &completeResolver);
  }

  // Complain about any other renamed references.
  (void)builder.diagnoseRemainingRenames(genericParams->getSourceRange().Start,
                                         allGenericParams);

  // Record the generic type parameter types and the requirements.
  auto sig = builder.getGenericSignature();

  // Debugging of the archetype builder and generic signature generation.
  if (Context.LangOpts.DebugGenericSignatures) {
    dc->printContext(llvm::errs());
    llvm::errs() << "\n";
    builder.dump(llvm::errs());
    llvm::errs() << "Generic signature: ";
    sig->print(llvm::errs());
    llvm::errs() << "\n";
    llvm::errs() << "Canonical generic signature: ";
    sig->getCanonicalSignature()->print(llvm::errs());
    llvm::errs() << "\n";
  }

  // Form the generic environment.
  return builder.getGenericEnvironment(sig);
}

static void revertDependentTypeLoc(TypeLoc &tl) {
  // If there's no type representation, there's nothing to revert.
  if (!tl.getTypeRepr())
    return;

  // Don't revert an error type; we've already complained.
  if (tl.wasValidated() && tl.isError())
    return;

  // Make sure we validate the type again.
  tl.setType(Type(), /*validated=*/false);
}

/// Revert the dependent types within the given generic parameter list.
void TypeChecker::revertGenericParamList(GenericParamList *genericParams) {
  // Revert the inherited clause of the generic parameter list.
  for (auto param : *genericParams) {
    param->setCheckedInheritanceClause(false);
    for (auto &inherited : param->getInherited())
      revertDependentTypeLoc(inherited);
  }

  // Revert the requirements of the generic parameter list.
  for (auto &req : genericParams->getRequirements()) {
    if (req.isInvalid())
      continue;

    switch (req.getKind()) {
    case RequirementReprKind::TypeConstraint: {
      revertDependentTypeLoc(req.getSubjectLoc());
      revertDependentTypeLoc(req.getConstraintLoc());
      break;
    }
    case RequirementReprKind::LayoutConstraint: {
      revertDependentTypeLoc(req.getSubjectLoc());
      break;
    }
    case RequirementReprKind::SameType:
      revertDependentTypeLoc(req.getFirstTypeLoc());
      revertDependentTypeLoc(req.getSecondTypeLoc());
      break;
    }
  }
}

void TypeChecker::validateGenericTypeSignature(GenericTypeDecl *typeDecl) {
  auto *gp = typeDecl->getGenericParams();
  auto *dc = typeDecl->getDeclContext();

  if (!gp) {
    auto *parentEnv = dc->getGenericEnvironmentOfContext();
    typeDecl->setGenericEnvironment(parentEnv);
    return;
  }

  gp->setOuterParameters(dc->getGenericParamsOfContext());

  prepareGenericParamList(gp, typeDecl);

  auto *env = checkGenericEnvironment(gp, dc, dc->getGenericSignatureOfContext(),
                                      /*allowConcreteGenericParams=*/false);
  typeDecl->setGenericEnvironment(env);
}

void TypeChecker::revertGenericFuncSignature(AbstractFunctionDecl *func) {
  // Revert the result type.
  if (auto fn = dyn_cast<FuncDecl>(func))
    if (!fn->getBodyResultTypeLoc().isNull())
      revertDependentTypeLoc(fn->getBodyResultTypeLoc());

  // Revert the body parameter types.
  for (auto paramList : func->getParameterLists()) {
    for (auto &param : *paramList) {
      revertDependentTypeLoc(param->getTypeLoc());
    }
  }
}

std::pair<bool, bool> TypeChecker::checkGenericArguments(
    DeclContext *dc, SourceLoc loc, SourceLoc noteLoc, Type owner,
    GenericSignature *genericSig, const TypeSubstitutionMap &substitutions,
    UnsatisfiedDependency *unsatisfiedDependency,
    ConformanceCheckOptions conformanceOptions,
    GenericRequirementsCheckListener *listener) {
  // Check each of the requirements.
  ModuleDecl *module = dc->getParentModule();
  for (const auto &req : genericSig->getRequirements()) {
    Type firstType = req.getFirstType().subst(
        QueryTypeSubstitutionMap{substitutions},
        LookUpConformanceInModule(module));
    if (firstType.isNull()) {
      // Another requirement will fail later; just continue.
      continue;
    }

    Type secondType;
    if (req.getKind() != RequirementKind::Layout)
      secondType = req.getSecondType();
    if (secondType) {
      secondType = secondType.subst(QueryTypeSubstitutionMap{substitutions},
                                    LookUpConformanceInModule(module));
      if (secondType.isNull()) {
        // Another requirement will fail later; just continue.
        continue;
      }
    }

    auto kind = req.getKind();
    if (listener && !listener->shouldCheck(kind, firstType, secondType))
      continue;

    switch (kind) {
    case RequirementKind::Conformance: {
      // Protocol conformance requirements.
      auto proto = secondType->castTo<ProtocolType>();
      // FIXME: This should track whether this should result in a private
      // or non-private dependency.
      // FIXME: Do we really need "used" at this point?
      // FIXME: Poor location information. How much better can we do here?
      auto result =
          conformsToProtocol(firstType, proto->getDecl(), dc,
                             conformanceOptions, loc, unsatisfiedDependency);

      // Unsatisfied dependency case.
      if (result.first)
        return std::make_pair(true, false);

      // Conformance check failure case.
      if (!result.second)
        return std::make_pair(false, false);

      continue;
    }

    case RequirementKind::Layout: {
      // TODO: Statically check if a the first type
      // conforms to the layout constraint, once we
      // support such static checks.
      continue;
    }

    case RequirementKind::Superclass:
      // Superclass requirements.
      if (!isSubtypeOf(firstType, secondType, dc)) {
        // FIXME: Poor source-location information.
        diagnose(loc, diag::type_does_not_inherit, owner, firstType,
                 secondType);

        diagnose(noteLoc, diag::type_does_not_inherit_requirement,
                 req.getFirstType(), req.getSecondType(),
                 genericSig->gatherGenericParamBindingsText(
                     {req.getFirstType(), req.getSecondType()}, substitutions));

        return std::make_pair(false, false);
      }
      continue;

    case RequirementKind::SameType:
      if (!firstType->isEqual(secondType)) {
        // FIXME: Better location info for both diagnostics.
        diagnose(loc, diag::types_not_equal, owner, firstType, secondType);

        diagnose(noteLoc, diag::types_not_equal_requirement, req.getFirstType(),
                 req.getSecondType(),
                 genericSig->gatherGenericParamBindingsText(
                     {req.getFirstType(), req.getSecondType()}, substitutions));

        return std::make_pair(false, false);
      }
      continue;
    }
  }

  return std::make_pair(false, true);
}

Type TypeChecker::getWitnessType(Type type, ProtocolDecl *protocol,
                                 ProtocolConformanceRef conformance,
                                 Identifier name,
                                 Diag<> brokenProtocolDiag) {
  Type ty = ProtocolConformance::getTypeWitnessByName(type, conformance,
                                                      name, this);
  if (!ty &&
      !(conformance.isConcrete() && conformance.getConcrete()->isInvalid()))
    diagnose(protocol->getLoc(), brokenProtocolDiag);

  return ty;
}
