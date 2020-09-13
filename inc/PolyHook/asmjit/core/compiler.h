// AsmJit - Machine code generation for C++
//
//  * Official AsmJit Home Page: https://asmjit.com
//  * Official Github Repository: https://github.com/asmjit/asmjit
//
// Copyright (c) 2008-2020 The AsmJit Authors
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.

#ifndef ASMJIT_CORE_COMPILER_H_INCLUDED
#define ASMJIT_CORE_COMPILER_H_INCLUDED

#include "../core/api-config.h"
#ifndef ASMJIT_NO_COMPILER

#include "../core/assembler.h"
#include "../core/builder.h"
#include "../core/constpool.h"
#include "../core/func.h"
#include "../core/inst.h"
#include "../core/operand.h"
#include "../core/support.h"
#include "../core/zone.h"
#include "../core/zonevector.h"

ASMJIT_BEGIN_NAMESPACE

// ============================================================================
// [Forward Declarations]
// ============================================================================

struct RATiedReg;
class RAWorkReg;

class JumpAnnotation;

class JumpNode;
class FuncNode;
class FuncRetNode;
class InvokeNode;

//! \addtogroup asmjit_compiler
//! \{

// ============================================================================
// [asmjit::VirtReg]
// ============================================================================

//! Virtual register data, managed by \ref BaseCompiler.
class VirtReg {
public:
  ASMJIT_NONCOPYABLE(VirtReg)

  //! Virtual register id.
  uint32_t _id;
  //! Virtual register info (signature).
  RegInfo _info;
  //! Virtual register size (can be smaller than `regInfo._size`).
  uint32_t _virtSize;
  //! Virtual register alignment (for spilling).
  uint8_t _alignment;
  //! Type-id.
  uint8_t _typeId;
  //! Virtual register weight for alloc/spill decisions.
  uint8_t _weight;
  //! True if this is a fixed register, never reallocated.
  uint8_t _isFixed : 1;
  //! True if the virtual register is only used as a stack (never accessed as register).
  uint8_t _isStack : 1;
  uint8_t _reserved : 6;

  //! Virtual register name (user provided or automatically generated).
  ZoneString<16> _name;

  // -------------------------------------------------------------------------
  // The following members are used exclusively by RAPass. They are initialized
  // when the VirtReg is created to NULL pointers and then changed during RAPass
  // execution. RAPass sets them back to NULL before it returns.
  // -------------------------------------------------------------------------

  //! Reference to `RAWorkReg`, used during register allocation.
  RAWorkReg* _workReg;

  //! \name Construction & Destruction
  //! \{

  inline VirtReg(uint32_t id, uint32_t signature, uint32_t virtSize, uint32_t alignment, uint32_t typeId) noexcept
    : _id(id),
      _virtSize(virtSize),
      _alignment(uint8_t(alignment)),
      _typeId(uint8_t(typeId)),
      _weight(1),
      _isFixed(false),
      _isStack(false),
      _reserved(0),
      _name(),
      _workReg(nullptr) { _info._signature = signature; }

  //! \}

  //! \name Accessors
  //! \{

  //! Returns the virtual register id.
  inline uint32_t id() const noexcept { return _id; }

  //! Returns the virtual register name.
  inline const char* name() const noexcept { return _name.data(); }
  //! Returns the size of the virtual register name.
  inline uint32_t nameSize() const noexcept { return _name.size(); }

  //! Returns a register information that wraps the register signature.
  inline const RegInfo& info() const noexcept { return _info; }
  //! Returns a virtual register type (maps to the physical register type as well).
  inline uint32_t type() const noexcept { return _info.type(); }
  //! Returns a virtual register group (maps to the physical register group as well).
  inline uint32_t group() const noexcept { return _info.group(); }

  //! Returns a real size of the register this virtual register maps to.
  //!
  //! For example if this is a 128-bit SIMD register used for a scalar single
  //! precision floating point value then its virtSize would be 4, however, the
  //! `regSize` would still say 16 (128-bits), because it's the smallest size
  //! of that register type.
  inline uint32_t regSize() const noexcept { return _info.size(); }

  //! Returns a register signature of this virtual register.
  inline uint32_t signature() const noexcept { return _info.signature(); }

  //! Returns the virtual register size.
  //!
  //! The virtual register size describes how many bytes the virtual register
  //! needs to store its content. It can be smaller than the physical register
  //! size, see `regSize()`.
  inline uint32_t virtSize() const noexcept { return _virtSize; }

  //! Returns the virtual register alignment.
  inline uint32_t alignment() const noexcept { return _alignment; }

  //! Returns the virtual register type id, see `Type::Id`.
  inline uint32_t typeId() const noexcept { return _typeId; }

  //! Returns the virtual register weight - the register allocator can use it
  //! as explicit hint for alloc/spill decisions.
  inline uint32_t weight() const noexcept { return _weight; }
  //! Sets the virtual register weight (0 to 255) - the register allocator can
  //! use it as explicit hint for alloc/spill decisions and initial bin-packing.
  inline void setWeight(uint32_t weight) noexcept { _weight = uint8_t(weight); }

  //! Returns whether the virtual register is always allocated to a fixed
  //! physical register (and never reallocated).
  //!
  //! \note This is only used for special purposes and it's mostly internal.
  inline bool isFixed() const noexcept { return bool(_isFixed); }

  //! Returns whether the virtual register is indeed a stack that only uses
  //! the virtual register id for making it accessible.
  //!
  //! \note It's an error if a stack is accessed as a register.
  inline bool isStack() const noexcept { return bool(_isStack); }

  inline bool hasWorkReg() const noexcept { return _workReg != nullptr; }
  inline RAWorkReg* workReg() const noexcept { return _workReg; }
  inline void setWorkReg(RAWorkReg* workReg) noexcept { _workReg = workReg; }
  inline void resetWorkReg() noexcept { _workReg = nullptr; }

  //! \}
};

// ============================================================================
// [asmjit::BaseCompiler]
// ============================================================================

//! Code emitter that uses virtual registers and performs register allocation.
//!
//! Compiler is a high-level code-generation tool that provides register
//! allocation and automatic handling of function calling conventions. It was
//! primarily designed for merging multiple parts of code into a function
//! without worrying about registers and function calling conventions.
//!
//! BaseCompiler can be used, with a minimum effort, to handle 32-bit and
//! 64-bit code generation within a single code base.
//!
//! BaseCompiler is based on BaseBuilder and contains all the features it
//! provides. It means that the code it stores can be modified (removed, added,
//! injected) and analyzed. When the code is finalized the compiler can emit
//! the code into an Assembler to translate the abstract representation into a
//! machine code.
//!
//! Check out architecture specific compilers for more details and examples:
//!
//!   - \ref x86::Compiler - X86/X64 compiler implementation.
class ASMJIT_VIRTAPI BaseCompiler : public BaseBuilder {
public:
  ASMJIT_NONCOPYABLE(BaseCompiler)
  typedef BaseBuilder Base;

  //! Current function.
  FuncNode* _func;
  //! Allocates `VirtReg` objects.
  Zone _vRegZone;
  //! Stores array of `VirtReg` pointers.
  ZoneVector<VirtReg*> _vRegArray;
  //! Stores jump annotations.
  ZoneVector<JumpAnnotation*> _jumpAnnotations;

  //! Local constant pool, flushed at the end of each function.
  ConstPoolNode* _localConstPool;
  //! Global constant pool, flushed by `finalize()`.
  ConstPoolNode* _globalConstPool;

  //! \name Construction & Destruction
  //! \{

  //! Creates a new `BaseCompiler` instance.
  ASMJIT_API BaseCompiler() noexcept;
  //! Destroys the `BaseCompiler` instance.
  ASMJIT_API virtual ~BaseCompiler() noexcept;

  //! \}

  //! \name Function Management
  //! \{

  //! Returns the current function.
  inline FuncNode* func() const noexcept { return _func; }

  //! Creates a new \ref FuncNode.
  ASMJIT_API Error _newFuncNode(FuncNode** out, const FuncSignature& signature);
  //! Creates a new \ref FuncNode adds it to the compiler.
  ASMJIT_API Error _addFuncNode(FuncNode** out, const FuncSignature& signature);

  //! Creates a new \ref FuncRetNode.
  ASMJIT_API Error _newRetNode(FuncRetNode** out, const Operand_& o0, const Operand_& o1);
  //! Creates a new \ref FuncRetNode and adds it to the compiler.
  ASMJIT_API Error _addRetNode(FuncRetNode** out, const Operand_& o0, const Operand_& o1);

  //! Creates a new \ref FuncNode with the given `signature` and returns it.
  inline FuncNode* newFunc(const FuncSignature& signature) {
    FuncNode* node;
    _newFuncNode(&node, signature);
    return node;
  }

  //! Creates a new \ref FuncNode with the given `signature`, adds it to the
  //! compiler by using the \ref addFunc(FuncNode*) overload, and returns it.
  inline FuncNode* addFunc(const FuncSignature& signature) {
    FuncNode* node;
    _addFuncNode(&node, signature);
    return node;
  }

  //! Adds a function `node` to the instruction stream.
  ASMJIT_API FuncNode* addFunc(FuncNode* func);
  //! Emits a sentinel that marks the end of the current function.
  ASMJIT_API Error endFunc();

  //! Sets a function argument at `argIndex` to `reg`.
  ASMJIT_API Error setArg(uint32_t argIndex, const BaseReg& reg);

  inline FuncRetNode* newRet(const Operand_& o0, const Operand_& o1) {
    FuncRetNode* node;
    _newRetNode(&node, o0, o1);
    return node;
  }

  inline FuncRetNode* addRet(const Operand_& o0, const Operand_& o1) {
    FuncRetNode* node;
    _addRetNode(&node, o0, o1);
    return node;
  }

  //! \}

  //! \name Function Invocation
  //! \{

  //! Creates a new \ref InvokeNode.
  ASMJIT_API Error _newInvokeNode(InvokeNode** out, uint32_t instId, const Operand_& o0, const FuncSignature& signature);
  //! Creates a new \ref InvokeNode and adds it to Compiler.
  ASMJIT_API Error _addInvokeNode(InvokeNode** out, uint32_t instId, const Operand_& o0, const FuncSignature& signature);

  //! Creates a new `InvokeNode`.
  inline InvokeNode* newCall(uint32_t instId, const Operand_& o0, const FuncSignature& signature) {
    InvokeNode* node;
    _newInvokeNode(&node, instId, o0, signature);
    return node;
  }

  //! Adds a new `InvokeNode`.
  inline InvokeNode* addCall(uint32_t instId, const Operand_& o0, const FuncSignature& signature) {
    InvokeNode* node;
    _addInvokeNode(&node, instId, o0, signature);
    return node;
  }

  //! \}

  //! \name Virtual Registers
  //! \{

  //! Creates a new virtual register representing the given `typeId` and `signature`.
  //!
  //! \note This function is public, but it's not generally recommended to be used
  //! by AsmJit users, use architecture-specific `newReg()` functionality instead
  //! or functions like \ref _newReg() and \ref _newRegFmt().
  ASMJIT_API Error newVirtReg(VirtReg** out, uint32_t typeId, uint32_t signature, const char* name);

  //! Creates a new virtual register of the given `typeId` and stores it to `out` operand.
  ASMJIT_API Error _newReg(BaseReg* out, uint32_t typeId, const char* name = nullptr);

  //! Creates a new virtual register of the given `typeId` and stores it to `out` operand.
  //!
  //! \note This version accepts a snprintf() format `fmt` followed by a variadic arguments.
  ASMJIT_API Error _newRegFmt(BaseReg* out, uint32_t typeId, const char* fmt, ...);

  //! Creates a new virtual register compatible with the provided reference register `ref`.
  ASMJIT_API Error _newReg(BaseReg* out, const BaseReg& ref, const char* name = nullptr);

  //! Creates a new virtual register compatible with the provided reference register `ref`.
  //!
  //! \note This version accepts a snprintf() format `fmt` followed by a variadic arguments.
  ASMJIT_API Error _newRegFmt(BaseReg* out, const BaseReg& ref, const char* fmt, ...);

  //! Tests whether the given `id` is a valid virtual register id.
  inline bool isVirtIdValid(uint32_t id) const noexcept {
    uint32_t index = Operand::virtIdToIndex(id);
    return index < _vRegArray.size();
  }
  //! Tests whether the given `reg` is a virtual register having a valid id.
  inline bool isVirtRegValid(const BaseReg& reg) const noexcept {
    return isVirtIdValid(reg.id());
  }

  //! Returns \ref VirtReg associated with the given `id`.
  inline VirtReg* virtRegById(uint32_t id) const noexcept {
    ASMJIT_ASSERT(isVirtIdValid(id));
    return _vRegArray[Operand::virtIdToIndex(id)];
  }

  //! Returns \ref VirtReg associated with the given `reg`.
  inline VirtReg* virtRegByReg(const BaseReg& reg) const noexcept { return virtRegById(reg.id()); }

  //! Returns \ref VirtReg associated with the given virtual register `index`.
  //!
  //! \note This is not the same as virtual register id. The conversion between
  //! id and its index is implemented by \ref Operand_::virtIdToIndex() and \ref
  //! Operand_::indexToVirtId() functions.
  inline VirtReg* virtRegByIndex(uint32_t index) const noexcept { return _vRegArray[index]; }

  //! Returns an array of all virtual registers managed by the Compiler.
  inline const ZoneVector<VirtReg*>& virtRegs() const noexcept { return _vRegArray; }

  //! \name Stack
  //! \{

  //! Creates a new stack of the given `size` and `alignment` and stores it to `out`.
  //!
  //! \note `name` can be used to give the stack a name, for debugging purposes.
  ASMJIT_API Error _newStack(BaseMem* out, uint32_t size, uint32_t alignment, const char* name = nullptr);

  //! Updates the stack size of a stack created by `_newStack()` by its `virtId`.
  ASMJIT_API Error setStackSize(uint32_t virtId, uint32_t newSize, uint32_t newAlignment = 0);

  //! Updates the stack size of a stack created by `_newStack()`.
  inline Error setStackSize(const BaseMem& mem, uint32_t newSize, uint32_t newAlignment = 0) {
    return setStackSize(mem.id(), newSize, newAlignment);
  }

  //! \}

  //! \name Constants
  //! \{

  //! Creates a new constant of the given `scope` (see \ref ConstPool::Scope).
  //!
  //! This function adds a constant of the given `size` to the built-in \ref
  //! ConstPool and stores the reference to that constant to the `out` operand.
  ASMJIT_API Error _newConst(BaseMem* out, uint32_t scope, const void* data, size_t size);

  //! \}

  //! \name Miscellaneous
  //! \{

  //! Rename the given virtual register `reg` to a formatted string `fmt`.
  ASMJIT_API void rename(const BaseReg& reg, const char* fmt, ...);

  //! \}

  //! \name Jump Annotations
  //! \{

  inline const ZoneVector<JumpAnnotation*>& jumpAnnotations() const noexcept {
    return _jumpAnnotations;
  }

  ASMJIT_API Error newJumpNode(JumpNode** out, uint32_t instId, uint32_t instOptions, const Operand_& o0, JumpAnnotation* annotation);
  ASMJIT_API Error emitAnnotatedJump(uint32_t instId, const Operand_& o0, JumpAnnotation* annotation);

  //! Returns a new `JumpAnnotation` instance, which can be used to aggregate
  //! possible targets of a jump where the target is not a label, for example
  //! to implement jump tables.
  ASMJIT_API JumpAnnotation* newJumpAnnotation();

  //! \}

#ifndef ASMJIT_NO_DEPRECATED
  ASMJIT_DEPRECATED("alloc() has no effect, it will be removed in the future")
  inline void alloc(BaseReg&) {}
  ASMJIT_DEPRECATED("spill() has no effect, it will be removed in the future")
  inline void spill(BaseReg&) {}
#endif // !ASMJIT_NO_DEPRECATED

  //! \name Events
  //! \{

  ASMJIT_API Error onAttach(CodeHolder* code) noexcept override;
  ASMJIT_API Error onDetach(CodeHolder* code) noexcept override;

  //! \}
};

// ============================================================================
// [asmjit::JumpAnnotation]
// ============================================================================

//! Jump annotation used to annotate jumps.
//!
//! \ref BaseCompiler allows to emit jumps where the target is either register
//! or memory operand. Such jumps cannot be trivially inspected, so instead of
//! doing heuristics AsmJit allows to annotate such jumps with possible targets.
//! Register allocator then use the annotation to construct control-flow, which
//! is then used by liveness analysis and other tools to prepare ground for
//! register allocation.
class JumpAnnotation {
public:
  ASMJIT_NONCOPYABLE(JumpAnnotation)

  //! Compiler that owns this JumpAnnotation.
  BaseCompiler* _compiler;
  //! Annotation identifier.
  uint32_t _annotationId;
  //! Vector of label identifiers, see \ref labelIds().
  ZoneVector<uint32_t> _labelIds;

  inline JumpAnnotation(BaseCompiler* compiler, uint32_t annotationId) noexcept
    : _compiler(compiler),
      _annotationId(annotationId) {}

  //! Returns the compiler that owns this JumpAnnotation.
  inline BaseCompiler* compiler() const noexcept { return _compiler; }
  //! Returns the annotation id.
  inline uint32_t annotationId() const noexcept { return _annotationId; }
  //! Returns a vector of label identifiers that lists all targets of the jump.
  const ZoneVector<uint32_t>& labelIds() const noexcept { return _labelIds; }

  //! Tests whether the given `label` is a target of this JumpAnnotation.
  inline bool hasLabel(const Label& label) const noexcept { return hasLabelId(label.id()); }
  //! Tests whether the given `labelId` is a target of this JumpAnnotation.
  inline bool hasLabelId(uint32_t labelId) const noexcept { return _labelIds.contains(labelId); }

  //! Adds the `label` to the list of targets of this JumpAnnotation.
  inline Error addLabel(const Label& label) noexcept { return addLabelId(label.id()); }
  //! Adds the `labelId` to the list of targets of this JumpAnnotation.
  inline Error addLabelId(uint32_t labelId) noexcept { return _labelIds.append(&_compiler->_allocator, labelId); }
};

// ============================================================================
// [asmjit::JumpNode]
// ============================================================================

//! Jump instruction with \ref JumpAnnotation.
//!
//! \note This node should be only used to represent jump where the jump target
//! cannot be deduced by examining instruction operands. For example if the jump
//! target is register or memory location. This pattern is often used to perform
//! indirect jumps that use jump table, e.g. to implement `switch{}` statement.
class JumpNode : public InstNode {
public:
  ASMJIT_NONCOPYABLE(JumpNode)

  JumpAnnotation* _annotation;

  //! \name Construction & Destruction
  //! \{

  ASMJIT_INLINE JumpNode(BaseCompiler* cc, uint32_t instId, uint32_t options, uint32_t opCount, JumpAnnotation* annotation) noexcept
    : InstNode(cc, instId, options, opCount, kBaseOpCapacity),
      _annotation(annotation) {
    setType(kNodeJump);
  }

  //! \}

  //! \name Accessors
  //! \{

  //! Tests whether this JumpNode has associated a \ref JumpAnnotation.
  inline bool hasAnnotation() const noexcept { return _annotation != nullptr; }
  //! Returns the \ref JumpAnnotation associated with this jump, or `nullptr`.
  inline JumpAnnotation* annotation() const noexcept { return _annotation; }
  //! Sets the \ref JumpAnnotation associated with this jump to `annotation`.
  inline void setAnnotation(JumpAnnotation* annotation) noexcept { _annotation = annotation; }

  //! \}
};

// ============================================================================
// [asmjit::FuncNode]
// ============================================================================

//! Function node represents a function used by \ref BaseCompiler.
//!
//! A function is composed of the following:
//!
//!   - Function entry, \ref FuncNode acts as a label, so the entry is implicit.
//!     To get the entry, simply use \ref FuncNode::label(), which is the same
//!     as \ref LabelNode::label().
//!
//!   - Function exit, which is represented by \ref FuncNode::exitNode(). A
//!     helper function \ref FuncNode::exitLabel() exists and returns an exit
//!     label instead of node.
//!
//!   - Function \ref FuncNode::endNode() sentinel. This node marks the end of
//!     a function - there should be no code that belongs to the function after
//!     this node, but the Compiler doesn't enforce that at the moment.
//!
//!   - Function detail, see \ref FuncNode::detail().
//!
//!   - Function frame, see \ref FuncNode::frame().
//!
//!   - Function arguments mapped to virtual registers, see \ref FuncNode::args().
//!
//! In a node list, the function and its body looks like the following:
//!
//! \code{.unparsed}
//! [...]       - Anything before the function.
//!
//! [FuncNode]  - Entry point of the function, acts as a label as well.
//!   <Prolog>  - Prolog inserted by the register allocator.
//!   {...}     - Function body - user code basically.
//! [ExitLabel] - Exit label
//!   <Epilog>  - Epilog inserted by the register allocator.
//!   <Return>  - Return inserted by the register allocator.
//!   {...}     - Can contain data or user code (error handling, special cases, ...).
//! [FuncEnd]   - End sentinel
//!
//! [...]       - Anything after the function.
//! \endcode
//!
//! When a function is added to the compiler by \ref BaseCompiler::addFunc() it
//! actually inserts 3 nodes (FuncNode, ExitLabel, and FuncEnd) and sets the
//! current cursor to be FuncNode. When \ref BaseCompiler::endFunc() is called
//! the cursor is set to FuncEnd. This guarantees that user can use ExitLabel
//! as a marker after additional code or data can be placed, and it's a common
//! practice.
class FuncNode : public LabelNode {
public:
  ASMJIT_NONCOPYABLE(FuncNode)

  //! Function detail.
  FuncDetail _funcDetail;
  //! Function frame.
  FuncFrame _frame;
  //! Function exit label.
  LabelNode* _exitNode;
  //! Function end (sentinel).
  SentinelNode* _end;
  //! Arguments array as `VirtReg`.
  VirtReg** _args;

  //! \name Construction & Destruction
  //! \{

  //! Creates a new `FuncNode` instance.
  //!
  //! Always use `BaseCompiler::addFunc()` to create `FuncNode`.
  ASMJIT_INLINE FuncNode(BaseBuilder* cb) noexcept
    : LabelNode(cb),
      _funcDetail(),
      _frame(),
      _exitNode(nullptr),
      _end(nullptr),
      _args(nullptr) {
    setType(kNodeFunc);
  }

  //! \}

  //! \{
  //! \name Accessors

  //! Returns function exit `LabelNode`.
  inline LabelNode* exitNode() const noexcept { return _exitNode; }
  //! Returns function exit label.
  inline Label exitLabel() const noexcept { return _exitNode->label(); }

  //! Returns "End of Func" sentinel.
  inline SentinelNode* endNode() const noexcept { return _end; }

  //! Returns function declaration.
  inline FuncDetail& detail() noexcept { return _funcDetail; }
  //! Returns function declaration.
  inline const FuncDetail& detail() const noexcept { return _funcDetail; }

  //! Returns function frame.
  inline FuncFrame& frame() noexcept { return _frame; }
  //! Returns function frame.
  inline const FuncFrame& frame() const noexcept { return _frame; }

  //! Returns arguments count.
  inline uint32_t argCount() const noexcept { return _funcDetail.argCount(); }
  //! Returns returns count.
  inline uint32_t retCount() const noexcept { return _funcDetail.retCount(); }

  //! Returns arguments list.
  inline VirtReg** args() const noexcept { return _args; }

  //! Returns argument at `i`.
  inline VirtReg* arg(uint32_t i) const noexcept {
    ASMJIT_ASSERT(i < argCount());
    return _args[i];
  }

  //! Sets argument at `i`.
  inline void setArg(uint32_t i, VirtReg* vReg) noexcept {
    ASMJIT_ASSERT(i < argCount());
    _args[i] = vReg;
  }

  //! Resets argument at `i`.
  inline void resetArg(uint32_t i) noexcept {
    ASMJIT_ASSERT(i < argCount());
    _args[i] = nullptr;
  }

  //! Returns function attributes.
  inline uint32_t attributes() const noexcept { return _frame.attributes(); }
  //! Adds `attrs` to the function attributes.
  inline void addAttributes(uint32_t attrs) noexcept { _frame.addAttributes(attrs); }

  //! \}
};

// ============================================================================
// [asmjit::FuncRetNode]
// ============================================================================

//! Function return, used by \ref BaseCompiler.
class FuncRetNode : public InstNode {
public:
  ASMJIT_NONCOPYABLE(FuncRetNode)

  //! \name Construction & Destruction
  //! \{

  //! Creates a new `FuncRetNode` instance.
  inline FuncRetNode(BaseBuilder* cb) noexcept : InstNode(cb, BaseInst::kIdAbstract, 0, 0) {
    _any._nodeType = kNodeFuncRet;
  }

  //! \}
};

// ============================================================================
// [asmjit::InvokeNode]
// ============================================================================

//! Function invocation, used by \ref BaseCompiler.
class InvokeNode : public InstNode {
public:
  ASMJIT_NONCOPYABLE(InvokeNode)

  //! Function detail.
  FuncDetail _funcDetail;
  //! Returns.
  Operand_ _rets[2];
  //! Arguments.
  Operand_* _args;

  //! \name Construction & Destruction
  //! \{

  //! Creates a new `InvokeNode` instance.
  inline InvokeNode(BaseBuilder* cb, uint32_t instId, uint32_t options) noexcept
    : InstNode(cb, instId, options, kBaseOpCapacity),
      _funcDetail(),
      _args(nullptr) {
    setType(kNodeInvoke);
    _resetOps();
    _rets[0].reset();
    _rets[1].reset();
    addFlags(kFlagIsRemovable);
  }

  //! \}

  //! \name Accessors
  //! \{

  //! Sets the function signature.
  inline Error init(const FuncSignature& signature, const Environment& environment) noexcept {
    return _funcDetail.init(signature, environment);
  }

  //! Returns the function detail.
  inline FuncDetail& detail() noexcept { return _funcDetail; }
  //! Returns the function detail.
  inline const FuncDetail& detail() const noexcept { return _funcDetail; }

  //! Returns the target operand.
  inline Operand& target() noexcept { return _opArray[0].as<Operand>(); }
  //! \overload
  inline const Operand& target() const noexcept { return _opArray[0].as<Operand>(); }

  //! Returns the number of function arguments.
  inline uint32_t argCount() const noexcept { return _funcDetail.argCount(); }
  //! Returns the number of function return values.
  inline uint32_t retCount() const noexcept { return _funcDetail.retCount(); }

  //! Returns the return value at `i`.
  inline Operand& ret(uint32_t i = 0) noexcept {
    ASMJIT_ASSERT(i < 2);
    return _rets[i].as<Operand>();
  }
  //! \overload
  inline const Operand& ret(uint32_t i = 0) const noexcept {
    ASMJIT_ASSERT(i < 2);
    return _rets[i].as<Operand>();
  }

  //! Returns the function argument at `i`.
  inline Operand& arg(uint32_t i) noexcept {
    ASMJIT_ASSERT(i < kFuncArgCountLoHi);
    return _args[i].as<Operand>();
  }
  //! \overload
  inline const Operand& arg(uint32_t i) const noexcept {
    ASMJIT_ASSERT(i < kFuncArgCountLoHi);
    return _args[i].as<Operand>();
  }

  //! Sets the function argument at `i` to `op`.
  ASMJIT_API bool _setArg(uint32_t i, const Operand_& op) noexcept;
  //! Sets the function return value at `i` to `op`.
  ASMJIT_API bool _setRet(uint32_t i, const Operand_& op) noexcept;

  //! Sets the function argument at `i` to `reg`.
  inline bool setArg(uint32_t i, const BaseReg& reg) noexcept { return _setArg(i, reg); }
  //! Sets the function argument at `i` to `imm`.
  inline bool setArg(uint32_t i, const Imm& imm) noexcept { return _setArg(i, imm); }

  //! Sets the function return value at `i` to `var`.
  inline bool setRet(uint32_t i, const BaseReg& reg) noexcept { return _setRet(i, reg); }

  //! \}
};

// ============================================================================
// [asmjit::FuncPass]
// ============================================================================

//! Function pass extends \ref Pass with \ref FuncPass::runOnFunction().
class ASMJIT_VIRTAPI FuncPass : public Pass {
public:
  ASMJIT_NONCOPYABLE(FuncPass)
  typedef Pass Base;

  //! \name Construction & Destruction
  //! \{

  ASMJIT_API FuncPass(const char* name) noexcept;

  //! \}

  //! \name Accessors
  //! \{

  //! Returns the associated `BaseCompiler`.
  inline BaseCompiler* cc() const noexcept { return static_cast<BaseCompiler*>(_cb); }

  //! \}

  //! \name Run
  //! \{

  //! Calls `runOnFunction()` on each `FuncNode` node found.
  ASMJIT_API Error run(Zone* zone, Logger* logger) override;

  //! Called once per `FuncNode`.
  virtual Error runOnFunction(Zone* zone, Logger* logger, FuncNode* func) = 0;

  //! \}
};

//! \}

ASMJIT_END_NAMESPACE

#endif // !ASMJIT_NO_COMPILER
#endif // ASMJIT_CORE_COMPILER_H_INCLUDED
