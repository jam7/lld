//===- Simple.cpp ---------------------------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Symbols.h"
#include "SyntheticSections.h"
#include "Target.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::support::endian;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

namespace {
class Simple final : public TargetInfo {
public:
  Simple();
  RelExpr getRelExpr(RelType Type, const Symbol &S,
                     const uint8_t *Loc) const override;
  void writePlt(uint8_t *Buf, uint64_t GotEntryAddr, uint64_t PltEntryAddr,
                int32_t Index, unsigned RelOff) const override;
  void relocateOne(uint8_t *Loc, RelType Type, uint64_t Val) const override;
};
} // namespace

Simple::Simple() {
  CopyRel = R_SIMPLE_COPY;
  GotRel = R_SIMPLE_GLOB_DAT;
  PltRel = R_SIMPLE_JMP_SLOT;
  RelativeRel = R_SIMPLE_RELATIVE;
  GotEntrySize = 8;
  PltEntrySize = 32;
  PltHeaderSize = 4 * PltEntrySize;

  PageSize = 8192;
  DefaultMaxPageSize = 0x100000;
  DefaultImageBase = 0x100000;
}

RelExpr Simple::getRelExpr(RelType Type, const Symbol &S,
                            const uint8_t *Loc) const {
  switch (Type) {
  case R_SIMPLE_32:
  case R_SIMPLE_UA32:
  case R_SIMPLE_64:
  case R_SIMPLE_UA64:
  case R_SIMPLE_HI22:
  case R_SIMPLE_LO10:
    return R_ABS;
  case R_SIMPLE_PC10:
  case R_SIMPLE_PC22:
  case R_SIMPLE_DISP32:
  case R_SIMPLE_WDISP30:
    return R_PC;
  case R_SIMPLE_GOT10:
    return R_GOT_OFF;
  case R_SIMPLE_GOT22:
    return R_GOT_OFF;
  case R_SIMPLE_WPLT30:
    return R_PLT_PC;
  case R_SIMPLE_NONE:
    return R_NONE;
  default:
    return R_INVALID;
  }
}

void Simple::relocateOne(uint8_t *Loc, RelType Type, uint64_t Val) const {
  switch (Type) {
  case R_SIMPLE_32:
  case R_SIMPLE_UA32:
    // V-word32
    checkUInt<32>(Loc, Val, Type);
    write32be(Loc, Val);
    break;
  case R_SIMPLE_DISP32:
    // V-disp32
    checkInt<32>(Loc, Val, Type);
    write32be(Loc, Val);
    break;
  case R_SIMPLE_WDISP30:
  case R_SIMPLE_WPLT30:
    // V-disp30
    checkInt<32>(Loc, Val, Type);
    write32be(Loc, (read32be(Loc) & ~0x3fffffff) | ((Val >> 2) & 0x3fffffff));
    break;
  case R_SIMPLE_22:
    // V-imm22
    checkUInt<22>(Loc, Val, Type);
    write32be(Loc, (read32be(Loc) & ~0x003fffff) | (Val & 0x003fffff));
    break;
  case R_SIMPLE_GOT22:
  case R_SIMPLE_PC22:
  case R_SIMPLE_HI22:
    // T-imm22
    write32be(Loc, (read32be(Loc) & ~0x003fffff) | ((Val >> 10) & 0x003fffff));
    break;
  case R_SIMPLE_WDISP19:
    // V-disp19
    checkInt<21>(Loc, Val, Type);
    write32be(Loc, (read32be(Loc) & ~0x0007ffff) | ((Val >> 2) & 0x0007ffff));
    break;
  case R_SIMPLE_GOT10:
  case R_SIMPLE_PC10:
  case R_SIMPLE_LO10:
    // T-simm10
    write32be(Loc, (read32be(Loc) & ~0x000003ff) | (Val & 0x000003ff));
    break;
  case R_SIMPLE_64:
  case R_SIMPLE_UA64:
  case R_SIMPLE_GLOB_DAT:
    // V-xword64
    write64be(Loc, Val);
    break;
  default:
    error(getErrorLocation(Loc) + "unrecognized reloc " + Twine(Type));
  }
}

void Simple::writePlt(uint8_t *Buf, uint64_t GotEntryAddr,
                       uint64_t PltEntryAddr, int32_t Index,
                       unsigned RelOff) const {
  const uint8_t PltData[] = {
      0x03, 0x00, 0x00, 0x00, // sethi   (. - .PLT0), %g1
      0x30, 0x68, 0x00, 0x00, // ba,a    %xcc, .PLT1
      0x01, 0x00, 0x00, 0x00, // nop
      0x01, 0x00, 0x00, 0x00, // nop
      0x01, 0x00, 0x00, 0x00, // nop
      0x01, 0x00, 0x00, 0x00, // nop
      0x01, 0x00, 0x00, 0x00, // nop
      0x01, 0x00, 0x00, 0x00  // nop
  };
  memcpy(Buf, PltData, sizeof(PltData));

  uint64_t Off = PltHeaderSize + Index * PltEntrySize;
  relocateOne(Buf, R_SIMPLE_22, Off);
  relocateOne(Buf + 4, R_SIMPLE_WDISP19, -(Off + 4 - PltEntrySize));
}

TargetInfo *elf::getSimpleTargetInfo() {
  static Simple Target;
  return &Target;
}
