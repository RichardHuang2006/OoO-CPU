#pragma once
#include "types.h"

// opcode -> funct3/funct7 decode across R/I/S/B/U/J with sign-extended imms.
DecodedInst decode(uint32_t raw, uint32_t pc);
