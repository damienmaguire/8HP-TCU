##
## Makefile for the 8HP TCU firmware (STM32F107VCT6).
## Based on the openinverter Stm32-vcu (ZombieVerter) Makefile.
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

# Board: HW_REV=4 Stm32_TCU_V4 (default), HW_REV=5 Stm32_TCU_V4 rev B
# (`make HW_REV=5`, image stm32_tcu_revb, objects in obj_revb/),
# HW_REV=6 rev C, provisional (`make HW_REV=6`, stm32_tcu_revc, obj_revc/).
HW_REV          ?= 4
CPUFLAGS        = -mcpu=cortex-m3 -mthumb
ARCHDEF         = -DSTM32F1 -DHW_REV=$(HW_REV)
ifeq ($(HW_REV),4)
BINARY          = stm32_tcu
OUT_DIR         = obj
MAPFILE         = linker.map
# V4: slow analogue inputs through libopeninv AnaIn (ADC1 + DMA)
ANAIN_OBJ       = anain.o
else ifeq ($(HW_REV),5)
BINARY          = stm32_tcu_revb
OUT_DIR         = obj_revb
MAPFILE         = linker_revb.map
# Rev B: ADC1 + DMA run the speed scan, AnaIn is not linked (slowadc.cpp)
ANAIN_OBJ       =
else ifeq ($(HW_REV),6)
BINARY          = stm32_tcu_revc
OUT_DIR         = obj_revc
MAPFILE         = linker_revc.map
# Rev C: as rev B
ANAIN_OBJ       =
else
$(error HW_REV must be 4 (V4), 5 (rev B) or 6 (rev C))
endif
LDSCRIPT        = stm32_tcu.ld
OPENCM3_LIB     = opencm3_stm32f1
OPENCM3_TARGETS = stm32/f1

PREFIX	  ?= arm-none-eabi
SIZE      = $(PREFIX)-size
CC        = $(PREFIX)-gcc
CPP       = $(PREFIX)-g++
LD        = $(PREFIX)-gcc
OBJCOPY   = $(PREFIX)-objcopy
MKDIR_P   = mkdir -p
CFLAGS    = -Os -Wall -Wextra -Ilibopeninv/include -Iinclude/ -Ilibopencm3/include \
            -fno-common -fno-builtin -pedantic $(ARCHDEF) -DMAX_USER_MESSAGES=10 \
            $(CPUFLAGS) -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
CPPFLAGS  = -Os -Wall -Wextra -Ilibopeninv/include -Iinclude/ -Ilibopencm3/include \
            -fno-common -std=c++17 -pedantic $(ARCHDEF) -DMAX_USER_MESSAGES=10 \
            -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions \
            -fno-unwind-tables $(CPUFLAGS) -ggdb3
LDFLAGS   = -Llibopencm3/lib -T$(LDSCRIPT) -march=armv7 -nostartfiles \
            -Wl,--gc-sections,-Map,$(MAPFILE)

# Hardware-free control code (also built by the host tests)
CORE_OBJS = caltypes.o cal_330e.o cal_f11.o cal_v095.o calset.o canif.o drivemode.o gears.o interlock.o max22200.o eds.o sas.o \
            engage.o speedsens.o tcu.o valveout.o anaconv.o linpump.o \
            shifter.o gwslever.o canlever.o torquein.o adapt.o diag.o tcc.o statstream.o
# Firmware glue and libopeninv
OBJSL     = stm32_tcu.o hwinit.o slowadc.o terminal_prj.o $(CORE_OBJS) \
            stm32scheduler.o params.o terminal.o my_string.o digio.o my_fp.o \
            printf.o $(ANAIN_OBJ) canhardware.o canmap.o cansdo.o sdocommands.o \
            param_save.o errormessage.o stm32_can.o terminalcommands.o

OBJS      = $(patsubst %.o,$(OUT_DIR)/%.o, $(OBJSL))

# libopeninv's cansdo.cpp hands a packed SDO frame to the CAN driver as a
# uint32_t pointer. The frames it passes sit in word-aligned CAN buffers,
# so this is harmless (the ZombieVerter builds with the same warning). It
# is upstream code, so the warning is turned off for this object only.
$(OUT_DIR)/cansdo.o: CPPFLAGS += -Wno-address-of-packed-member
vpath %.c src/ libopeninv/src/
vpath %.cpp src/ libopeninv/src/

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

# Suppress the rwx segment warning on binutils 2.39 or later.
comma := ,
try-run = $(shell set -e; if ($(1)) >/dev/null 2>&1; then echo "$(2)"; else echo "$(3)"; fi)
ld-option = $(call try-run, $(PREFIX)-ld $(1) -v,-Wl$(comma)$(1))
LDFLAGS += $(call ld-option,--no-warn-rwx-segments)

all: directories images
Debug: images
Release: images
cleanDebug: clean
images: get-deps $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	@printf "  OBJCOPY $(BINARY).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(BINARY) $(BINARY).hex
	$(Q)$(SIZE) $(BINARY)

directories: $(OUT_DIR)

$(OUT_DIR):
	$(Q)$(MKDIR_P) $(OUT_DIR)

$(BINARY): $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -l$(OPENCM3_LIB) -lm

$(OUT_DIR)/%.o: %.c Makefile | $(OUT_DIR)
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile | $(OUT_DIR)
	@printf "  CPP     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CPP) $(CPPFLAGS) -MMD -MP -o $@ -c $<

DEP = $(OBJS:%.o=%.d)
-include $(DEP)

# Regenerate the calibration tables from Ian's exports (not run by 'all';
# the exports are not in the repo):
#   make cal Q030_JSON=/path/330e_Q030_cal_full.json \
#            F11_JSON=/path/f11_sas_eds_cal.json F11_PSYS_JSON=/path/f11_psys_gearin_cal.json \
#            F11_PORT_JSON=/path/f11_a81v058_cal_ported.json \
#            F11_SHIFT_JSON=/path/f11_shift_pressure_cal.json \
#            F11_GEARIN_JSON=/path/f11_gearin_timing_cal.json V095_DIR=/path/cal_v095 [CAL_SET=el]
CAL_SET ?= normal
cal:
	@test -n "$(Q030_JSON)" -a -n "$(F11_JSON)" -a -n "$(F11_PSYS_JSON)" -a -n "$(F11_PORT_JSON)" \
	  -a -n "$(F11_SHIFT_JSON)" -a -n "$(F11_GEARIN_JSON)" -a -n "$(V095_DIR)" || \
	  (echo "set Q030_JSON=... F11_JSON=... F11_PSYS_JSON=... F11_PORT_JSON=... F11_SHIFT_JSON=... F11_GEARIN_JSON=... V095_DIR=..."; exit 1)
	python3 tools/gen_cal.py --q030 $(Q030_JSON) --f11 $(F11_JSON) --f11-psys $(F11_PSYS_JSON) \
	  --f11-port $(F11_PORT_JSON) --f11-shift $(F11_SHIFT_JSON) --f11-gearin $(F11_GEARIN_JSON) \
	  --v095 $(V095_DIR) --set $(CAL_SET)

clean:
	@printf "  CLEAN   $(OUT_DIR)\n"
	$(Q)rm -rf $(OUT_DIR)
	@printf "  CLEAN   firmware images\n"
	$(Q)rm -f $(BINARY) $(BINARY).bin $(BINARY).hex $(MAPFILE)

.PHONY: directories get-deps images clean cal Test cleanTest

get-deps:
ifneq ($(shell test -s libopencm3/lib/lib$(OPENCM3_LIB).a && echo -n yes),yes)
	@printf "  GIT SUBMODULE\n"
	$(Q)git submodule update --init
	@printf "  MAKE libopencm3 $(OPENCM3_TARGETS)\n"
	$(Q)$(MAKE) -C libopencm3 TARGETS=$(OPENCM3_TARGETS)
endif

Test:
	cd test && $(MAKE)
cleanTest:
	cd test && $(MAKE) clean
