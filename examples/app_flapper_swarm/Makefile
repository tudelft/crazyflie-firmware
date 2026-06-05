# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../..

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

#
# Prompt for the drone role on every build (e.g. after 'make clean').
# This replaces the default 'all' goal so a plain 'make' asks first.
#
.DEFAULT_GOAL := build

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

.PHONY: build

# Interactive role selection. Loads a curated defconfig so the deck set is
# deterministic instead of relying on the long Kconfig question list.
#   anchor        -> configs/anchor_defconfig       (Loco/UWB only)
#   flapper       -> configs/flapper_defconfig      (Loco + MTF02 + 4x4 ToF)
#   flapper_notof -> configs/flapper_notof_defconfig (Loco + MTF02, no 4x4)
build:
	@printf '\n=== Which drone are you building for? ===\n'; \
	printf '  [a] anchor        - Loco/UWB only (no 4x4)\n'; \
	printf '  [f] flapper       - Loco + MTF02 + 4x4 ToF\n'; \
	printf '  [n] flapper_notof - Loco + MTF02, no 4x4 ToF\n'; \
	printf 'Choice [a/f/n]: '; \
	read role; \
	case "$$role" in \
	  a|A|anchor)        preset=anchor;;        \
	  f|F|flapper)       preset=flapper;;       \
	  n|N|notof)         preset=flapper_notof;; \
	  *) echo "Unknown choice '$$role' - run 'make' again and type a, f, or n."; exit 1;; \
	esac; \
	echo ">>> Loading $${preset}_defconfig ..."; \
	$(MAKE) $${preset}_defconfig && \
	echo ">>> Building $$preset firmware ..." && \
	$(MAKE) all
