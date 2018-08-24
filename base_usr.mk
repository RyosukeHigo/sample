# =============================================================================
#  Make include file base_usr.mk:
#
#   RTI1005 5.2.5 (18-Mar-2005)
#   Wed Feb 15 14:18:53 2006
#
#   Copyright (c) 1999-2002 dSPACE GmbH, GERMANY
# =============================================================================

# =============================================================================
# ===== Define file version macro. Never change this value. ===================
# =============================================================================
USER_MAKEFILE_VERSION = 2
# =============================================================================

# -----------------------------------------------------------------------------
# Macros for user-specific settings.
#
# All macros below may list several items (files or directories). You can
# separate multiple items with blanks or list them on several lines using
# the \ (backslash) as line continuation character.
#
# The directory names may include absolute or partial path
# descriptions, e.g., .\project1\file3.c
#
# Blanks in directory or file names are not allowed.
#
# Examples:
#
# USER_SRCS = file1.c file2.c file3.c
#
# USER_SRCS = \
#   file1.c \
#   file2.c \
#   file3.c
#
# -----------------------------------------------------------------------------

# Directories where S-Function C source files are stored.
SFCN_DIR = "..\..\system" "..\..\hand" "..\..\wam" "..\..\avs" "..\..\force" "..\..\basis" "..\..\xyzstage"

# Additional C source files to be compiled (file name extension .c).
USER_SRCS = SystemFunc.c HandFunc.c WAMFunc.c AVSFunc.c \
			HandApp.c WAMApp.c WAM2App.c AVSApp.c  XYZstageApp.c\
			LinkHomoMat.c vec3DRot.c vec3DInProduct.c pmatrix.c homoMat.c myMatrix.c Vector.c Kalman.c \
			Predict3D.c throwJnt2armJnt.c storeData.c

# Additional assembler source files to be compiled (file name extension .asm).
USER_ASM_SRCS =

# Directories where additional C and assembler source files are stored.
USER_SRCS_DIR =

# Path names for user include files.
USER_INCLUDES_PATH = "..\..\system" "..\..\hand" "..\..\wam" "..\..\avs" "..\..\force" "..\..\xyzstage"

# Additional user object files to be linked.
USER_OBJS =

# Additional user libraries to be linked.
#USER_LIBS = ..\..\basis\basis.lib		// なぜかライブラリを指定してもUSER_SRCSで指定しないとダメ

# EOF -------------------------------------------------------------------------
