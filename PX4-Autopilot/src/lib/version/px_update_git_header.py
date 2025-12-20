#!/usr/bin/env python

import argparse
import os
import sys

# Argument parsing (necesario porque el sistema de compilación lo llama con argumentos)
parser = argparse.ArgumentParser(description="""Generador de encabezado de versión NO-GIT (Parcheado).""")
parser.add_argument('filename', metavar='version.h', help='Header output file')
parser.add_argument('--git_tag', help='git tag string')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='Verbose output', default=False)
parser.add_argument('--validate', dest='validate', action='store_true', help='Validate the tag format', default=False)

args = parser.parse_args()
filename = args.filename
verbose = args.verbose

# Leemos el archivo anterior si existe para no sobreescribir si no es necesario
try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''



CUSTOM_TAG = "v1.14.0-dev"          # La versión que verá el usuario
CUSTOM_HASH = "0000000000000000000000000000000000000000" # Hash falso (40 ceros)
CUSTOM_BRANCH = "main"              # Rama por defecto


header = """
/* Auto Magically Generated file (NO-GIT VERSION) */
/* Do not edit! */
#pragma once
"""

git_tag = CUSTOM_TAG
git_version = CUSTOM_HASH
git_version_short = git_version[0:16]
git_branch_name = CUSTOM_BRANCH
oem_tag = ''
tag_or_branch = CUSTOM_BRANCH

header += f"""
#define PX4_GIT_VERSION_STR "{git_version}"
#define PX4_GIT_VERSION_BINARY 0x{git_version_short}
#define PX4_GIT_TAG_STR "{git_tag}"
#define PX4_GIT_BRANCH_NAME "{git_branch_name}"

#define PX4_GIT_OEM_VERSION_STR  "{oem_tag}"

#define PX4_GIT_TAG_OR_BRANCH_NAME "{tag_or_branch}"
"""

# Mavlink Version Fake
mavlink_git_version = CUSTOM_HASH
mavlink_git_version_short = mavlink_git_version[0:16]

header += f"""
#define MAVLINK_LIB_GIT_VERSION_STR  "{mavlink_git_version}"
#define MAVLINK_LIB_GIT_VERSION_BINARY 0x{mavlink_git_version_short}
"""

# NuttX Version Fake
nuttx_git_tag = "v0.0.0"
nuttx_git_version = CUSTOM_HASH
nuttx_git_version_short = nuttx_git_version[0:16]

header += f"""
#define NUTTX_GIT_VERSION_STR  "{nuttx_git_version}"
#define NUTTX_GIT_VERSION_BINARY 0x{nuttx_git_version_short}
#define NUTTX_GIT_TAG_STR  "{nuttx_git_tag}"
"""

if old_header != header:
    if verbose:
        print('Updating header {}'.format(filename))
    with open(filename, 'w') as fp_header:
        fp_header.write(header)