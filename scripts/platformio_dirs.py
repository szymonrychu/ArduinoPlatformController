#!/usr/bin/env python3
import os

Import("env")

_split_result = env["PIOENV"].split('-')
subdir = _split_result[0]

# access to global build environment
# print(env.Dump())
# exit()

# access to project build environment (is used source files in "src" folder)
# print(projenv)
project_dir = os.path.join(env['PROJECT_DIR'], 'platformio', subdir)

env['PROJECT_SRC_DIR'] = os.path.join(project_dir, 'src')
env['PROJECT_INCLUDE_DIR'] = os.path.join(project_dir, 'include')
env['LIBSOURCE_DIRS'].append(os.path.join(project_dir, 'lib'))
