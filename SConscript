
from building import *
import rtconfig

cwd = GetCurrentDir()

src = Glob('*.c')
CPPPATH = [cwd]
LOCAL_CCFLAGS = ''

group = DefineGroup('isl29035', src, depend = ['PKG_USING_ISL29035'], CPPPATH = CPPPATH, LOCAL_CCFLAGS = LOCAL_CCFLAGS)

Return('group')
