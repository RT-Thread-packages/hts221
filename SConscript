from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add hts221 src files.
src += Glob('sensor_st_hts221.c')
src += Glob('libraries/hts221.c')
src += Glob('libraries/hts221_reg.c')

# add hts221 include path.
path  = [cwd, cwd + '/libraries']

# add src and include to group.
group = DefineGroup('hts221', src, depend = ['PKG_USING_HTS221'], CPPPATH = path)

Return('group')
