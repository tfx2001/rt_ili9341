from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# # add ssd1306 src files.
# if GetDepend('PKG_USING_SSD1306'):
src += Glob('src/ili9341.c')

# add ssd1306 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('ili9341', src, depend = [''], CPPPATH = path)

Return('group')