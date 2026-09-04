# Processor frequency
F_CPU = 16000000

CUSTOM_MATRIX    = yes

# M0100 quadrature mouse (see m0100_mouse.c)
POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_DRIVER = custom

SRC = matrix.c m0110.c m0100_mouse.c
