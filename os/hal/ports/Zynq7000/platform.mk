# List of all the Zynq7000 platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/Zynq7000/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/st_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/wdg_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/pal_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/ext_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/serial_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/spi_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/gpt_lld.c \
              ${CHIBIOS}/os/hal/ports/Zynq7000/gic.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/Zynq7000
