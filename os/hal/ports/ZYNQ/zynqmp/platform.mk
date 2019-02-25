# List of all the ZYNQ7000 platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/ZYNQ/common/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/zynqmp/st_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/pal_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/ext_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/serial_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/spi_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/gpt_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/i2c_lld.c \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common/gic.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/ZYNQ/zynqmp \
              ${CHIBIOS}/os/hal/ports/ZYNQ/common \
