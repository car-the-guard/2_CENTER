# ===== Toolchain (D3-G Cross Compiler) =====
CC      = aarch64-linux-gnu-gcc
CFLAGS  = -Wall -O2 -pthread -Iinclude
LDFLAGS = -pthread -lm

# ===== Output =====
TARGET = app_main

# ===== Source Files =====
SRC = \
    src/daemon2_real.c \
    src/accident_send_real.c \
    src/bluetooth_interface_real.c \
    src/can_interface_real.c \
    src/collision_response_real.c \
    src/collision_risk_real.c \
    src/driving_info_real.c \
    src/uart_interface_real.c 

# ===== Object Files =====
OBJ = $(SRC:.c=.o)

# ===== Build Rules =====
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean