#common makefile header

DIR_INC = ../../include
DIR_BIN = ../../bin
DIR_LIB = ../../libs

TARGET	= tts_offline_sample
BIN_TARGET = $(DIR_BIN)/$(TARGET)

CROSS_COMPILE = 
CFLAGS = -g -Wall -I$(DIR_INC)

LDFLAGS := -L$(DIR_LIB)/

LDFLAGS += -lmsc -lrt -ldl -lpthread -lstdc++

OBJECTS := $(patsubst %.c,%.o,$(wildcard *.c))

$(BIN_TARGET) : $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o : %.c
	$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@
clean:
	@rm -f *.o $(BIN_TARGET)

.PHONY:clean

#common makefile foot
