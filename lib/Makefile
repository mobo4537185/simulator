CC = gcc
SRC = .
include $(CFLAGS_FILE)

SRC_C_FILES += $(wildcard crc/*.c)


SRC_O_FILES = $(SRC_C_FILES:%.c=%.o)
DEPEND_FILES = src/.depend.d 

.INTERMEDIATE: $(SRC_O_FILES) 
    
.PHONY: all clean

all: libmylib

libmylib: $(SRC_O_FILES) $(DEPEND_FILES)
	ar  rcs $@.a $(SRC_O_FILES)

#%.o  : %.c
#	$(CC) $(CFLAGS) $< -o $@

#$(SRC_C_FILES:%.c=src/.depend/%.d)


clean:
	rm -f *.o
	rm -f *.a
	rm -f $(DEPEND_FILES)

include $(PROJECT_HOME)/rules.depend
sinclude $(DEPEND)
