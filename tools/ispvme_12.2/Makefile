CC = gcc
AR = ar
CFLAGS = -Wall -W -Wunused -lpthread -g -O2 -ggdb
#LFLAGS = -lm -pthread -DVME_DEBUG
LFLAGS = -lm -pthread
DEL_FILE = rm -f
MV_FILE = mv -f
OBJ_FILE_NODE = ./*.o
#VPATH = 
INCPATH     =     -I../include/
TARGET   = ispvm

OBJECTS += hardware.o ispvm_ui.o ivm_core.o


.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<
$(TARGET):$(OBJECTS)
	$(CC) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	
clean:
	-$(DEL_FILE) $(TARGET)
	-$(DEL_FILE) $(OBJ_FILE_NODE)


