DIR_INC = ./include
DIR_SRC = ./src
DIR_OBJ = ./build
DIR_BIN = ./build

SRC = $(wildcard ${DIR_SRC}/*.c)  
OBJ = $(patsubst %.c,${DIR_OBJ}/%.o,$(notdir ${SRC})) 

TARGET = main

BIN_TARGET = ${DIR_BIN}/${TARGET}

CC = gcc
CFLAGS = -g -Wall -I${DIR_INC} 

${BIN_TARGET}:${OBJ}
	$(CC) $(OBJ)  -o $@ -lm
${DIR_OBJ}/%.o:${DIR_SRC}/%.c
	$(CC) $(CFLAGS) -c  $< -o $@ -lm
.PHONY:clean
clean:
	rm -r ${DIR_OBJ}/*
.PHONY:run 
run:
	./build/main

