CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -pedantic -g -Iinclude
LDFLAGS = -lm

TARGET = drone_simulation

SRC_DIR = src
SOURCES = $(SRC_DIR)/main.c $(SRC_DIR)/simulation.c $(SRC_DIR)/drone.c $(SRC_DIR)/report.c $(SRC_DIR)/utils.c

OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
    $(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(SRC_DIR)/main.o: $(SRC_DIR)/main.c include/types.h include/simulation.h include/drone.h include/report.h include/utils.h
    $(CC) $(CFLAGS) -c -o $@ $<

$(SRC_DIR)/simulation.o: $(SRC_DIR)/simulation.c include/simulation.h include/types.h include/drone.h include/utils.h
    $(CC) $(CFLAGS) -c -o $@ $<

$(SRC_DIR)/drone.o: $(SRC_DIR)/drone.c include/drone.h include/types.h include/utils.h
    $(CC) $(CFLAGS) -c -o $@ $<

$(SRC_DIR)/report.o: $(SRC_DIR)/report.c include/report.h include/types.h include/utils.h
    $(CC) $(CFLAGS) -c -o $@ $<

$(SRC_DIR)/utils.o: $(SRC_DIR)/utils.c include/utils.h include/types.h
    $(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
    ./$(TARGET) sample_figure.txt

clean:
    rm -f $(TARGET) $(OBJECTS) $(SRC_DIR)/*.o simulation_report.txt