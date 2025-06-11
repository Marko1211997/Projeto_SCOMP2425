CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -D_GNU_SOURCE
LDFLAGS = -lrt -lpthread -lm

TARGET = drone_simulation
SOURCES = simulationSprint2.c

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)

clean:
	@echo "Cleaning up..."
	@pkill -f $(TARGET) 2>/dev/null || true
	@sleep 1
	@ipcrm -M /drone_simulation_shm 2>/dev/null || true
	@ipcrm -S /step_semaphore 2>/dev/null || true
	@ipcrm -S /barrier_semaphore 2>/dev/null || true
	@rm -f $(TARGET)
	@rm -f *.txt
	@rm -f simulation_report.txt
	@echo "Cleanup complete!"

force-clean: clean
	@echo "Force cleaning..."
	@killall -9 $(TARGET) 2>/dev/null || true
	@sleep 2

run: $(TARGET)
	./$(TARGET) sample_1_figure.txt

debug: $(TARGET)
	gdb ./$(TARGET)

rebuild: clean all

.PHONY: all clean force-clean run debug rebuild