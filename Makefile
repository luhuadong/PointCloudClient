CC=gcc
TARGET=pcc

all:
	$(CC) pcc.c packet.c cJSON.c -o $(TARGET)

clean:
	rm $(TARGET)
