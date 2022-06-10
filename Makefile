CC=gcc
TARGET=pcc

all:
	$(CC) pcc.c packet.c cJSON.c -o $(TARGET)
	$(CC) pcc2.c packet.c -o pcc2

clean:
	rm $(TARGET)
