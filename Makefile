CC=gcc
TARGET=pcc

all:
	$(CC) pcc.c packet.c cJSON.c -o $(TARGET)
	$(CC) pcc_view.c packet.c -o pcc_view

clean:
	rm $(TARGET)
