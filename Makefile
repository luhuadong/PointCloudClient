CC=gcc
TARGET=pcc

all:
	$(CC) pcc.c cJSON.c -o $(TARGET)
	$(CC) pcc_s.c -o pcc_s

clean:
	rm $(TARGET)
