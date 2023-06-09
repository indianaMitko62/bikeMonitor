#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

int readFile(fs::FS &fs, const char * path)
{
	File file = fs.open(path);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	Serial.print("Read from file: ");
	while(file.available())
	{
		Serial.write(file.read());
	}
	return 0;
	file.close();
}

int writeFile(fs::FS &fs, const char * path, const char * message)
{
	File file = fs.open(path, FILE_WRITE);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	if(file.print(message))
	{
		Serial.println("write successful");
	} 
	else 
	{
		Serial.println("write failed");
		return 1;
	}
	file.close();
	return 0;
}

int appendFile(fs::FS &fs, const char * path, const char * message)
{
	File file = fs.open(path, FILE_APPEND);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	if(file.print(message))
	{
		Serial.println("Message appended");
	} 
	else 
	{
		Serial.println("Append failed");
		return 1;
	}
	file.close();
	return 0;
}

int renameFile(fs::FS &fs, const char * path1, const char * path2)
{
	if (fs.rename(path1, path2))
	{
		Serial.println("File renamed");
		return 0;
	} 
	else 
	{
		Serial.println("Rename failed");
		return 1;
	}
}

int deleteFile(fs::FS &fs, const char * path)
{
if(fs.remove(path))
{
	Serial.println("File deleted");
	return 0;
} 
else 
{
	Serial.println("Delete failed");
	return 1;
}
}

void printSDInfo()
{
	uint8_t cardType = SD.cardType();

	if(cardType == CARD_NONE)
	{
		Serial.println("No SD card attached");
		return;
	}

	Serial.print("SD Card Type: ");
	if(cardType == CARD_MMC)
	{
		Serial.println("MMC");
	} 
	else if(cardType == CARD_SD)
	{
		Serial.println("SDSC");
	} 
	else if(cardType == CARD_SDHC)
	{
		Serial.println("SDHC");
	}
	else 
	{
		Serial.println("UNKNOWN");
	}

	uint64_t cardSize = SD.cardSize() / (1024 * 1024);
	Serial.printf("SD Card Size: %lluMB\n", cardSize);

}
void setup()
{
	Serial.begin(115200);
	if(!SD.begin(5))
	{
		Serial.println("Card Mount Failed");
		return;
	}

	printSDInfo();

	writeFile(SD, "/hello.txt", "Hello ");
	appendFile(SD, "/hello.txt", "World!\n");
	readFile(SD, "/hello.txt");
	deleteFile(SD, "/foo.txt");
	renameFile(SD, "/hello.txt", "/foo.txt");
	readFile(SD, "/foo.txt");
	Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
	Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void loop()
{

}