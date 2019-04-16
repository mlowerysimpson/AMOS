// filedata.h: interface for the filedata class.
//
//////////////////////////////////////////////////////////////////////

#ifndef FILEDATA_H
#define FILEDATA_H

#include <cstdio>
#include <cstring>
#include <unistd.h>

class filedata  
{
public:
	int FindString(char *category, char *field);
	char * getFilename();
	static int getFileLength(char * filename);
	filedata(char *filename);
	filedata(unsigned char *buffer, int nSize);
	int getString(char *category, char *field, char **stringval);
	double getDouble(char *category, char *field);
	int getDouble(char *category, char *field, int numItems, double arrayItems[]);
	int getInteger(char *category, char *field);
	int getInteger(char *category, char *field,  int numItems, int arrayItems[]);
	int openDataFile(char *filename);
	int openReadOnlyDataFile(char *filename);
	void writeData(char *category, char *field, char *data);  //output a line containing a text description and data 
	void writeData(char *category, char *field, double data); //output a line containing a text description and decimal number
	void writeData(char *category, char *field, int numItems, double dataItems[]); //output a line containing a text description and an array of decimal numbers
	void writeData(char *category, char *field, int data); //output a line containing a text description and an integer number
	void writeData(char *category, char *field, int numItems, int dataItems[]); //output a line containing a text description and an array of integers
	void eraseData(char *category, char *field);//erase a line 
	int closeDataFile(); //writes the buffer data to the file previously opened, before closing the file
	

	virtual ~filedata();

protected:
	int charbuffer;
	int found_category;
	int nRead;
	int fileexists;
	int getNumDigits(int data);
	int getNumDigits(double data); //include the decimal point
	void insertToBuffer(int itemsize, char *itemBuffer, int insertionpoint); //inserts itemBuffer string into databuffer
	int eraseLine(int index); //erases a line of databuffer, starting at index -- returns number of characters erased
	void doWrite(char *datastring,int index); //performs writing to buffer

	int fileopened;
	int filedataindex;
	char *databuffer;
	//FILE * datafile;
	FILE *datafile;

private:
	char m_fileFolder[FILENAME_MAX];
	bool m_bWritten;
	char *m_filename;
};

#endif




