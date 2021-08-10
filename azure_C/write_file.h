#pragma once
#include <iostream>
#include <filesystem>
#include <fstream>

//using namespace std;
//namespace fs = std::filesystem;

long WriteToFile(const char* fileName, void* buffer, size_t bufferSize) {
	std::cout << bufferSize << std::endl;

	std::ofstream hFile;
	hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);

	if (hFile.is_open())
	{
		hFile.write((char*)buffer, static_cast<std::streamsize>(bufferSize));
		hFile.close();
	}

	std::cout << "[Streaming Service] Color frame is stored in " << fileName << std::endl;

	return 0;
}