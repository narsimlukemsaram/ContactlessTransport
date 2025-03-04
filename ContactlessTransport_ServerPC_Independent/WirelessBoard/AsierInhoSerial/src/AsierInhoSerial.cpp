#include <AsierInhoSerial.h>
#include <src/AsierInhoSerialImpl.h>

AsierInhoSerial::AsierInhoSerialBoard* AsierInhoSerial::createAsierInho() {
	AsierInhoSerialImpl* result = new AsierInhoSerialImpl();
	return result;
}

void (*_printMessage_AsierInho) (const char*) = NULL;
void (*_printError_AsierInho)(const char*) = NULL;
void (*_printWarning_AsierInho)(const char*) = NULL;

void AsierInhoSerial::printMessage(const char* str) {
	if (_printMessage_AsierInho) _printMessage_AsierInho(str);
}
void AsierInhoSerial::printError(const char* str) {
	if (_printError_AsierInho) _printError_AsierInho(str);
}
void AsierInhoSerial::printWarning(const char* str) {
	if (_printWarning_AsierInho) _printWarning_AsierInho(str);
}

void AsierInhoSerial::RegisterPrintFuncs(void(*p_Message)(const char*), void(*p_Warning)(const char*), void(*p_Error)(const char*)) {
	if (!_printMessage_AsierInho)_printMessage_AsierInho = p_Message;
	if (!_printError_AsierInho)_printError_AsierInho = p_Error;
	if (!_printWarning_AsierInho)_printWarning_AsierInho = p_Warning;
	printMessage("Got message functions!");
}