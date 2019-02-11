#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
// #include <xsens/xstime.h>
// #include <xcommunication/legacydatapacket.h>
// #include <xcommunication/int_xsdatapacket.h>
// #include <xcommunication/enumerateusbdevices.h>

// #include "deviceclass.h"

// #include <iostream>
// #include <iomanip>
// #include <stdexcept>
// #include <string>

// #ifdef __GNUC__
// #include "conio.h" // for non ANSI _kbhit() and _getch()
// #else
// #include <conio.h>
// #endif

// int ConfigureXsens(){
	// DeviceClass device;

	// try{
		// Scan for connected USB devices
		// std::cout << "Scanning for USB devices..." << std::endl;
		// XsPortInfoArray portInfoArray;
		// xsEnumerateUsbDevices(portInfoArray);
		// if (!portInfoArray.size()){
			// std::string portName;
			// int baudRate;
			// std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. /dev/ttyUSB0): " <<
			// std::endl;
			// std::cin >> portName;
			// std::cout << "Please enter baud rate (eg. 115200): ";
			// std::cin >> baudRate;

			// XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
			// portInfoArray.push_back(portInfo);
		// }
	// }
	// catch (std::runtime_error const & error){
		// std::cout << error.what() << std::endl;
	// }
	// catch (...){
		// std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	// }

	// std::cout << "Successful exit." << std::endl;

	// std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

	// return 0;
// }