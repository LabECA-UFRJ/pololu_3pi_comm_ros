/*!
 * \file SimpleSerial.h
 *
 * \author Terraneo Federico
 *
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 */

#ifndef _SIMPLESERIAL_H_
#define	_SIMPLESERIAL_H_

#include <boost/asio.hpp>

class SimpleSerial
{
	public:
	    	/// Default constructor
		SimpleSerial() : io() {serial = NULL;}

		/*!
		 * \brief This function is used to open a connection with a serial port.
		 *
		 * \param port Device name, example "/dev/ttyUSB0" or "COM4".
		 * \param baud_rate Communication speed, example 9600 or 115200.
		 *
		 * \throws boost::system::system_error if cannot open the
		 * 	serial device
		 */
		void open(std::string port, unsigned int baud_rate)
		{
			serial = new boost::asio::serial_port(io, port);
			serial->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
		}

		~SimpleSerial() {if (serial == NULL) delete serial;}

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
        boost::asio::write(*serial,boost::asio::buffer(s.c_str(),s.size()));
    }

	/**
     * Write a char to the serial device.
     * \param s char to write
     * \throws boost::system::system_error on failure
     */
    void write(const char s)
    {
    	boost::asio::write(*serial, boost::asio::buffer(&s, 1));
    }

    void write(const unsigned char* data, int length)
    {
        boost::asio::write(*serial, boost::asio::buffer(data, length));
    }

    /*!
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine()
    {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(*serial,asio::buffer(&c,1));
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port* serial;
};

#endif	/* _SIMPLESERIAL_H */

