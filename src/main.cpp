#include "ros/ros.h"
#include "protocol_msgs/Packet.h"

#include "SimpleSerial.h"

#include <iostream>

using namespace std;

class Reader
{
public:
    Reader(unsigned char* data) 
    {
        m_Data = data;
        m_Index = 0;
    }

    void Reset()
    {
        m_Index = 0;
    }

    void Skip(int count)
    {
        m_Index += count;
    }

    uint8_t ReadByte()
    {
        uint8_t value = m_Data[m_Index];
        m_Index++;
        return value;
    }

private:
    unsigned char* m_Data;
    int m_Index;
};

class Writer
{
public:
    Writer(unsigned char* data, int length) 
    {
        m_Data = data;
        m_Capacity = length;
        m_Length = 0;
        m_Index = 0;
    }

    int Length()
    {
        return m_Length;
    }

    int Capacity()
    {
        return m_Capacity;
    }

    void Reset()
    {
        m_Length = 0;
        m_Index = 0;
    }

    void Seek(int pos)
    {
        m_Index = pos;
    }

    void WriteByte(unsigned char c)
    {
        m_Data[m_Index] = c;
        m_Index++;
        m_Length++;
    }

    void WriteShort(uint16_t value)
    {
        m_Data[m_Index + 0] = (uint8_t)(value >> 8) & 0xFF;
        m_Data[m_Index + 1] = (uint8_t)(value >> 0) & 0xFF;
        m_Index += 2;
        m_Length += 2;
    }

    void WriteInt(int32_t value)
    {
        m_Data[m_Index + 0] = (uint8_t)(value >> 24) & 0xFF;
        m_Data[m_Index + 1] = (uint8_t)(value >> 16) & 0xFF;
        m_Data[m_Index + 2] = (uint8_t)(value >> 8) & 0xFF;
        m_Data[m_Index + 3] = (uint8_t)(value >> 0) & 0xFF;
        m_Index += 4;
        m_Length += 4;
    }

    void WriteBytes(const uint8_t* bytes, int count)
    {
        std::memcpy(&m_Data[m_Index], bytes, count);
        m_Index += count;
        m_Length += count;
    }

    void Skip(int count)
    {
        m_Index += count;
        m_Length += count;
    }

private:
    unsigned char* m_Data;
    int m_Capacity;
    int m_Length;
    int m_Index;
};

class Xbee
{
public:
    Xbee()
    {
        m_Reader = new Reader(m_Data);
        m_Writer = new Writer(m_Data, 64);
    }

    ~Xbee()
    {
        delete m_Writer;
        delete m_Reader;
    }

    int CreateMessage(const int32_t addressHigh, const int32_t addressLow, std::vector<uint8_t> payload)
    {
        m_Writer->Reset();

        m_Writer->WriteByte(0x7E); // Start delimiter.
        m_Writer->WriteShort((uint16_t)11 + (uint16_t)payload.size());

        m_Writer->WriteByte(0x00); // Frame type.
        m_Writer->WriteByte(0x00); // Frame id (always 0 if no response required).

        m_Writer->WriteInt(addressHigh); // Street.
        m_Writer->WriteInt(addressLow);  // City.

        m_Writer->WriteByte(0x01); // Options : Disable ACK.

        m_Writer->WriteBytes(payload.data(), payload.size()); // PUSH THE PAYLOAD!!
        m_Writer->WriteByte(ComputeChecksum()); // Checksum.

        // DEBUG
        m_Reader->Reset();

        cout << "Message size: " << m_Writer->Length() << endl;

        for (int i = 0; i < m_Writer->Length(); i++)
        {
            cout << hex << static_cast<int>(m_Reader->ReadByte()) << " ";
        }
        cout << flush;

        return m_Writer->Length();
    }

    uint8_t ComputeChecksum()
    {
        m_Reader->Reset();
        m_Reader->Skip(3);

        int length = m_Writer->Length();
        uint8_t checksum = 0xFF;

        for (int i = 0; i < length; i++) {
            checksum -= m_Reader->ReadByte();
        }

        return checksum;
    }

    void packetCallback(const protocol_msgs::Packet::ConstPtr& packet)
    {
        int length = CreateMessage(packet->addressHigh, packet->addressLow, packet->data);
        m_Serial->write(m_Data, length);
    }

    void SetSerial(SimpleSerial* serial)
    {
        m_Serial = serial;
    }
    
private:
    Reader* m_Reader;
    Writer* m_Writer;
    SimpleSerial* m_Serial;
    unsigned char m_Data[64];
};

int main(int argc, char** argv)
{
    Xbee xbee;
    SimpleSerial serial;

    ros::init(argc, argv, "serial_xbee");
    ros::NodeHandle nodeHandle;

    string port; 
    nodeHandle.param<std::string>("port", port, "/dev/ttyUSB0");

    int baud_rate;
    nodeHandle.param<int>("baud", baud_rate, 9600);

    serial.open(port, baud_rate);
    xbee.SetSerial(&serial);

    ros::Subscriber subscriber = nodeHandle.subscribe<protocol_msgs::Packet>("packet", 1000, &Xbee::packetCallback, &xbee);

    ros::spin();

    return 0;
}