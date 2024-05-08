#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>

struct AAA
{
    int direction;
    float deepth;
    uint32_t crc;
};

struct BBB
{
    float yaw;
    float pitch;
    float roll;
    uint32_t crc;
};

class UDPTest
{
public:
    UDPTest(bool is_send);
    void UDPRecv();
    void UDPSend();

private:
    UNITREE_LEGGED_SDK::UDP udp;
    AAA a;
};

UDPTest::UDPTest(bool is_send) : udp(is_send ? 8019 : 8020, "127.0.0.1", is_send ? 8020 : 8019, sizeof(AAA), sizeof(BBB))
{
    if (is_send)
    {
        std::cout << "send udp. " << std::endl;
    }
    else
    {
        std::cout << "receive udp. " << std::endl;
    }
}

void UDPTest::UDPRecv()
{
    udp.Recv();
    udp.GetRecv((char *)&a);
    std::cout << "deepth received: " << a.deepth << std::endl;
}

void UDPTest::UDPSend()
{
    udp.SetSend((char*)&a);
    udp.Send();
    std::cout << "deepth send: " << a.deepth << std::endl;
    a.deepth += 1.0;
}

int main(int argc, char **argv)
{
    bool is_send = !std::strcmp(argv[argc - 1], "send");
    UDPTest udp_test(is_send);
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send", 0.1, 3, boost::bind(&UDPTest::UDPSend, &udp_test));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv", 0.1, 2, boost::bind(&UDPTest::UDPRecv, &udp_test));
    if (is_send)
    {
        loop_udpSend.start();
    }
    else
    {
        loop_udpRecv.start();
    }

    while (1)
    {
        sleep(20);
    };
    return 0;
}
