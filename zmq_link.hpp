#ifndef __KATO_ZMQLINK_H__
#define __KATO_ZMQLINK_H__

#pragma once

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include "kato/log.hpp"

#define LINK_SHORT_SLEEP_US 1000

class ZMQLink
{
private:
    std::string address;
    const ushort port;
    zmq::context_t context;
    zmq::socket_t socket;
    std::atomic<bool> isBound{false};
    std::condition_variable condition;

public:
    std::atomic<bool> isListening{false};
    ZMQLink(const ushort _port = 5555, const std::string &_address = "127.0.0.1") : address(_address), port(_port), context(1), socket(context, zmq::socket_type::rep), isBound(false) {}
    ~ZMQLink()
    {
        isListening.store(false);
        condition.notify_one();
        if (isBound)
            teardownLink();
    }
    zmq::send_result_t Send(const std::string &_message)
    {
        zmq::message_t message(_message);
        return socket.send(message, zmq::send_flags::dontwait); // send init, does not block
    }
    std::string Receive()
    {
        // zmq::message_t recvMessage;
        // (void)socket.recv(recvMessage, zmq::recv_flags::dontwait);
        // return recvMessage.to_string();
        zmq::message_t recvMessage;
        if (socket.recv(recvMessage, zmq::recv_flags::dontwait))
            return recvMessage.to_string();
        return "";
    }
    void setupLink()
    {
        kato::log::cout << KATO_GREEN << "link.h::ZMQLink::" << "setupLink() " << "Settings up link at " + getURI() << KATO_RESET << std::endl;
        isBound.store(true);
        socket.bind(getURI().c_str());
    }
    void teardownLink()
    {
        socket.unbind(getURI().c_str());
        isBound.store(false);
        kato::log::cout << KATO_GREEN << "link.h::ZMQLink::" << "teardownLink() " << "Tearing down link at " + getURI() << KATO_RESET << std::endl;
    }
    const std::string getURI() const
    {
        return "tcp://" + address + ":" + std::to_string(port);
    }
};

// pkg-config --cflags --libs freetype2

#endif //__KATO_ZMQLINK_H__