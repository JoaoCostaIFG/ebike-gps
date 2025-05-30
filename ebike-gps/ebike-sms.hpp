#pragma once

#include "ebike-conf.h"
#include <memory>
#include "modem_utilities.h"
#include <TinyGsmClient.h>

class SMS
{
public:
    bool valid = false;
    String sender = "";
    String message = "";
};

bool deleteSMSByIndex(std::shared_ptr<TinyGsm> modem, int index);

SMS readSms(std::shared_ptr<TinyGsm> modem);