#pragma once

#include <memory>
#include "modem_utilities.h"
#include <TinyGsmClient.h>

class SMS
{
public:
    String sender = "";
    String message = "";
};

SMS readSms(std::shared_ptr<TinyGsm> modem);