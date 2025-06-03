#include "ebike-sms.hpp"
#include "ebike-log.hpp"

/*
AT+CMGR=1
+CMGR: "REC READ","+351931963308","","25/05/29,21:55:56+4"
helo
*/

static String getSmsSender(const String &line)
{
    // Parse the +CMGR header line to get sender number
    // Example: +CMGR: "REC UNREAD","+1234567890","","24/05/29,10:30:00+00"
    int firstQuote = line.indexOf('"');
    int secondQuote = line.indexOf('"', firstQuote + 1);
    int thirdQuote = line.indexOf('"', secondQuote + 1);
    int fourthQuote = line.indexOf('"', thirdQuote + 1);
    if (thirdQuote != -1 && fourthQuote != -1)
    {
        return line.substring(thirdQuote + 1, fourthQuote);
    }
    return "";
}

bool deleteSMSByIndex(std::shared_ptr<TinyGsm> modem, int index)
{
    modem->sendAT("+CMGD=" + String(index));
    return modem->waitResponse();
}

bool readSms(std::shared_ptr<TinyGsm> modem, SMS &sms)
{
    String data = "";
    modem->sendAT("+CMGR=1"); // index 1 is the latest message
    EBIKE_NFO("Getting SMS");
    if (!modem->waitResponse(5000, data))
    {
        EBIKE_ERR("Read SMS failed");
        return false;
    }
    EBIKE_DBG("SMS: ", data);

    data.replace("\r\nOK\r\n", "");
    data.replace("\rOK\r", "");
    data.trim();

    int startIndex = 0;
    int endIndex = 0;
    int lineNumber = 0;
    // Loop to find and extract lines
    while (startIndex < data.length())
    {
        endIndex = data.indexOf('\n', startIndex);

        String line;
        if (endIndex == -1)
        {
            // No more newlines, this is the last line
            line = data.substring(startIndex);
            startIndex = data.length();
        }
        else
        {
            // Extract the substring from startIndex to endIndex
            line = data.substring(startIndex, endIndex);
            startIndex = endIndex + 1; // Move startIndex past the newline character
        }

        // Handle \r
        if (line.length() > 0 && line.charAt(line.length() - 1) == '\r')
        {
            line.remove(line.length() - 1);
        }

        line.trim();
        if (line.length() > 0)
        {
            switch (lineNumber)
            {
            case 1:
                sms.sender = getSmsSender(data);
                break;
            case 2:
                sms.message = line;
                sms.valid = true;
                break;
            default:
                break;
            }
            lineNumber++;
        }
    }

    return true;
}