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

static void deleteSMSByIndex(std::shared_ptr<TinyGsm> modem, int index)
{
    EBIKE_NFO("Deleting SMS at index: ", index);
    modem->sendAT("+CMGD=" + String(index));
    if (!modem->waitResponse())
    {
        EBIKE_ERR("Failed to delete SMS.");
    }
    else
    {
        EBIKE_NFO("SMS deleted successfully.");
    }
}

SMS readSms(std::shared_ptr<TinyGsm> modem)
{
    SMS ret;

    String data = "";
    modem->sendAT("+CMGR=1"); // index 1 is the latest message
    if (!modem->waitResponse(1000, data))
    {
        EBIKE_ERR("Read SMS failed");
        return ret;
    }

    data.replace("\r\nOK\r\n", "");
    data.replace("\rOK\r", "");
    data.trim();
    EBIKE_NFO("Last Data: ", data);

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

        // Only process non-empty lines (e.g., if there were consecutive newlines)
        if (line.length() > 0)
        {
            if (lineNumber == 1)
            {
                ret.sender = getSmsSender(data);
                if (ret.sender.length() > 0)
                {
                    EBIKE_NFO("SMS Sender: ", ret.sender);
                }
                else
                {
                    EBIKE_ERR("Failed to parse SMS sender");
                }
            }
            lineNumber++;
        }
    }

    return ret;
}