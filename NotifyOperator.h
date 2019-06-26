//NotifyOperator.h
//Interface for the NotifyOperator class
#pragma once

#include "twilio.hh"
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <vector>

#define MAX_EMAIL_ITEMS 16 //maximum number of items to include in email, eg. "to", "from", "date", "body", etc.

using namespace std;

struct upload_status {
	int lines_read;
};

class NotifyOperator {//class used for notifying operator of a particular condition of the boat, ex: if an alarm occurs
	//can be used to send texts or emails to operator(s)
public:
	NotifyOperator();//constructor
	~NotifyOperator();//destructor
	
	bool IssueNotification(char *msgText, char *szSubject, void *pShipLog);//send out text(s) and email(s) to configured recipients

private:
	//data
	bool m_bLoadedConfigInfo;//true if we were able to successfully load the configuration info from the prefs.txt file
	//texting parameters
	bool m_bSendText;//flag is true if texts should be sent
	char *m_twilio_account_sid;//Twilio account ID (NULL if not available or used), used for sending out texts 
	char *m_twilio_auth_token;//Twilio authorization token (NULL if not available or used), used for sending out texts
	vector <char *>m_toNumbers;//list of phone numbers to send text messages to
	char *m_fromNumber;//the Twilio assigned phone # for sending text messages
	
	//email parameters
	bool m_bSendEmail;//flag is true if email should be sent
	char *m_fromAddr;//where the email is from
	vector <char *>m_emailAddressees;//list of email addresses that we are sending messages to
	char *m_emailUsername;//username for the email account that is sending the email
	char *m_emailPassword;//password for the email account that is sending the email
	int m_nMailServerPortNumber;//the port number used by the mailserver
	char *m_mailServerName;//the name of the mailserver

	//functions
	char *GetEmailRecipientList();//gets a character string of all the email recipients. Calling function is responsible for deleting the returned string
	static size_t payload_source(void *ptr, size_t size, size_t nmemb, void *userp);
	bool LoadConfigInfo();//load configuration data from prefs.txt file, return true if file could be successfully opened and read
	bool SendText(char *msgText,void *pShipLog);//send text message to one or more recipients
	bool SendEmail(char *msgText, char *szSubject, void *pShipLog);//send email message to one or more recipients
};

